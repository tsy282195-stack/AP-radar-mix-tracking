function test_passive_async_multimeas()
%TEST_PASSIVE_ASYNC_MULTIMEAS Regression checks for pure-passive events and
% robust same-source multi-bearing updates.

test_async_only_configuration();
test_no_frontend_active_passive_fusion();
test_pure_passive_microbatch();
test_los_fusion_wrap();
test_filter_multi_bearing_update();
fprintf('test_passive_async_multimeas: PASS\n');
end

function test_async_only_configuration()
cfg = config_fusion();
obsolete = {'filter_input_mode', 'passive_only_trust', ...
    'passive_range_interp_method', 'passive_range_angle_gate_deg', ...
    'passive_default_range_m', 'passive_range_sigma_scale', ...
    'passive_time_align_enabled', 'passive_align_to', ...
    'passive_align_mode', 'passive_interp_method', ...
    'passive_interp_max_gap_s', 'passive_tracklet_gate_deg', ...
    'passive_tracklet_gate_growth_degps', 'passive_tracklet_max_gap_s', ...
    'passive_tracklet_min_hits', 'passive_tracklet_amb_margin', ...
    'passive_tracklet_amb_abs_deg', 'passive_tracklet_fit_window', ...
    'passive_tracklet_predict_max_dt_s', 'passive_align_extrap_max_s', ...
    'passive_R_time_inflate', 'passive_R_time_rate_degps', ...
    'passive_align_fallback', 'ap_corr_gate_deg', 'ap_corr_gate_az_deg', ...
    'ap_corr_gate_el_deg', 'ap_corr_unmatched_cost', ...
    'ap_corr_assignment_scope', 'ap_angle_fusion_mode'};
assert(~any(isfield(cfg, obsolete)));
end

function test_no_frontend_active_passive_fusion()
cfg = struct('async_microbatch_dt_s', 0.005, ...
    'async_active_time_mode', 'plot', ...
    'async_passive_count_miss', false, ...
    'passive_bearing_update_on_pure', true, ...
    'sigma_passive_az_deg', 0.05, ...
    'sigma_passive_el_deg', 0.05);
frame = empty_frame();
frame.t_sec = 20;
frame.active_t = 20;
frame.active_xyz = [100; 200; 300];
frame.active_R = diag([4, 9, 16]);
frame.target_ids = 7;
frame.active_src = 1;
frame.passive_t = 20.001;
frame.passive_ang = [12; 3];
frame.passive_src = 2;
frame.passive_ids = 7;

[xyz, R, info, pb, ids, times, meta] = build_async_measurement_events(frame, cfg);
assert(numel(times) == 1);
assert(isequal(xyz{1}, frame.active_xyz));
assert(isequal(R{1}, frame.active_R));
assert(isequal(ids{1}, frame.target_ids));
assert(isequal(pb{1}.ang_deg, frame.passive_ang));
assert(meta(1).has_active && meta(1).has_passive);
assert(info.n_active == 1 && info.n_passive == 1 && info.n_events == 1);
assert(~isfield(info, 'n_paired') && ~isfield(info, 'n_passive_only'));
end

function test_pure_passive_microbatch()
cfg = struct('async_microbatch_dt_s', 0.005, ...
    'async_active_time_mode', 'frame', ...
    'async_passive_count_miss', false, ...
    'passive_bearing_update_on_pure', true, ...
    'sigma_passive_az_deg', 0.05, ...
    'sigma_passive_el_deg', 0.05);
frame = empty_frame();
frame.t_sec = 10;
frame.passive_t = [10, 10.002];
frame.passive_ang = [5.00, 5.03; 1.00, 0.99];
frame.passive_src = [2, 2];

[xyz, ~, info, pb, ~, times, meta] = build_async_measurement_events(frame, cfg);
assert(numel(times) == 1);
assert(isempty(xyz{1}));
assert(pb{1}.n_meas == 2);
assert(~meta(1).has_active && meta(1).has_passive);
assert(~meta(1).miss_cycle && ~meta(1).confirm_cycle);
assert(info.keep_pure_passive_events);

% The previous restrictive behavior remains explicitly selectable.
cfg.passive_bearing_update_on_pure = false;
[~, ~, info_off, ~, ~, times_off] = build_async_measurement_events(frame, cfg);
assert(isempty(times_off));
assert(~info_off.keep_pure_passive_events);
end

function test_los_fusion_wrap()
cfg = struct('passive_meas_fuse_weight', 'equal', ...
    'passive_meas_fuse_R_scale', 1.5);
R0 = diag([0.05^2, 0.05^2]);
R = repmat(R0, 1, 1, 2);
[z, Rf, info] = fuse_passive_bearing_group( ...
    [359.99, 0.01; 2.00, 2.00], R, [0, 0], cfg);
assert(abs(angle_diff(z(1), 0)) < 1e-6);
assert(abs(z(2) - 2) < 1e-6);
assert(info.n_used == 2 && info.fused);
assert(all(abs(info.mean_weight - [0.5, 0.5]) < 1e-12));
assert(all(eig(Rf) > 0));
assert(trace(Rf) < trace(R0));
end

function test_filter_multi_bearing_update()
cfg = config_fusion();
cfg.M_confirm = 1;
cfg.N_confirm = 2;
cfg.use_imm = false;
cfg.vel_trend_enabled = false;
cfg.use_target_id_prior = false;
cfg.adapt_R_enabled = false;
cfg.meas_robust_enabled = false;
cfg.birth_suppress_gated = false;
cfg.passive_bearing_update_on_pure = true;
cfg.passive_bearing_update_on_active = true;
cfg.passive_bearing_update_active_hit_tracks = false;
cfg.passive_bearing_min_dt_s = 0;
cfg.passive_meas_fuse_enabled = true;
cfg.passive_meas_fuse_weight = 'equal';
cfg.passive_meas_fuse_max_count = 4;
cfg.passive_meas_fuse_max_spread_deg = 0.35;
cfg.passive_bearing_gate = 16;
cfg.passive_bearing_nis_gate = 16;

target = [0; 10000; 0];
R_active = diag([50^2, 50^2, 50^2]);
xyz = {target, target, zeros(3, 0)};
R = {R_active, R_active, zeros(3, 3, 0)};
ids = {1, 1, zeros(1, 0)};
times = [0; 1; 1.1];
meta = repmat(struct('has_active', true, 'has_passive', false, ...
    'miss_cycle', true, 'confirm_cycle', true, 'n_active', 1, ...
    'n_passive', 0, 't_start', 0, 't_end', 0), 3, 1);
meta(2).t_start = 1; meta(2).t_end = 1;
meta(3).has_active = false;
meta(3).has_passive = true;
meta(3).miss_cycle = false;
meta(3).confirm_cycle = false;
meta(3).n_active = 0;
meta(3).n_passive = 2;
meta(3).t_start = 1.1; meta(3).t_end = 1.1;

pb = cell(3, 1);
pb{1} = empty_bearing(0);
pb{2} = empty_bearing(1);
Rb = repmat(diag([0.05^2, 0.05^2]), 1, 1, 2);
pb{3} = struct('t_sec', 1.1, 'ang_deg', [-0.02, 0.02; 0, 0], ...
    'R_deg2', Rb, 'src', [7, 7], 'tracklet_id', [1, 2], 'n_meas', 2);

platform = struct('lat_deg', 30, 'lon_deg', 110, 'alt_m', 1000, ...
    'interp_lat', @(t) 30 + 0*t, ...
    'interp_lon', @(t) 110 + 0*t, ...
    'interp_alt', @(t) 1000 + 0*t);

est = run_filter_adapt_ckf(xyz, R, times, cfg, pb, platform, ids, meta);
assert(est.N(1) == 1); % 出生主动量测计第1次，M=1时当帧立即确认
assert(est.N(3) == 1);
assert(est.passive_bearing_stats.n_updates == 1);
assert(est.passive_bearing_stats.n_fused_updates == 1);
assert(est.passive_bearing_stats.n_measurements_used == 2);
assert(est.tracks{3}.miss(1) == 0);
assert(sum(est.tracks{3}.S(:, 1) == 1) == 2); % 两个主动命中；纯被动事件未推进M/N
assert(all(eig(est.tracks{3}.P(:, :, 1)) > 0));
end

function pb = empty_bearing(t)
pb = struct('t_sec', t, 'ang_deg', zeros(2, 0), ...
    'R_deg2', zeros(2, 2, 0), 'src', zeros(1, 0), ...
    'tracklet_id', zeros(1, 0), 'n_meas', 0);
end

function f = empty_frame()
f = struct('t_sec', 0, 'active_xyz', zeros(3, 0), ...
    'active_rae', zeros(3, 0), 'active_R', zeros(3, 3, 0), ...
    'passive_ang', zeros(2, 0), 'passive_src', zeros(1, 0), ...
    'active_src', zeros(1, 0), 'target_ids', zeros(1, 0), ...
    'passive_ids', zeros(1, 0), 'active_t', zeros(1, 0), ...
    'passive_t', zeros(1, 0));
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end
