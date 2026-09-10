function test_track_birth_lifecycle()
%TEST_TRACK_BIRTH_LIFECYCLE  航迹新生、主动验收、M/N确认和同拍被动更新回归测试。

test_three_hits_confirm_on_third_frame();
test_five_of_ten_confirms_on_fifth_hit();
test_neighboring_new_target_is_not_swallowed();
test_distinct_ids_prevent_close_return_fusion();
test_confirmed_track_is_not_auto_merged_with_tentative();
test_redundant_returns_still_fuse();
test_nis_rejection_becomes_low_priority_conflict();
test_confirmed_track_wins_over_conflict_tentative();
test_velocity_initialization_retries();
test_same_event_passive_updates_newborn();
test_same_frame_birth_candidates_are_not_suppressed();
test_tentative_track_times_out_before_late_hit();
test_confirmed_track_timeout_boundary();
test_seconds_timeout_overrides_short_coast_limit();
test_passive_update_extends_confirmed_wait();
fprintf('test_track_birth_lifecycle: PASS\n');
end

function test_five_of_ten_confirms_on_fifth_hit()
cfg = base_cfg();
cfg.M_confirm = 5;
cfg.N_confirm = 10;
xyz = cell(5, 1);
R = cell(5, 1);
for k = 1:5
    xyz{k} = [100 * (k - 1); 0; 0];
    R{k} = cov3(1);
end
times = (0:4)';

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(isequal(est.N(:).', [0, 0, 0, 0, 1]));
assert(sum(est.tracks{5}.S(:, 1) == 1) == 5);
end

function test_three_hits_confirm_on_third_frame()
cfg = base_cfg();
cfg.M_confirm = 3;
cfg.N_confirm = 5;
xyz = {[0; 0; 0], [100; 0; 0], [200; 0; 0]};
R = {cov3(1), cov3(1), cov3(1)};
ids = {7, 7, 7};
times = [0; 1; 2];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(isequal(est.N(:).', [0, 0, 1]));
assert(est.L{3}(1, 2) == 1);
assert(sum(est.tracks{3}.S(:, 1) == 1) == 3);

metrics = evaluate_fusion_quant_metrics([], xyz, ids, est, times, cfg);
assert(metrics.start_time.n_started == 1);
assert(abs(metrics.start_time.records(1).first_confirmed_time - 2) < 1e-12);
assert(abs(metrics.start_time.records(1).track_start_delay_s - 2) < 1e-12);
assert(metrics.association.n_assoc_confirmed_tracks == 3);
assert(abs(metrics.association.rate_confirmed_tracks - 1) < 1e-12);
end

function test_neighboring_new_target_is_not_swallowed()
cfg = base_cfg();
cfg.birth_suppress_gated = true; % 明确释放的次回波仍须进入新生
xyz = {[0; 0; 0], [0, 2000; 0, 0; 0, 0]};
R = {cov3(1), cov3(2)};
ids = {1, [1, 2]};
times = [0; 1];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(size(est.tracks{2}.L, 2) == 2);
assert(numel(unique(est.assoc{2}.id)) == 2);
assert(any(est.assoc{2}.id == 1) && any(est.assoc{2}.id == 2));
end

function test_distinct_ids_prevent_close_return_fusion()
cfg = base_cfg();
cfg.use_target_id_prior = true;
xyz = {[0; 0; 0], [0, 50; 0, 0; 0, 0]};
R = {cov3(1), cov3(2)};
ids = {1, [1, 2]};
times = [0; 1];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(size(est.tracks{2}.L, 2) == 2);
assert(numel(unique(est.assoc{2}.id)) == 2);
end

function test_confirmed_track_is_not_auto_merged_with_tentative()
cfg = base_cfg();
cfg.M_confirm = 1;
cfg.N_confirm = 1;
cfg.use_target_id_prior = true;
cfg.merge_pos_dist_m = 100;
xyz = {[0; 0; 0], [0, 50; 0, 0; 0, 0]};
R = {cov3(1), cov3(2)};
ids = {1, [1, 2]};
times = [0; 1];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(size(est.tracks{2}.L, 2) == 2 && ...
       numel(unique(est.tracks{2}.L(2, :))) == 2, ...
    'A confirmed legacy track was automatically merged with a nearby tentative target.');
end

function test_redundant_returns_still_fuse()
cfg = base_cfg();
cfg.meas_fuse_weight = 'equal';
xyz = {[0; 0; 0], [0, 50; 0, 0; 0, 0]};
R = {cov3(1), cov3(2)};
ids = {1, [1, 1]};
times = [0; 1];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(size(est.tracks{2}.L, 2) == 1);
assert(numel(est.assoc{2}.id) == 2);
assert(all(est.assoc{2}.id == 1));
end

function test_nis_rejection_becomes_low_priority_conflict()
cfg = base_cfg();
cfg.birth_suppress_gated = true;
xyz = {[0; 0; 0], [5000; 0; 0]};
R = {cov3(1), cov3(1)};
ids = {1, 2};
times = [0; 1];

est = run_filter_adapt_ckf(xyz, R, times, cfg, [], [], ids);
assert(size(est.tracks{2}.L, 2) == 2);
old_idx = find(est.tracks{2}.L(2, :) == 1, 1);
new_idx = find(est.tracks{2}.L(2, :) == 2, 1);
assert(~isempty(old_idx) && ~isempty(new_idx));
assert(est.tracks{2}.nisbad(old_idx) == 1);
assert(sum(est.tracks{2}.S(:, old_idx) == 1) == 1);
assert(sum(est.tracks{2}.S(:, new_idx) == 1) == 1);
assert(est.tracks{2}.birth_kind(old_idx) == 0);
assert(est.tracks{2}.birth_kind(new_idx) == 1);
assert(est.tracks{2}.parent_id(new_idx) == 1);
assert(isequal(est.assoc{2}.id, 2));
end

function test_confirmed_track_wins_over_conflict_tentative()
cfg = base_cfg();
cfg.M_confirm = 3;
cfg.N_confirm = 5;
cfg.max_coast_frames = 20;
xyz = {[0; 0; 0], [0; 0; 0], [0; 0; 0], ...
    [0, 800; 0, 0; 0, 0], [400; 0; 0]};
R = {cov3(1), cov3(1), cov3(1), cov3(2), cov3(1)};
times = (0:4)';

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(size(est.tracks{4}.L, 2) == 2);
conflict_idx = find(est.tracks{4}.birth_kind ~= 0, 1);
assert(~isempty(conflict_idx));
assert(isequal(est.assoc{5}.id, 1));
id1 = find(est.tracks{5}.L(2, :) == 1, 1);
assert(~isempty(id1) && est.tracks{5}.miss(id1) == 0);
end

function test_velocity_initialization_retries()
cfg = base_cfg();
cfg.vinit_baseline_s = 1;
cfg.vinit_min_disp_m = 200;
xyz = {[0; 0; 0], [150; 0; 0], [300; 0; 0]};
R = {cov3(1), cov3(1), cov3(1)};
times = [0; 1; 2];

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(est.tracks{2}.vinit(1) == 0);
assert(est.tracks{3}.vinit(1) == 1);
assert(norm(est.tracks{3}.m([2, 5, 8], 1) - [150; 0; 0]) < 1e-9);
end

function test_same_event_passive_updates_newborn()
cfg = base_cfg();
cfg.passive_bearing_update_on_active = true;
cfg.passive_bearing_update_active_hit_tracks = false;
cfg.passive_bearing_min_dt_s = 0.10;
cfg.passive_bearing_fast_gate_deg = 2;
xyz = {[0; 10000; 0], [10000; 0; 0]};
R = {cov3(1), cov3(1)};
ids = {1, 2};
times = [0; 0.05];

pb = cell(2, 1);
pb{1} = struct('t_sec', 0, 'ang_deg', [0; 0], ...
    'R_deg2', diag([0.05^2, 0.05^2]), 'src', 1, ...
    'tracklet_id', 1, 'n_meas', 1);
% 第二拍同时给出“新目标东向”和“旧目标北向”两个角度；节流期间只能放行
% 本拍新生，因此北向角度不得把旧航迹重新激活。
pb{2} = struct('t_sec', [0.05, 0.05], 'ang_deg', [90, 0; 0, 0], ...
    'R_deg2', repmat(diag([0.05^2, 0.05^2]), 1, 1, 2), 'src', [1, 1], ...
    'tracklet_id', [2, 1], 'n_meas', 2);
meta = repmat(struct('has_active', true, 'has_passive', true, ...
    'miss_cycle', true, 'confirm_cycle', true, 'n_active', 1, ...
    'n_passive', 1, 't_start', 0, 't_end', 0), 2, 1);
meta(2).t_start = 0.05;
meta(2).t_end = 0.05;
meta(2).n_passive = 2;
platform = struct('lat_deg', 30, 'lon_deg', 110, 'alt_m', 1000, ...
    'interp_lat', @(t) 30 + 0*t, 'interp_lon', @(t) 110 + 0*t, ...
    'interp_alt', @(t) 1000 + 0*t);

est = run_filter_adapt_ckf(xyz, R, times, cfg, pb, platform, ids, meta);
assert(est.passive_bearing_stats.n_updates == 2);
assert(est.passive_bearing_stats.n_measurements_used == 2);
assert(est.passive_bearing_stats.n_no_track == 0);
assert(sum(est.tracks{1}.S(:, 1) == 1) == 1);
assert(size(est.tracks{2}.L, 2) == 2);
old_idx = find(est.tracks{2}.L(2, :) == 1, 1);
assert(est.tracks{2}.miss(old_idx) == 1);
end

function test_same_frame_birth_candidates_are_not_suppressed()
cfg = base_cfg();
xyz = {[0, 5; 0, 0; 0, 0]};
R = {cov3(2)};
ids = {[1, 2]};

est = run_filter_adapt_ckf(xyz, R, 0, cfg, [], [], ids);
assert(size(est.tracks{1}.L, 2) == 2);
assert(numel(unique(est.assoc{1}.id)) == 2);
end

function test_tentative_track_times_out_before_late_hit()
cfg = base_cfg();
cfg.M_confirm = 3;
cfg.N_confirm = 5;
cfg.tentative_max_silence_s = 2;
xyz = {[0; 0; 0], [0; 0; 0]};
R = {cov3(1), cov3(1)};
times = [0; 2.01];

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(est.track_timeout_stats.n_tentative == 1);
assert(abs(est.track_timeout_stats.records(1).deadline_t - 2) < 1e-12);
assert(abs(est.track_timeout_stats.records(1).timeout_time - 2.01) < 1e-12);
assert(size(est.tracks{2}.L, 2) == 1);
assert(est.tracks{2}.L(2, 1) == 2);
assert(sum(est.tracks{2}.S(:, 1) == 1) == 1);
end

function test_confirmed_track_timeout_boundary()
cfg = base_cfg();
cfg.M_confirm = 1;
cfg.N_confirm = 1;
cfg.confirmed_max_silence_s = 3;

xyz = {[0; 0; 0], [0; 0; 0]};
R = {cov3(1), cov3(1)};
est_at_limit = run_filter_adapt_ckf(xyz, R, [0; 3], cfg);
assert(est_at_limit.track_timeout_stats.n_confirmed == 0);
assert(est_at_limit.tracks{2}.L(2, 1) == 1);

est_over_limit = run_filter_adapt_ckf(xyz, R, [0; 3.01], cfg);
assert(est_over_limit.track_timeout_stats.n_confirmed == 1);
assert(abs(est_over_limit.track_timeout_stats.records(1).deadline_t - 3) < 1e-12);
assert(size(est_over_limit.tracks{2}.L, 2) == 1);
assert(est_over_limit.tracks{2}.L(2, 1) == 2);
assert(est_over_limit.N(2) == 1);
metrics = evaluate_fusion_quant_metrics([], xyz, {1, 1}, ...
    est_over_limit, [0; 3.01], cfg);
assert(metrics.track_timeout.n_confirmed == 1);

cfg.track_timeout_enabled = false;
est_disabled = run_filter_adapt_ckf(xyz, R, [0; 20], cfg);
assert(est_disabled.track_timeout_stats.n_total == 0);
assert(est_disabled.tracks{2}.L(2, 1) == 1);
end

function test_seconds_timeout_overrides_short_coast_limit()
cfg = base_cfg();
cfg.M_confirm = 1;
cfg.N_confirm = 1;
cfg.max_coast_frames = 1;
cfg.confirmed_max_silence_s = 3;
xyz = {[0; 0; 0], zeros(3, 0), zeros(3, 0), zeros(3, 0), [0; 0; 0]};
R = {cov3(1), zeros(3, 3, 0), zeros(3, 3, 0), zeros(3, 3, 0), cov3(1)};
times = [0; 0.5; 1.0; 1.5; 2.5];

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(est.track_delete_stats.n_confirmed_coast == 0);
assert(est.track_timeout_stats.n_confirmed == 0);
assert(est.tracks{4}.miss(1) == 3);
assert(est.tracks{5}.L(2, 1) == 1);
assert(est.tracks{5}.miss(1) == 0);
end

function test_passive_update_extends_confirmed_wait()
cfg = base_cfg();
cfg.M_confirm = 1;
cfg.N_confirm = 1;
cfg.confirmed_max_silence_s = 1;
cfg.tentative_max_silence_s = 0.5;
cfg.passive_bearing_update_on_pure = true;
cfg.passive_bearing_min_dt_s = 0;
target = [0; 10000; 0];
xyz = {target, zeros(3, 0), zeros(3, 0), target};
R = {cov3(1), zeros(3, 3, 0), zeros(3, 3, 0), cov3(1)};
ids = {1, zeros(1, 0), zeros(1, 0), 1};
times = [0; 0.4; 0.8; 1.6];

pb = cell(4, 1);
pb{1} = empty_bearing(0);
pb{2} = empty_bearing(0.4);
pb{3} = struct('t_sec', 0.8, 'ang_deg', [0; 0], ...
    'R_deg2', diag([0.05^2, 0.05^2]), 'src', 1, ...
    'tracklet_id', 1, 'n_meas', 1);
pb{4} = empty_bearing(1.6);
meta = repmat(struct('has_active', true, 'has_passive', false, ...
    'miss_cycle', true, 'confirm_cycle', true, 'n_active', 1, ...
    'n_passive', 0, 't_start', 0, 't_end', 0), 4, 1);
meta(2).n_active = 0;
meta(2).t_start = 0.4;
meta(2).t_end = 0.4;
meta(3).has_active = false;
meta(3).has_passive = true;
meta(3).miss_cycle = false;
meta(3).confirm_cycle = false;
meta(3).n_active = 0;
meta(3).n_passive = 1;
meta(3).t_start = 0.8;
meta(3).t_end = 0.8;
meta(4).t_start = 1.6;
meta(4).t_end = 1.6;
platform = struct('lat_deg', 30, 'lon_deg', 110, 'alt_m', 1000, ...
    'interp_lat', @(t) 30 + 0*t, 'interp_lon', @(t) 110 + 0*t, ...
    'interp_alt', @(t) 1000 + 0*t);

est = run_filter_adapt_ckf(xyz, R, times, cfg, pb, platform, ids, meta);
assert(est.passive_bearing_stats.n_updates == 1);
assert(est.track_timeout_stats.n_confirmed == 0);
assert(est.tracks{3}.L(2, 1) == 1);
assert(est.tracks{3}.miss(1) == 1); % passive update must not erase active miss
assert(abs(est.tracks{3}.last_update_t(1) - 0.8) < 1e-12);
assert(abs(est.tracks{3}.last_passive_update_t(1) - 0.8) < 1e-12);
assert(est.tracks{4}.L(2, 1) == 1);
end

function cfg = base_cfg()
cfg = config_fusion();
cfg.use_imm = false;
cfg.vel_trend_enabled = false;
cfg.use_target_id_prior = false;
cfg.adapt_R_enabled = false;
cfg.meas_robust_enabled = false;
cfg.active_pre_gate_enabled = false;
cfg.birth_suppress_gated = false;
cfg.birth_guard_m = 10;
cfg.merge_pos_dist_m = 1;
cfg.assoc_pos_gate_m = 5000;
cfg.cost_unmatched = 150;
cfg.nis_gate = 16;
cfg.assoc_accept_nis = 16;
cfg.meas_fuse_enabled = true;
cfg.meas_fuse_weight = 'likelihood';
cfg.meas_fuse_R_scale = 1.5;
cfg.meas_fuse_cluster_gamma = 16;
cfg.meas_fuse_cluster_dist_m = 200;
cfg.vinit_baseline_s = 10;
% This suite exercises miss/NIS lifecycle rules, not production weight pruning.
cfg.missed_weight_decay = 1;
cfg.track_accuracy_purity_th = 1;
cfg.track_accuracy_min_assoc = 3;
cfg.do_plot = false;
end

function R = cov3(n)
R = repmat(diag([100^2, 100^2, 100^2]), 1, 1, n);
end

function pb = empty_bearing(t)
pb = struct('t_sec', t, 'ang_deg', zeros(2, 0), ...
    'R_deg2', zeros(2, 2, 0), 'src', zeros(1, 0), ...
    'tracklet_id', zeros(1, 0), 'n_meas', 0);
end
