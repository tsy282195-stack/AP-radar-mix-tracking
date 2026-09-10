function [est, events] = run_filter_joint_legacy_backbone( ...
        fused_xyz, fused_R, frame_times, passive_bearing, platform, ...
        fused_ids, event_meta, cfg, source_events)
%RUN_FILTER_JOINT_LEGACY_BACKBONE Extend the mature 3-D tracker with 2-D tracks.

K = numel(fused_xyz);
if nargin < 9, source_events = []; end
if isempty(passive_bearing), passive_bearing = cell(K, 1); end
passive_enabled = logical(get_cfg(cfg, 'passive_bearing_enabled', true));
[passive_bearing, ~] = select_enabled_angle_sources( ...
    passive_bearing, passive_enabled);

% TXT files are storage shards, not sensors. Optional de-duplication only
% removes exact overlapping rows before all passive shards share one source.
[passive_bearing, shard_stats] = dedup_angle_packets(passive_bearing, cfg);
passive_bearing = normalize_passive_sensor(passive_bearing);
[n_angle, n_passive_angle, n_active_angle] = angle_source_counts(passive_bearing);
n_active_range = sum(cellfun(@(z) size(z, 2), fused_xyz));
has_angle = n_angle > 0;
events = make_filter_events(fused_xyz, fused_R, frame_times, passive_bearing, ...
    fused_ids, platform, cfg, source_events);

fprintf(['  联合输入能力: 主动RAE=%d, 独立AE=%d ' ...
    '(被动=%d, 主动AE-only=%d)\n'], ...
    n_active_range, n_angle, n_passive_angle, n_active_angle);

cfg3 = cfg;
cfg3.joint_extension_enabled = has_angle;
cfg3.passive_bearing_enabled = has_angle;
cfg3.passive_bearing_confirm_hit = false;
cfg3.passive_bearing_update_on_active = true;
cfg3.passive_bearing_update_on_pure = true;
cfg3.passive_bearing_update_active_hit_tracks = true;
cfg3.passive_bearing_min_dt_s = 0;
cfg3.passive_bearing_fast_gate_deg = get_cfg(cfg, ...
    'joint_passive_fast_gate_deg', inf);
cfg3.use_target_id_prior = false;

est3 = run_filter_adapt_ckf(fused_xyz, fused_R, frame_times, cfg3, ...
    passive_bearing, platform, fused_ids, event_meta);

if has_angle
    if ~isfield(est3, 'joint2d') || isempty(est3.joint2d)
        error('run_filter_joint_legacy_backbone:MissingOnline2D', ...
            'The mature 3-D loop did not return its online angle branch.');
    end
    est2 = est3.joint2d;
    print_online_2d_summary(est2);
    processing_architecture = 'mature_3d_with_logical_dimension_manager';
else
    est2 = empty_online_2d_estimate(events, cfg);
    fprintf('  纯主动RAE输入: 二维分支与升降维管理未启用。\n');
    processing_architecture = 'mature_3d_only_joint_adapter';
end
residual_map = online_residual_map(est2);
offset = round(get_cfg(cfg, 'joint_2d_id_offset', 1000000));
if maximum_legacy_id(est3) >= offset
    error('run_filter_joint_legacy_backbone:IdNamespaceCollision', ...
        'cfg.joint_2d_id_offset must exceed every possible 3-D track ID.');
end

est = assemble_joint_estimate(est3, est2, events, residual_map, ...
    offset, platform, cfg);
est.online_causal = true;
est.processing_passes = 1;
est.processing_architecture = processing_architecture;
est.stats.angle_shard_inputs = shard_stats.n_input;
est.stats.angle_shard_duplicates = shard_stats.n_removed;
est.stats.input_active_rae = n_active_range;
est.stats.input_passive_ae = n_passive_angle;
est.stats.input_active_ae_only = n_active_angle;
end

function [n_total, n_passive, n_active] = angle_source_counts(pb)
n_passive = 0; n_active = 0;
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    n = size(field_or(pb{k}, 'ang_deg', zeros(2, 0)), 2);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    n_passive = n_passive + nnz(kind == 1);
    n_active = n_active + nnz(kind == 2);
end
n_total = n_passive + n_active;
end

function est = empty_online_2d_estimate(events, cfg)
K = numel(events);
empty_cells = cell(K, 1);
est = struct();
for name = {'X', 'P', 'L', 'X2', 'P2', 'L2', 'logical_tracks', ...
        'output', 'tracks', 'assoc', 'companions'}
    est.(name{1}) = empty_cells;
end
est.N = zeros(K, 1); est.N2 = zeros(K, 1); est.N_total = zeros(K, 1);
est.filter_times = reshape([events.t_sec], [], 1);
est.event_meta = events;
est.mode_counts = struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1));
est.timing = struct('total', 0);
est.transition_log = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
est.stats = struct();
est.confirmation = struct('M', get_cfg(cfg, 'joint_confirm_M', 3), ...
    'N_events', get_cfg(cfg, 'joint_confirm_N', 5), ...
    'passive_group_size', get_cfg(cfg, 'joint_passive_confirm_consecutive_hits', 3), ...
    'passive_max_gap_s', get_cfg(cfg, 'joint_passive_confirm_max_gap_s', 0.2));
est.framework = 'joint_2d3d';
est.output_contract = 'logical_track_v1';
end

function print_online_2d_summary(est2)
s = field_or(est2, 'stats', struct());
c = field_or(est2, 'confirmation', struct());
fprintf('\n========== 在线二维分支与维度管理 ==========\n');
fprintf('  确认条件: M=%g, 事件窗N=%g, 每组被动命中=%g, 连续间隔<=%.3fs\n', ...
    field_or(c, 'M', NaN), field_or(c, 'N_events', NaN), ...
    field_or(c, 'passive_group_size', NaN), ...
    field_or(c, 'passive_max_gap_s', NaN));
fprintf(['  量测关联=%g, 新生=%g, 新生抑制=%g, 暂态删除=%g, ' ...
    '重复合并=%g\n'], field_or(s, 'passive_assigned', 0), ...
    field_or(s, 'passive_births', 0), ...
    field_or(s, 'passive_birth_suppressed', 0), ...
    field_or(s, 'deleted_tentative', 0), ...
    field_or(s, 'duplicates_merged', 0));
fprintf(['  主动AE到二维候选/验收边=%g/%g, 严格拒绝[NIS=%g, 间断=%g, ' ...
    '方位=%g, 俯仰=%g, LOS=%g], 释放量测=%g\n'], ...
    field_or(s, 'active_2d_candidate_edges', 0), ...
    field_or(s, 'active_2d_accept_edges', 0), ...
    field_or(s, 'active_2d_reject_nis', 0), ...
    field_or(s, 'active_2d_reject_gap', 0), ...
    field_or(s, 'active_2d_reject_az', 0), ...
    field_or(s, 'active_2d_reject_el', 0), ...
    field_or(s, 'active_2d_reject_los', 0), ...
    field_or(s, 'active_2d_released_measurements', 0));
fprintf(['  二维候选/验收边=%g/%g, 严格拒绝[NIS=%g, 间断=%g, ' ...
    '方位=%g, 俯仰=%g, LOS=%g], 释放量测=%g\n'], ...
    field_or(s, 'passive_candidate_edges', 0), ...
    field_or(s, 'passive_accept_edges', 0), ...
    field_or(s, 'passive_reject_nis', 0), ...
    field_or(s, 'passive_reject_gap', 0), ...
    field_or(s, 'passive_reject_az', 0), ...
    field_or(s, 'passive_reject_el', 0), ...
    field_or(s, 'passive_reject_los', 0), ...
    field_or(s, 'passive_released_measurements', 0));
fprintf('  残余独立二维输出: %d点, %d条唯一航迹\n', ...
    sum(est2.N2), numel(output_ids(est2)));
log = field_or(est2, 'transition_log', struct('reason', {}));
reasons = {log.reason};
fprintf(['  跨维重复抑制=%g, 同ID接管=%g ' ...
    '(暂态=%g, 晚生=%g, 已绑定=%g), 质量降二维=%g, 恢复三维=%g\n'], ...
    field_or(s, 'cross_dimension_suppressed', 0), ...
    field_or(s, 'cross_dimension_adopted', 0), ...
    field_or(s, 'cross_dimension_tentative_suppressed', 0), ...
    field_or(s, 'cross_dimension_late_suppressed', 0), ...
    field_or(s, 'cross_dimension_bound_suppressed', 0), ...
    nnz(strcmp(reasons, '3d_quality_degraded')), ...
    nnz(strcmp(reasons, '2d_to_3d_2of3_confirmed')));
fprintf('  外部三维ID重绑定: 候选=%g, 提交=%g, 歧义拒绝=%g, detached被动更新=%g\n', ...
    field_or(s, 'external_rebind_candidates', 0), ...
    field_or(s, 'external_rebind_committed', 0), ...
    field_or(s, 'external_rebind_ambiguous', 0), ...
    field_or(s, 'detached_passive_assigned', 0));
fprintf(['    漏斗: 活动新ID=%g, 旧companion=%g, 投影无效=%g, ' ...
    '拒绝[角度=%g,角速度=%g,NIS=%g], 分配=%g, ' ...
    '等待[历史=%g,三维就绪=%g,旧ID退出=%g]\n'], ...
    field_or(s, 'external_rebind_active_unknown', 0), ...
    field_or(s, 'external_rebind_eligible', 0), ...
    field_or(s, 'external_rebind_projection_invalid', 0), ...
    field_or(s, 'external_rebind_reject_angle', 0), ...
    field_or(s, 'external_rebind_reject_rate', 0), ...
    field_or(s, 'external_rebind_reject_switch', 0), ...
    field_or(s, 'external_rebind_assigned', 0), ...
    field_or(s, 'external_rebind_wait_history', 0), ...
    field_or(s, 'external_rebind_wait_ready', 0), ...
    field_or(s, 'external_rebind_wait_bound', 0));
end

function [pb, has_active_angle] = select_enabled_angle_sources(pb, passive_enabled)
has_active_angle = false;
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    ang = field_or(pb{k}, 'ang_deg', zeros(2, 0)); n = size(ang, 2);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    has_active_angle = has_active_angle || any(kind == 2);
    keep = kind == 2 | (passive_enabled & kind == 1);
    if all(keep), continue; end
    R = normalize_covariance(field_or(pb{k}, 'R_deg2', []), 2, n, eye(2));
    pb{k}.ang_deg = ang(:, keep); pb{k}.R_deg2 = R(:, :, keep);
    pb{k}.src = sized_row(field_or(pb{k}, 'src', []), n, NaN); pb{k}.src = pb{k}.src(keep);
    pb{k}.shard = sized_row(field_or(pb{k}, 'shard', []), n, NaN); pb{k}.shard = pb{k}.shard(keep);
    pb{k}.kind = kind(keep);
    ids = sized_row(field_or(pb{k}, 'tracklet_id', []), n, NaN);
    pb{k}.tracklet_id = ids(keep);
    tt = sized_row(field_or(pb{k}, 't_sec', []), n, NaN); pb{k}.t_sec = tt(keep);
    pb{k}.n_meas = nnz(keep);
end
end

function pb = normalize_passive_sensor(pb)
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    n = field_or(pb{k}, 'n_meas', size(field_or(pb{k}, 'ang_deg', zeros(2, 0)), 2));
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    normalized = ones(1, n);
    normalized(kind == 2) = 2;
    pb{k}.kind = kind;
    pb{k}.src = normalized;
end
end

function [pb, stats] = dedup_angle_packets(pb, cfg)
stats = struct('n_input', 0, 'n_removed', 0);
for k = 1:numel(pb)
    if ~isempty(pb{k}) && isstruct(pb{k})
        stats.n_input = stats.n_input + size(field_or( ...
            pb{k}, 'ang_deg', zeros(2, 0)), 2);
    end
end
if ~get_cfg(cfg, 'joint_shard_dedup_enabled', false), return; end
time_gate = get_cfg(cfg, 'joint_shard_duplicate_time_s', 1e-9);
angle_gate = get_cfg(cfg, 'joint_shard_duplicate_angle_deg', 1e-9);
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    ang = field_or(pb{k}, 'ang_deg', zeros(2, 0)); n = size(ang, 2);
    if n < 2, continue; end
    src = sized_row(field_or(pb{k}, 'src', nan(1, n)), n, NaN);
    shard = sized_row(field_or(pb{k}, 'shard', src), n, NaN);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    tt = sized_row(field_or(pb{k}, 't_sec', []), n, NaN);
    keep = zeros(1, 0);
    for q = 1:n
        duplicate = false;
        for r = reshape(keep, 1, [])
            different_shard = isfinite(shard(q)) && isfinite(shard(r)) && ...
                shard(q) ~= shard(r);
            same_time = isfinite(tt(q)) && isfinite(tt(r)) && ...
                abs(tt(q) - tt(r)) <= time_gate;
            same_angle = abs(angle_diff(ang(1, q), ang(1, r))) <= angle_gate && ...
                abs(ang(2, q) - ang(2, r)) <= angle_gate;
            if different_shard && kind(q) == kind(r) && same_time && same_angle
                duplicate = true;
                break;
            end
        end
        if ~duplicate, keep(end + 1) = q; end %#ok<AGROW>
    end
    R = normalize_covariance(field_or(pb{k}, 'R_deg2', []), 2, n, eye(2));
    ids = sized_row(field_or(pb{k}, 'tracklet_id', []), n, NaN);
    stats.n_removed = stats.n_removed + n - numel(keep);
    pb{k}.ang_deg = ang(:, keep); pb{k}.R_deg2 = R(:, :, keep);
    pb{k}.src = src(keep); pb{k}.kind = kind(keep);
    pb{k}.shard = shard(keep);
    pb{k}.tracklet_id = ids(keep); pb{k}.t_sec = tt(keep);
    pb{k}.n_meas = numel(keep);
end
end

function events = make_filter_events(xyz, R, times, pb, ids, platform, cfg, source)
K = numel(xyz);
events = repmat(event_template(), K, 1);
for k = 1:K
    e = event_template();
    e.cycle_id = k; e.t_sec = times(k); e.t_start = times(k); e.t_end = times(k);
    z = xyz{k}; n = size(z, 2);
    e.has_active = n > 0; e.active.n_meas = n;
    e.active.t_sec = repmat(times(k), 1, n); e.active.xyz = z;
    e.active.R_xyz = normalize_covariance(R{k}, 3, n, diag([2500, 2500, 2500].^2));
    e.active.R_ae = repmat(diag([get_cfg(cfg, 'sigma_az_deg', 0.08)^2, ...
        get_cfg(cfg, 'sigma_el_deg', 0.06)^2]), 1, 1, n);
    e.active.has_range = true(1, n); e.active.src = ones(1, n);
    e.active.ids = sized_row(cell_value(ids, k, []), n, NaN);
    e.active.rae = nan(3, n);
    sensor = platform_enu(times(k), platform, cfg);
    for j = 1:n
        rel = z(:, j) - sensor;
        e.active.rae(:, j) = [norm(rel); atan2d(rel(1), rel(2)); ...
            atan2d(rel(3), hypot(rel(1), rel(2)))];
    end

    p = cell_value(pb, k, []);
    if ~isempty(p) && isstruct(p)
        ang = field_or(p, 'ang_deg', zeros(2, 0)); np = size(ang, 2);
        e.has_passive = np > 0; e.passive.n_meas = np;
        e.passive.ang = ang;
        e.passive.R_ae = normalize_covariance(field_or(p, 'R_deg2', []), 2, np, ...
            diag([get_cfg(cfg, 'sigma_passive_az_deg', 0.05)^2, ...
            get_cfg(cfg, 'sigma_passive_el_deg', 0.04)^2]));
        e.passive.ids = sized_row(field_or(p, 'tracklet_id', []), np, NaN);
        e.passive.src = sized_row(field_or(p, 'src', ones(1, np)), np, 1);
        e.passive.kind = sized_row(field_or(p, 'kind', ones(1, np)), np, 1);
        pt = field_or(p, 't_sec', times(k));
        e.passive.t_sec = sized_row(pt, np, times(k));
        if np > 0
            e.t_start = min([e.t_start, e.passive.t_sec]);
            e.t_end = max([e.t_end, e.passive.t_sec]);
        end
    end
    if k <= numel(source) && isfield(source(k), 't_start')
        e.t_start = min(e.t_start, source(k).t_start);
        e.t_end = max(e.t_end, source(k).t_end);
    end
    events(k) = e;
end
end

function map = online_residual_map(est2)
K = numel(est2.event_meta); map = cell(K, 1);
for k = 1:K
    e = est2.event_meta(k);
    if isfield(e, 'passive') && isfield(e.passive, 'original_index')
        map{k} = reshape(e.passive.original_index, 1, []);
    else
        map{k} = 1:e.passive.n_meas;
    end
end
end

function est = assemble_joint_estimate(e3, e2, events, residual_map, ...
        offset, platform, cfg)
K = numel(events); est = init_joint_estimate(K, events);
duplicate_outputs_suppressed = 0;
mature3d = e3;
if isfield(mature3d, 'joint2d'), mature3d = rmfield(mature3d, 'joint2d'); end
est.mature3d = mature3d; est.passive2d = e2;
for k = 1:K
    out = repmat(output_template(), 0, 1);
    sensor = platform_enu(events(k).t_sec, platform, cfg);
    companions = companions_at(e2, k);
    active_ids = legacy_assoc_ids(e3, k);
    passive_ids = passive_update_ids(e3, k);
    if k <= numel(e3.X) && ~isempty(e3.X{k})
        labels = e3.L{k};
        for j = 1:size(e3.X{k}, 2)
            external_id = labels(j, 2);
            if ~isempty(companion_pending_external(companions, external_id))
                continue;
            end
            companion = companion_for_external(companions, external_id);
            if ~isempty(companion) && companion.output_dim ~= 3, continue; end
            id = logical_id_for_external(external_id, companion, offset);
            birth_event = logical_birth_event(labels(j, 1), companion);
            mature_last_update = external_track_last_update(e3, k, external_id);
            o = make_3d_output(id, birth_event, events(k).t_sec, ...
                e3.X{k}(:, j), e3.P{k}(:, :, j), external_id, ...
                active_ids, passive_ids, companion, sensor, mature_last_update);
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end

    % A confirmed 2-D logical track may upgrade as soon as its attached
    % mature-3D candidate has 2/3 active evidence, before the candidate
    % reaches the mature tracker's stricter publication threshold.
    for j = 1:numel(companions)
        q = companions(j);
        id = logical_id_for_external(q.external_3d_id, q, offset);
        if q.output_dim == 3 && ~any([out.id] == id)
            [x, P, birth_event, found] = external_track_state(e3, k, q.external_3d_id);
            if found
                birth_event = logical_birth_event(birth_event, q);
                mature_last_update = external_track_last_update( ...
                    e3, k, q.external_3d_id);
                o = make_3d_output(id, birth_event, events(k).t_sec, x, P, ...
                    q.external_3d_id, active_ids, passive_ids, q, sensor, ...
                    mature_last_update);
                out(end + 1, 1) = o; %#ok<AGROW>
            end
        elseif q.output_dim == 2 && ~any([out.id] == id)
            o = make_companion_2d_output(q, id, events(k).t_sec);
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end

    if k <= numel(e2.output)
        for j = 1:numel(e2.output{k})
            q = e2.output{k}(j);
            if q.output_dim ~= 2, continue; end
            id = q.id + offset;
            if any([out.id] == id), continue; end
            o = output_template(); o.id = id; o.birth_event = q.birth_event;
            o.t_sec = q.t_sec; o.output_dim = 2; o.confirmed = q.confirmed;
            if isfield(q, 'last_update_t'), o.last_update_t = q.last_update_t; end
            o.mode = '2d_angle_only'; o.status_code = 1; o.truth_id = q.truth_id;
            o.az_deg = q.az_deg; o.el_deg = q.el_deg;
            o.angle_state = q.angle_state; o.angle_cov = q.angle_cov;
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end
    [out, n_duplicate] = arbitrate_logical_outputs(out);
    duplicate_outputs_suppressed = duplicate_outputs_suppressed + n_duplicate;
    est.output{k} = out;
    est = store_joint_event(est, e3, e2, events, residual_map, ...
        k, out, offset, companions);
end
est.transition_log = build_transition_log(est);
est.framework = 'joint_2d3d';
est.backbone = 'mature_active3d';
est.status_legend = struct('angle_only_2d', 1, 'passive_2d', 1, 'active_3d', 2, ...
    'active_passive_3d', 3, 'passive_maintained_3d', 4, 'coast_3d', 5);
est.status_legend.angle_2d_with_3d_shadow = 6;
est.timing.total = field_or(e3.timing, 'total', 0) + field_or(e2.timing, 'total', 0);
est.stats = struct('active_births', field_or(e3.birth_stats, 'n_total', 0), ...
    'passive_births', field_or(e2.stats, 'passive_births', 0), ...
    'passive_equivalent_hits', field_or(e3.passive_bearing_stats, ...
        'n_equivalent_confirm_hits', 0), ...
    'cross_dimension_merges', field_or(e2.stats, 'cross_dimension_adopted', 0), ...
    'cross_dimension_suppressed', field_or(e2.stats, 'cross_dimension_suppressed', 0), ...
    'passive_duplicate_merges', field_or(e2.stats, 'duplicates_merged', 0), ...
    'external_rebinds', field_or(e2.stats, 'external_rebind_committed', 0), ...
    'duplicate_outputs_suppressed', duplicate_outputs_suppressed);
est.output_freshness = struct( ...
    'three_d', field_or(e3, 'output_freshness', struct()), ...
    'two_d', struct('max_silence_s', get_cfg(cfg, ...
        'joint_2d_output_max_silence_s', 0.5), ...
        'n_suppressed', field_or(e2.stats, 'output_stale_suppressed', 0)));
end

function o = make_3d_output(id, birth_event, t, x, P, external_id, ...
        active_ids, passive_ids, companion, sensor, mature_last_update)
o = output_template();
o.id = id; o.birth_event = birth_event; o.t_sec = t;
o.output_dim = 3; o.confirmed = true; o.state3d = x; o.cov3d = P;
o.position_enu = x([1, 4, 7]); o.velocity_enu = x([2, 5, 8]);
o.acceleration_enu = x([3, 6, 9]);
o.last_update_t = mature_last_update;
rel = o.position_enu - sensor;
o.az_deg = atan2d(rel(1), rel(2));
o.el_deg = atan2d(rel(3), hypot(rel(1), rel(2))); o.range_m = norm(rel);
ha = ismember(external_id, active_ids); hp = ismember(external_id, passive_ids);
if ha && hp
    o.status_code = 3; o.mode = '3d_active_passive';
elseif ha
    o.status_code = 2; o.mode = '3d_active';
elseif hp
    o.status_code = 4; o.mode = '3d_passive_maintained';
else
    o.status_code = 5; o.mode = '3d_coast';
end
if ~isempty(companion)
    o.last_update_t = companion.last_update_t;
    o.quality = companion.quality;
    o.mode = companion.mode;
elseif ha || hp
    o.last_update_t = t;
end
end

function t = external_track_last_update(e3, k, external_id)
t = NaN;
if k > numel(e3.tracks) || isempty(e3.tracks{k}), return; end
s = e3.tracks{k};
if ~isfield(s, 'L') || isempty(s.L) || ~isfield(s, 'last_update_t'), return; end
i = find(s.L(2, :) == external_id, 1);
if ~isempty(i) && numel(s.last_update_t) >= i
    t = s.last_update_t(i);
end
end

function o = make_companion_2d_output(q, id, t)
o = output_template();
o.id = id; o.birth_event = q.birth_event; o.t_sec = t;
o.output_dim = 2; o.confirmed = q.confirmed; o.status_code = 6;
o.last_update_t = q.last_update_t;
o.mode = q.mode; o.angle_state = q.angle_state; o.angle_cov = q.angle_cov;
o.az_deg = q.angle_state(1); o.el_deg = q.angle_state(4); o.quality = q.quality;
end

function est = store_joint_event(est, e3, e2, events, residual_map, ...
        k, out, offset, companions)
idx2 = find([out.output_dim] == 2); idx3 = find([out.output_dim] == 3);
est.N2(k) = numel(idx2); est.N(k) = numel(idx3); est.N_total(k) = numel(out);
if isempty(idx2)
    est.X2{k} = zeros(6, 0); est.P2{k} = zeros(6, 6, 0); est.L2{k} = zeros(0, 2);
else
    est.X2{k} = cat(2, out(idx2).angle_state); est.P2{k} = cat(3, out(idx2).angle_cov);
    birth_events = [out(idx2).birth_event]; output_ids = [out(idx2).id];
    est.L2{k} = [birth_events(:), output_ids(:)];
end
if isempty(idx3)
    est.X{k} = []; est.P{k} = []; est.L{k} = [];
else
    est.X{k} = cat(2, out(idx3).state3d); est.P{k} = cat(3, out(idx3).cov3d);
    birth_events = [out(idx3).birth_event]; output_ids = [out(idx3).id];
    est.L{k} = [birth_events(:), output_ids(:)];
end
est.mode_counts.n2d(k) = est.N2(k); est.mode_counts.n3d(k) = est.N(k);
if isempty(out)
    est.mode_counts.nhold(k) = 0;
else
    est.mode_counts.nhold(k) = nnz(strcmp({out.mode}, 'hold'));
end
est.logical_tracks{k} = outputs_to_snapshots(out);
if k <= numel(e3.tracks), est.tracks{k} = e3.tracks{k}; end
est.companions{k} = companions;
est.assoc{k} = combine_assoc(e3, e2, events, residual_map, k, ...
    offset, companions);
end

function a = combine_assoc(e3, e2, events, residual_map, k, offset, companions)
a = assoc_template();
if k <= numel(e3.assoc) && ~isempty(e3.assoc{k})
    x = e3.assoc{k}; n = numel(x.id);
    for q = 1:n
        mi = indexed(x, 'meas_index', q, q);
        if mi < 1 || mi > events(k).active.n_meas, continue; end
        id = logical_id_for_external(x.id(q), ...
            companion_for_external_or_pending(companions, x.id(q)), offset);
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(x, q);
        a = append_assoc(a, id, 'active', mi, indexed(x, 'tid', q, NaN), ...
            events(k).active.rae(2:3, mi), events(k).active.xyz(:, mi), ...
            NaN, 3, innovation, nis, group_size, innovation_kind);
    end
end
if k <= numel(e3.passive_assoc) && ~isempty(e3.passive_assoc{k})
    d = e3.passive_assoc{k}; jj = find(d.used_mask & isfinite(d.track_id));
    for mi = jj
        id = logical_id_for_external(d.track_id(mi), ...
            companion_for_external_or_pending(companions, d.track_id(mi)), offset);
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(d, mi);
        a = append_assoc(a, id, 'passive', mi, ...
            events(k).passive.ids(mi), events(k).passive.ang(:, mi), ...
            nan(3, 1), NaN, 3, innovation, nis, group_size, innovation_kind);
    end
end
if k <= numel(e2.assoc) && ~isempty(e2.assoc{k})
    x = e2.assoc{k};
    for q = 1:numel(x.id)
        local = x.meas_index(q);
        if local < 1 || local > numel(residual_map{k}), continue; end
        mi = residual_map{k}(local);
        if q <= numel(x.type) && startsWith(x.type{q}, 'companion_passive_')
            from_2d = strcmp(x.type{q}, 'companion_passive_2d');
            companion = companion_for_logical(companions, x.id(q), from_2d);
            id = logical_id_for_external(NaN, companion, offset);
        else
            id = x.id(q) + offset;
        end
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(x, q);
        a = append_assoc(a, id, 'passive', mi, events(k).passive.ids(mi), ...
            events(k).passive.ang(:, mi), nan(3, 1), ...
            indexed(x, 'cost', q, NaN), 2, innovation, nis, ...
            group_size, innovation_kind);
    end
end
end

function companions = companions_at(est2, k)
companions = repmat(companion_template(), 0, 1);
if isfield(est2, 'companions') && k <= numel(est2.companions) && ...
        ~isempty(est2.companions{k})
    companions = est2.companions{k};
end
end

function q = companion_for_external(companions, external_id)
q = repmat(companion_template(), 0, 1);
if isempty(companions) || ~isfinite(external_id), return; end
i = find([companions.external_3d_id] == external_id, 1);
if ~isempty(i), q = companions(i); end
end

function q = companion_pending_external(companions, external_id)
q = repmat(companion_template(), 0, 1);
if isempty(companions) || ~isfinite(external_id), return; end
i = find([companions.pending_external_3d_id] == external_id, 1);
if ~isempty(i), q = companions(i); end
end

function q = companion_for_external_or_pending(companions, external_id)
q = companion_for_external(companions, external_id);
if isempty(q), q = companion_pending_external(companions, external_id); end
end

function q = companion_for_logical(companions, logical_id, from_2d)
q = repmat(companion_template(), 0, 1);
if isempty(companions) || ~isfinite(logical_id), return; end
i = find([companions.logical_id] == logical_id & ...
    [companions.from_2d] == logical(from_2d), 1);
if ~isempty(i), q = companions(i); end
end

function id = logical_id_for_external(external_id, companion, offset)
id = external_id;
if ~isempty(companion) && isfinite(companion.logical_id) && companion.logical_id > 0
    id = companion.logical_id;
    if companion.from_2d, id = id + offset; end
end
end

function birth_event = logical_birth_event(external_birth_event, companion)
birth_event = external_birth_event;
if ~isempty(companion) && ...
        isfinite(companion.birth_event) && companion.birth_event > 0
    birth_event = companion.birth_event;
end
end

function [out, n_suppressed] = arbitrate_logical_outputs(out)
n_suppressed = 0;
if numel(out) < 2, return; end
ids = [out.id]; keep = true(1, numel(out));
for id = unique(ids)
    idx = find(ids == id);
    if numel(idx) < 2, continue; end
    freshness = [out(idx).last_update_t];
    freshness(~isfinite(freshness)) = -inf;
    score = freshness + 1e-9 * [out(idx).output_dim];
    [~, best] = max(score);
    idx(best) = [];
    keep(idx) = false;
    n_suppressed = n_suppressed + numel(idx);
end
out = out(keep);
end

function [x, P, birth_event, found] = external_track_state(e3, k, external_id)
x = nan(9, 1); P = nan(9); birth_event = 0; found = false;
if k > numel(e3.tracks) || isempty(e3.tracks{k}), return; end
s = e3.tracks{k};
if ~isfield(s, 'L') || isempty(s.L), return; end
i = find(s.L(2, :) == external_id, 1);
if isempty(i) || size(s.m, 2) < i || size(s.P, 3) < i, return; end
x = s.m(:, i); P = s.P(:, :, i); birth_event = s.L(1, i);
found = all(isfinite(x)) && all(isfinite(P(:)));
end

function ids = legacy_assoc_ids(est, k)
ids = zeros(1, 0);
if k <= numel(est.assoc) && ~isempty(est.assoc{k}), ids = unique(est.assoc{k}.id); end
end

function ids = passive_update_ids(est, k)
ids = zeros(1, 0);
if k <= numel(est.passive_assoc) && ~isempty(est.passive_assoc{k})
    ids = unique(est.passive_assoc{k}.updated_track_ids);
end
end

function ids = output_ids(est)
ids = zeros(1, 0);
for k = 1:numel(est.output)
    if ~isempty(est.output{k}), ids = [ids, [est.output{k}.id]]; end %#ok<AGROW>
end
ids = unique(ids);
end

function ids = legacy_output_ids(est)
ids = zeros(1, 0);
for k = 1:numel(est.L)
    if ~isempty(est.L{k}), ids = [ids, est.L{k}(:, 2).']; end %#ok<AGROW>
end
ids = unique(ids);
end

function m = maximum_legacy_id(est)
ids = legacy_output_ids(est);
for k = 1:numel(est.tracks)
    if ~isempty(est.tracks{k}) && isfield(est.tracks{k}, 'L')
        ids = [ids, est.tracks{k}.L(2, :)]; %#ok<AGROW>
    end
end
if isempty(ids), m = 0; else, m = max(ids); end
end

function s = outputs_to_snapshots(out)
s = repmat(snapshot_template(), numel(out), 1);
for i = 1:numel(out)
    s(i).id = out(i).id; s(i).birth_event = out(i).birth_event;
    s(i).confirmed = out(i).confirmed; s(i).mode = out(i).mode;
    if isfield(out(i), 'last_update_t')
        s(i).last_update_t = out(i).last_update_t;
    end
    s(i).truth_id = out(i).truth_id;
    s(i).angle_state = out(i).angle_state; s(i).angle_cov = out(i).angle_cov;
    s(i).state3d = out(i).state3d; s(i).cov3d = out(i).cov3d;
    s(i).status_code = out(i).status_code; s(i).quality = out(i).quality;
end
end

function log = build_transition_log(est)
log = repmat(struct('id', 0, 't_sec', NaN, 'from', '', 'to', '', 'reason', ''), 0, 1);
last_id = zeros(1, 0); last_mode = cell(1, 0);
for k = 1:numel(est.output)
    for q = 1:numel(est.output{k})
        o = est.output{k}(q); j = find(last_id == o.id, 1);
        if isempty(j)
            last_id(end + 1) = o.id; last_mode{end + 1} = o.mode; %#ok<AGROW>
        elseif ~strcmp(last_mode{j}, o.mode)
            log(end + 1) = struct('id', o.id, 't_sec', o.t_sec, ...
                'from', last_mode{j}, 'to', o.mode, 'reason', 'status_change'); %#ok<AGROW>
            last_mode{j} = o.mode;
        end
    end
end
end

function est = init_joint_estimate(K, events)
est = struct('X', {cell(K, 1)}, 'P', {cell(K, 1)}, 'L', {cell(K, 1)}, ...
    'N', zeros(K, 1), 'X2', {cell(K, 1)}, 'P2', {cell(K, 1)}, ...
    'L2', {cell(K, 1)}, 'N2', zeros(K, 1), 'N_total', zeros(K, 1), ...
    'logical_tracks', {cell(K, 1)}, 'output', {cell(K, 1)}, ...
    'tracks', {cell(K, 1)}, 'companions', {cell(K, 1)}, ...
    'assoc', {cell(K, 1)}, ...
    'filter_times', reshape([events.t_sec], [], 1), 'event_meta', events, ...
    'mode_counts', struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1)), 'timing', struct('total', 0), ...
    'transition_log', []);
end

function o = output_template()
o = struct('id', 0, 'birth_event', 0, 't_sec', NaN, 'mode', '', ...
    'status_code', 0, 'output_dim', 0, 'confirmed', false, 'truth_id', NaN, ...
    'last_update_t', NaN, ...
    'az_deg', NaN, 'el_deg', NaN, 'range_m', NaN, ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9), ...
    'position_enu', nan(3, 1), 'velocity_enu', nan(3, 1), ...
    'acceleration_enu', nan(3, 1), 'quality', struct());
end

function s = snapshot_template()
s = struct('id', 0, 'birth_event', 0, 'confirmed', false, 'mode', '', ...
    'status_code', 0, 'last_update_t', NaN, 'truth_id', NaN, ...
    'quality', struct(), 'hit_history', zeros(1, 0), ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9));
end

function s = companion_template()
s = struct('external_3d_id', NaN, 'local_2d_id', NaN, ...
    'logical_id', NaN, 'previous_external_3d_id', NaN, ...
    'pending_external_3d_id', NaN, ...
    'from_2d', false, 'confirmed', false, 'mode', '', 'output_dim', 0, ...
    'external_fresh', false, 'birth_event', 0, 'birth_t', NaN, ...
    'last_update_t', NaN, 'angle_state', nan(6, 1), ...
    'angle_cov', nan(6), 'quality', struct());
end

function e = event_template()
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'confirm_cycle', false, ...
    'has_active', false, 'has_passive', false, ...
    'active', active_template(), 'passive', passive_template());
end

function a = active_template()
a = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'R_xyz', zeros(3, 3, 0), 'R_ae', zeros(2, 2, 0), ...
    'has_range', false(1, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
end

function p = passive_template()
p = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), ...
    'kind', zeros(1, 0), 'n_meas', 0);
end

function a = assoc_template()
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, ...
    'meas_index', zeros(1, 0), 'tid', zeros(1, 0), ...
    'ang', zeros(2, 0), 'xyz', zeros(3, 0), 'cost', zeros(1, 0), ...
    'filter_dim', zeros(1, 0), 'innovation', zeros(3, 0), ...
    'nis', zeros(1, 0), 'group_size', zeros(1, 0), ...
    'innovation_kind', zeros(1, 0));
end

function a = append_assoc(a, id, type, mi, tid, ang, xyz, cost, filter_dim, ...
        innovation, nis, group_size, innovation_kind)
if nargin < 10 || isempty(innovation), innovation = nan(3, 1); end
if nargin < 11 || isempty(nis), nis = NaN; end
if nargin < 12 || isempty(group_size), group_size = 1; end
if nargin < 13 || isempty(innovation_kind), innovation_kind = 0; end
a.id(end + 1) = id; a.type{end + 1} = type; a.meas_index(end + 1) = mi;
a.tid(end + 1) = tid; a.ang(:, end + 1) = ang; a.xyz(:, end + 1) = xyz;
a.cost(end + 1) = cost; a.filter_dim(end + 1) = filter_dim;
a.innovation(:, end + 1) = sized_innovation(innovation);
a.nis(end + 1) = nis;
a.group_size(end + 1) = group_size;
a.innovation_kind(end + 1) = innovation_kind;
end

function [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(s, q)
innovation = nan(3, 1); nis = NaN; group_size = 1; innovation_kind = 0;
if isstruct(s) && isfield(s, 'innovation') && size(s.innovation, 2) >= q
    innovation = sized_innovation(s.innovation(:, q));
end
nis = indexed(s, 'nis', q, nis);
group_size = indexed(s, 'group_size', q, group_size);
innovation_kind = indexed(s, 'innovation_kind', q, innovation_kind);
end

function v = sized_innovation(x)
v = nan(3, 1);
if isempty(x), return; end
n = min(3, numel(x)); v(1:n) = x(1:n);
end

function C = normalize_covariance(C, dim, n, fallback)
if isempty(C), C = repmat(fallback, 1, 1, n); return; end
if ismatrix(C), C = repmat(C, 1, 1, n); end
if size(C, 3) < n, C(:, :, end + 1:n) = repmat(fallback, 1, 1, n - size(C, 3)); end
C = C(1:dim, 1:dim, 1:n);
end

function v = sized_row(v, n, fallback)
if isempty(v), v = fallback * ones(1, n); else, v = v(:).'; end
if isscalar(v) && n > 1, v = repmat(v, 1, n); end
if numel(v) < n, v = [v, fallback * ones(1, n - numel(v))]; end
v = v(1:n);
end

function v = cell_value(c, k, fallback)
if iscell(c) && k <= numel(c) && ~isempty(c{k}), v = c{k}; else, v = fallback; end
end

function v = indexed(s, name, q, fallback)
if isstruct(s) && isfield(s, name) && numel(s.(name)) >= q
    v = s.(name)(q);
else
    v = fallback;
end
end

function v = field_or(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = fallback; end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function sensor = platform_enu(t, platform, cfg)
lat = platform.interp_lat(t); lon = platform.interp_lon(t); alt = platform.interp_alt(t);
origin = field_or(cfg, 'local_origin', 'first_platform');
if isnumeric(origin) && numel(origin) >= 3 && all(isfinite(origin(1:3)))
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);
else
    lat0 = platform.lat_deg(1); lon0 = platform.lon_deg(1); alt0 = platform.alt_m(1);
end
sensor = ecef_to_enu_rot(lat0, lon0) * ...
    (llh_to_ecef(lat, lon, alt) - llh_to_ecef(lat0, lon0, alt0));
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137; e2 = 6.69437999014e-3; lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
N = a / sqrt(1 - e2 * sin(lat)^2);
ecef = [(N + alt_m) * cos(lat) * cos(lon); ...
    (N + alt_m) * cos(lat) * sin(lon); ...
    (N * (1 - e2) + alt_m) * sin(lat)];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
R = [-sin(lon), cos(lon), 0; ...
    -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat); ...
    cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)];
end
