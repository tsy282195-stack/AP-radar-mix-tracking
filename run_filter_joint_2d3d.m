function est = run_filter_joint_2d3d(events, platform, cfg)
%RUN_FILTER_JOINT_2D3D Unified logical-track filter with 2-D/3-D branches.
%
% Each physical target owns one logical ID. The angle IMM-KF and spatial
% IMM filter are internal branches of that logical track. Association and
% lifecycle evidence are counted once at logical-track level.

K = numel(events);
p = joint_params(cfg);
est = init_estimate(K, events, p);
transitions = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, ...
    'reason', {}, 'kind', {});
stats = struct('active_assigned', 0, 'passive_assigned', 0, ...
    'active_births', 0, 'passive_births', 0, ...
    'active_birth_suppressed', 0, 'passive_birth_suppressed', 0, ...
    'active_measurements', 0, 'passive_measurements', 0, ...
    'active_unaccounted', 0, 'passive_unaccounted', 0, ...
    'active_update_rejected', 0, 'active_range_measurements', 0, ...
    'active_range_updates', 0, 'active_range_births', 0, ...
    'active_range_birth_suppressed', 0, 'active_range_unaccounted', 0, ...
    'deleted_tentative', 0, 'deleted_confirmed', 0, ...
    'deleted_capacity', 0, 'duplicates_merged', 0);
est.framework = 'joint_2d3d';
est.output_contract = 'logical_track_v1';
est.transition_log = transitions;
est.transition_summary = struct('n_total', 0, ...
    'n_output_dimension', 0, 'n_quality_state', 0);
est.stats = stats;
if K == 0
    return;
end

tracks = repmat(track_template(p), 0, 1);
next_id = 1;

fprintf('\n========== 统一二维/三维逻辑航迹滤波 ==========\n');
fprintf('  事件=%d, 逻辑确认=%d/%d, 二维门=%.2f, 三维门=%.2f\n', ...
    K, p.confirm_M, p.confirm_N, p.gate_2d, p.gate_3d);
fprintf(['  三维质量控制: sigma_r预警/降级=%.0f/%.0fm, ', ...
    '有效距离龄期预警/降级=%.1f/%.1fs, 连续申请=%d\n'], ...
    p.radial_sigma_warn_m, p.radial_sigma_drop_m, ...
    p.range_age_warn_s, p.range_age_drop_s, p.down_consecutive);

t_start = tic;
stage_timing = struct('predict_s', 0, 'active_s', 0, 'passive_s', 0, ...
    'lifecycle_s', 0, 'store_s', 0);
for k = 1:K
    e = events(k);
    t = primary_event_time(e);
    if ~isfinite(t)
        error('run_filter_joint_2d3d:InvalidEventTime', ...
            '事件%d没有有限处理时间。', k);
    end
    if k > 1 && t < est.filter_times(k - 1) - 1e-9
        error('run_filter_joint_2d3d:NonmonotonicEvents', ...
            '事件%d时间%.9f早于上一事件滤波时间%.9f。', ...
            k, t, est.filter_times(k - 1));
    end

    % Expire silence before a new measurement can refresh last_update_t.
    stage_start = tic;
    [tracks, stats] = remove_silent_tracks(tracks, t, p, stats);
    stage_timing.lifecycle_s = stage_timing.lifecycle_s + toc(stage_start);

    % Predict every logical track. Lifecycle windows are advanced after
    % association only for sensors that have observed that track before.
    stage_start = tic;
    for i = 1:numel(tracks)
        tracks(i) = predict_track(tracks(i), t, p);
        if e.has_active
            tracks(i).active_history = shift_window(tracks(i).active_history, 0);
        end
    end
    stage_timing.predict_s = stage_timing.predict_s + toc(stage_start);

    assoc = assoc_template();
    suppressed_a = false(1, e.active.n_meas);
    suppressed_p = false(1, e.passive.n_meas);
    event_hit_ids = zeros(1, 0);
    stats.active_measurements = stats.active_measurements + e.active.n_meas;
    stats.passive_measurements = stats.passive_measurements + e.passive.n_meas;
    stats.active_range_measurements = stats.active_range_measurements + ...
        nnz(e.active.has_range);

    % Active RAE has priority inside a synchronized event.
    stage_start = tic;
    active_sensor = platform_enu(t, platform, cfg);
    active_groups = measurement_source_groups(e.active);
    for source_group = 1:numel(active_groups)
        original_indices = active_groups{source_group};
        meas = subset_active_measurements(e.active, original_indices);
        [pairs_a, cost_a] = associate_active(tracks, meas, active_sensor, p);
        used_a = false(1, meas.n_meas);
        for q = 1:size(pairs_a, 1)
            ti = pairs_a(q, 1); mi = pairs_a(q, 2);
            original_mi = original_indices(mi);
            [candidate, accepted, range_updated, space_nis] = update_track_active( ...
                tracks(ti), meas, mi, t, active_sensor, p);
            if ~accepted
                stats.active_update_rejected = stats.active_update_rejected + 1;
                continue;
            end
            tracks(ti) = candidate;
            tracks(ti).seen_active = true;
            if range_updated
                tracks(ti).active_history(end) = 1;
                tracks(ti).last_active_t = t;
                stats.active_range_updates = stats.active_range_updates + 1;
            end
            tracks(ti).last_update_t = t;
            tracks(ti).miss = 0;
            tracks(ti) = vote_truth_id(tracks(ti), meas.ids(mi));
            used_a(mi) = true;
            event_hit_ids(end + 1) = tracks(ti).id; %#ok<AGROW>
            assoc = append_assoc(assoc, tracks(ti).id, 'active', original_mi, ...
                meas.ids(mi), e.active.rae(2:3, original_mi), ...
                e.active.xyz(:, original_mi), cost_a(ti, mi), ...
                measurement_dimension(meas, mi), true, ...
                range_updated, space_nis, measurement_source(meas, mi));
            stats.active_assigned = stats.active_assigned + 1;
        end

        % Births from this source are created only after its one-to-one
        % assignment. They are then available to subsequent independent
        % sources in the same event, preventing one target per file.
        for mi = find(~used_a)
            original_mi = original_indices(mi);
            if active_measurement_explained( ...
                    tracks, cost_a, mi, pairs_a(:, 1), meas.has_range(mi), p.birth_explain_nis)
                stats.active_birth_suppressed = stats.active_birth_suppressed + 1;
                suppressed_a(original_mi) = true;
                if meas.has_range(mi)
                    stats.active_range_birth_suppressed = ...
                        stats.active_range_birth_suppressed + 1;
                end
                continue;
            end
            tr = new_track(next_id, k, t, meas.rae(2:3, mi), ...
                meas.R_ae(:, :, mi), meas.xyz(:, mi), ...
                meas.R_xyz(:, :, mi), meas.has_range(mi), ...
                measurement_time(meas, mi, t), p);
            tr.seen_active = true;
            tr = vote_truth_id(tr, meas.ids(mi));
            tracks(end + 1, 1) = tr; %#ok<AGROW>
            event_hit_ids(end + 1) = next_id; %#ok<AGROW>
            assoc = append_assoc(assoc, next_id, 'active_birth', original_mi, ...
                meas.ids(mi), e.active.rae(2:3, original_mi), ...
                e.active.xyz(:, original_mi), NaN, ...
                measurement_dimension(meas, mi), true, ...
                meas.has_range(mi), NaN, measurement_source(meas, mi));
            if meas.has_range(mi)
                stats.active_range_updates = stats.active_range_updates + 1;
                stats.active_range_births = stats.active_range_births + 1;
            end
            next_id = next_id + 1;
            stats.active_births = stats.active_births + 1;
        end
    end
    stage_timing.active_s = stage_timing.active_s + toc(stage_start);

    % Preserve the passive scan timestamp inside a synchronized logical
    % opportunity. No lifecycle window is advanced during this substep.
    t_passive = passive_event_time(e, t);
    if e.has_passive && t_passive > t
        stage_start = tic;
        [tracks, stats] = remove_silent_tracks(tracks, t_passive, p, stats);
        stage_timing.lifecycle_s = stage_timing.lifecycle_s + toc(stage_start);
    end
    stage_start = tic;
    if e.has_passive && t_passive > t
        for i = 1:numel(tracks)
            tracks(i) = predict_track(tracks(i), t_passive, p);
        end
        t = t_passive;
        event_sensor = platform_enu(t, platform, cfg);
    else
        event_sensor = active_sensor;
    end

    % Re-associate passive AE after active updates/births. A passive return
    % can update both branches of the same logical track, but hit evidence
    % remains one for the event.
    passive_groups = measurement_source_groups(e.passive);
    for source_group = 1:numel(passive_groups)
        original_indices = passive_groups{source_group};
        meas = subset_passive_measurements(e.passive, original_indices);
        [pairs_p, cost_p] = associate_passive(tracks, meas, event_sensor, p);
        used_p = false(1, meas.n_meas);
        for q = 1:size(pairs_p, 1)
            ti = pairs_p(q, 1); mi = pairs_p(q, 2);
            original_mi = original_indices(mi);
            tracks(ti) = update_track_passive(tracks(ti), meas, mi, t, ...
                event_sensor, p);
            tracks(ti).seen_passive = true;
            tracks(ti).last_update_t = t;
            tracks(ti).last_passive_t = t;
            tracks(ti).miss = 0;
            tracks(ti) = vote_truth_id(tracks(ti), meas.ids(mi));
            used_p(mi) = true;
            event_hit_ids(end + 1) = tracks(ti).id; %#ok<AGROW>
            assoc = append_assoc(assoc, tracks(ti).id, 'passive', original_mi, ...
                meas.ids(mi), e.passive.ang(:, original_mi), ...
                nan(3, 1), cost_p(ti, mi), 2, true, false, NaN, ...
                measurement_source(meas, mi));
            stats.passive_assigned = stats.passive_assigned + 1;
        end

        for mi = find(~used_p)
            original_mi = original_indices(mi);
            if explained_available_measurement(cost_p, mi, pairs_p(:, 1), p.birth_explain_nis)
                stats.passive_birth_suppressed = stats.passive_birth_suppressed + 1;
                suppressed_p(original_mi) = true;
                continue;
            end
            tr = new_track(next_id, k, t, meas.ang(:, mi), ...
                meas.R_ae(:, :, mi), [], [], false, NaN, p);
            tr.seen_passive = true;
            tr = vote_truth_id(tr, meas.ids(mi));
            tracks(end + 1, 1) = tr; %#ok<AGROW>
            event_hit_ids(end + 1) = next_id; %#ok<AGROW>
            assoc = append_assoc(assoc, next_id, 'passive_birth', original_mi, ...
                meas.ids(mi), e.passive.ang(:, original_mi), ...
                nan(3, 1), NaN, 2, true, false, NaN, ...
                measurement_source(meas, mi));
            next_id = next_id + 1;
            stats.passive_births = stats.passive_births + 1;
        end
    end
    stage_timing.passive_s = stage_timing.passive_s + toc(stage_start);

    stage_start = tic;
    event_hit_ids = unique(event_hit_ids);
    event_track_ids = reshape([tracks.id], 1, []);
    event_hit = ismember(event_track_ids, event_hit_ids);
    for i = 1:numel(tracks)
        if tracks(i).birth_event == k
            continue;
        end
        hit = event_hit(i);
        opportunity = hit || lifecycle_miss_opportunity(tracks(i), e);
        if ~opportunity
            continue;
        end
        tracks(i).hit_history = shift_window(tracks(i).hit_history, double(hit));
        tracks(i).age = tracks(i).age + 1;
        if hit
            tracks(i).miss = 0;
        else
            tracks(i).miss = tracks(i).miss + 1;
        end
    end

    [tracks, n_merged, merge_id_map] = merge_tentative_duplicates( ...
        tracks, t, event_sensor, p);
    assoc = remap_assoc_ids(assoc, merge_id_map);
    stats.duplicates_merged = stats.duplicates_merged + n_merged;

    % Confirmation, quality assessment, and hysteretic mode transitions.
    for i = 1:numel(tracks)
        old_mode = tracks(i).mode;
        [tracks(i), reason] = manage_mode(tracks(i), t, event_sensor, p);
        if ~strcmp(old_mode, tracks(i).mode)
            transitions(end + 1) = struct('id', tracks(i).id, 't_sec', t, ...
                'from', old_mode, 'to', tracks(i).mode, 'reason', reason, ...
                'kind', transition_kind(old_mode, tracks(i).mode)); %#ok<AGROW>
        end
    end

    % Preserve the input branch before pruning; formal retention is separate.
    assoc = attach_assoc_filter_dimensions(assoc, tracks);
    assoc.input_dim = assoc.filter_dim;
    assoc.filter_dim(:) = 0;

    % Window exhaustion depends on this event's evidence; silence was
    % already handled before association at each processing timestamp.
    keep = true(1, numel(tracks));
    for i = 1:numel(tracks)
        if ~tracks(i).confirmed
            exhausted = tracks(i).age >= p.confirm_N && sum(tracks(i).hit_history) < p.confirm_M;
            if exhausted
                keep(i) = false;
                stats.deleted_tentative = stats.deleted_tentative + 1;
            end
        end
    end
    tracks = tracks(keep);

    if numel(tracks) > p.max_tracks
        score = arrayfun(@(x) double(x.confirmed) * 1e6 + sum(x.hit_history), tracks);
        [~, ord] = sort(score, 'descend');
        stats.deleted_capacity = stats.deleted_capacity + numel(tracks) - p.max_tracks;
        tracks = tracks(ord(1:p.max_tracks));
    end
    assoc = attach_assoc_filter_dimensions(assoc, tracks);
    est.measurement_disposition{k} = struct( ...
        'active', joint_measurement_disposition(e.active.n_meas, assoc, 'active', suppressed_a, k), ...
        'passive', joint_measurement_disposition(e.passive.n_meas, assoc, 'passive', suppressed_p, k));
    stage_timing.lifecycle_s = stage_timing.lifecycle_s + toc(stage_start);

    stage_start = tic;
    [est, outputs] = store_event(est, tracks, assoc, k, t, event_sensor);
    stage_timing.store_s = stage_timing.store_s + toc(stage_start);
    if mod(k, max(1, floor(K / 10))) == 0 || k == K
        dims = [outputs.output_dim];
        n_warn3d = nnz(arrayfun(@(x) x.confirmed && x.quality.warn3d, tracks));
        n_drop_request = nnz(arrayfun(@(x) x.confirmed && x.quality.degrade_request, tracks));
        fprintf(['  事件 %d/%d t=%.3f: 存活=%d, 输出2D=%d, 输出3D=%d, ', ...
            '3D预警=%d, 降级申请=%d\n'], k, K, t, numel(tracks), ...
            nnz(dims == 2), nnz(dims == 3), n_warn3d, n_drop_request);
    end
end

est.timing.total = toc(t_start);
est.timing.stages = stage_timing;
est.transition_log = transitions;
if isempty(transitions)
    est.transition_summary = struct('n_total', 0, ...
        'n_output_dimension', 0, 'n_quality_state', 0);
else
    transition_kinds = {transitions.kind};
    est.transition_summary = struct('n_total', numel(transitions), ...
        'n_output_dimension', nnz(strcmp(transition_kinds, 'output_dimension')), ...
        'n_quality_state', nnz(strcmp(transition_kinds, 'quality_state')));
end
stats.active_range_unaccounted = stats.active_range_measurements - ...
    stats.active_range_updates - stats.active_range_birth_suppressed;
stats.active_unaccounted = stats.active_measurements - stats.active_assigned - ...
    stats.active_births - stats.active_birth_suppressed;
stats.passive_unaccounted = stats.passive_measurements - stats.passive_assigned - ...
    stats.passive_births - stats.passive_birth_suppressed;
if stats.active_range_unaccounted ~= 0 || stats.active_unaccounted ~= 0 || ...
        stats.passive_unaccounted ~= 0
    error('run_filter_joint_2d3d:MeasurementAccountingMismatch', ...
        '量测去向不守恒: 主动=%d, 被动=%d, 主动距离=%d。', ...
        stats.active_unaccounted, stats.passive_unaccounted, ...
        stats.active_range_unaccounted);
end
est.stats = stats;
fprintf(['统一逻辑航迹滤波完成: %.2fs, 最终存活=%d, 输出维度切换=%d, ', ...
    '质量状态变化=%d\n'], est.timing.total, numel(tracks), ...
    est.transition_summary.n_output_dimension, est.transition_summary.n_quality_state);
fprintf(['  阶段耗时: 预测=%.2fs, 主动=%.2fs, 被动=%.2fs, ', ...
    '生命周期/模式=%.2fs, 存储=%.2fs\n'], ...
    stage_timing.predict_s, stage_timing.active_s, stage_timing.passive_s, ...
    stage_timing.lifecycle_s, stage_timing.store_s);
fprintf(['  主动距离去向: 输入=%d, 空间更新/新生=%d, 明确抑制=%d; ', ...
    '被动去向: 输入=%d, 关联=%d, 新生=%d, 抑制=%d\n'], ...
    stats.active_range_measurements, stats.active_range_updates, ...
    stats.active_range_birth_suppressed, stats.passive_measurements, ...
    stats.passive_assigned, stats.passive_births, stats.passive_birth_suppressed);
end

function [tracks, stats] = remove_silent_tracks(tracks, t, p, stats)
silence = max(t - [tracks.last_update_t], 0);
confirmed = [tracks.confirmed];
expired_tentative = ~confirmed & silence > p.tentative_timeout_s;
expired_confirmed = confirmed & silence > p.confirmed_timeout_s;
stats.deleted_tentative = stats.deleted_tentative + nnz(expired_tentative);
stats.deleted_confirmed = stats.deleted_confirmed + nnz(expired_confirmed);
% Births append by (end+1,1), including after the last track expires.
tracks = reshape(tracks(~(expired_tentative | expired_confirmed)), [], 1);
end

function p = joint_params(cfg)
p.confirm_M = get_cfg(cfg, 'joint_confirm_M', 3);
p.confirm_N = get_cfg(cfg, 'joint_confirm_N', 5);
p.gate_2d = get_cfg(cfg, 'joint_gate_2d', 9.2103);
p.gate_3d = get_cfg(cfg, 'joint_gate_3d', 11.3449);
p.unmatched_cost = get_cfg(cfg, 'joint_unmatched_cost', 50);
p.birth_explain_nis = get_cfg(cfg, 'joint_birth_explain_nis', 1.0);
p.max_tracks = get_cfg(cfg, 'max_tracks', 400);
p.tentative_timeout_s = get_cfg(cfg, 'joint_tentative_timeout_s', 3);
p.confirmed_timeout_s = get_cfg(cfg, 'joint_confirmed_timeout_s', 12);
p.angle_q_cv = get_cfg(cfg, 'joint_angle_q_cv', 0.08);
p.angle_q_ca = get_cfg(cfg, 'joint_angle_q_ca', 0.50);
p.angle_rate_std = get_cfg(cfg, 'joint_angle_rate_birth_std_dps', 2.0);
p.angle_acc_std = get_cfg(cfg, 'joint_angle_acc_birth_std_dps2', 3.0);
p.space_sigma_a_cv = get_cfg(cfg, 'joint_space_sigma_a_cv', 12);
p.space_sigma_j_ca = get_cfg(cfg, 'joint_space_sigma_j_ca', 15);
p.space_vel_std = get_cfg(cfg, 'joint_space_vel_birth_std_mps', 500);
p.space_acc_std = get_cfg(cfg, 'joint_space_acc_birth_std_mps2', 80);
p.imm_tpm = [get_cfg(cfg, 'joint_imm_cv_stay', 0.96), 1 - get_cfg(cfg, 'joint_imm_cv_stay', 0.96); ...
    1 - get_cfg(cfg, 'joint_imm_ca_stay', 0.94), get_cfg(cfg, 'joint_imm_ca_stay', 0.94)];
p.imm_mu0 = [get_cfg(cfg, 'joint_imm_cv_probability', 0.65); ...
    1 - get_cfg(cfg, 'joint_imm_cv_probability', 0.65)];
p.angle95_max_deg = get_cfg(cfg, 'joint_angle95_max_deg', 1.0);
p.birth3_M = get_cfg(cfg, 'joint_3d_birth_M', 3);
p.birth3_N = get_cfg(cfg, 'joint_3d_birth_N', 5);
p.upgrade3_M = get_cfg(cfg, 'joint_3d_upgrade_M', 2);
p.upgrade3_N = get_cfg(cfg, 'joint_3d_upgrade_N', 3);
p.up_consecutive = get_cfg(cfg, 'joint_up_consecutive', 1);
p.down_consecutive = get_cfg(cfg, 'joint_down_consecutive', 3);
p.switch_gate = get_cfg(cfg, 'joint_switch_gate_2d', 9.2103);
p.switch_cov_inflate = get_cfg(cfg, 'joint_switch_cov_inflate', 2.0);
p.radial_sigma_warn_m = get_cfg(cfg, 'joint_radial_sigma_warn_m', 5000);
p.radial_sigma_drop_m = get_cfg(cfg, 'joint_radial_sigma_drop_m', 10000);
p.range_age_warn_s = get_cfg(cfg, 'joint_range_age_warn_s', 3);
p.range_age_drop_s = get_cfg(cfg, 'joint_range_age_drop_s', 10);
p.space_nis_window = get_cfg(cfg, 'joint_space_nis_window', 5);
p.shadow_max_s = get_cfg(cfg, 'joint_shadow_max_s', 20);
p.mode_3d_prior_cost = get_cfg(cfg, 'joint_mode_3d_prior_cost', 0.5);
p.merge_angle_deg = get_cfg(cfg, 'joint_merge_angle_deg', 0.08);
p.merge_rate_dps = get_cfg(cfg, 'joint_merge_rate_dps', 1.0);
p.merge_nis = get_cfg(cfg, 'joint_merge_nis', 13.2767);
p.max_dt = get_cfg(cfg, 'joint_max_predict_dt_s', 5);
p.history_level = lower(char(get_cfg(cfg, 'joint_history_level', 'output')));
p.space_layout = joint_space_state_layout();
end

function tr = track_template(p)
tr = struct('id', 0, 'birth_event', 0, 'birth_t', NaN, 'confirmed', false, ...
    'mode', 'tentative', 'age', 0, 'miss', 0, 'last_t', NaN, ...
    'last_update_t', NaN, 'last_active_t', NaN, 'last_passive_t', NaN, ...
    'last_valid_range_t', NaN, ...
    'seen_active', false, 'seen_passive', false, ...
    'hit_history', zeros(1, p.confirm_N), ...
    'active_history', zeros(1, max(p.birth3_N, p.upgrade3_N)), ...
    'angle', invalid_branch(6), 'space', invalid_branch(9), ...
    'quality', quality_template(), 'up_count', 0, 'down_count', 0, ...
    'truth_id', NaN, 'truth_conf', 0);
end

function b = invalid_branch(n)
b = struct('valid', false, 'x', nan(n, 1), 'P', nan(n), ...
    'xm', nan(n, 2), 'Pm', nan(n, n, 2), 'mu', [0.5; 0.5], ...
    'last_t', NaN, 'last_nis', NaN, 'last_nis_norm', NaN, ...
    'nis_history', zeros(1, 0), 'nis_norm_history', zeros(1, 0));
end

function tr = new_track(id, birth_event, t, ae, R_ae, xyz, R_xyz, has_active, range_t, p)
tr = track_template(p);
tr.id = id; tr.birth_event = birth_event; tr.birth_t = t;
tr.last_t = t; tr.last_update_t = t; tr.age = 1;
tr.hit_history(end) = 1;
tr.angle = init_angle_branch(ae, R_ae, t, p);
if has_active
    tr.space = init_space_branch(xyz, R_xyz, t, p);
    tr.active_history(end) = 1;
    tr.last_active_t = t;
    tr.last_valid_range_t = refresh_valid_range_time(tr.last_valid_range_t, range_t);
end
end

function b = init_angle_branch(ae, R, t, p)
b = invalid_branch(6);
b.valid = true;
b.x = [wrap_az(ae(1)); 0; 0; ae(2); 0; 0];
    P0 = diag([1, p.angle_rate_std^2, p.angle_acc_std^2, ...
        1, p.angle_rate_std^2, p.angle_acc_std^2]);
    P0([1, 4], [1, 4]) = make_spd(R);
b.P = P0; b.xm = repmat(b.x, 1, 2); b.Pm = repmat(P0, 1, 1, 2);
b.mu = p.imm_mu0; b.last_t = t;
end

function b = init_space_branch(xyz, R, t, p)
b = invalid_branch(9);
b.valid = true;
b.x = zeros(9, 1); b.x([1, 4, 7]) = xyz;
    P0 = zeros(9);
    for d = 1:3
        ii = (d - 1) * 3 + (1:3);
        P0(ii, ii) = diag([1, p.space_vel_std^2, p.space_acc_std^2]);
    end
    P0(p.space_layout.position, p.space_layout.position) = make_spd(R);
b.P = make_spd(P0); b.xm = repmat(b.x, 1, 2); b.Pm = repmat(b.P, 1, 1, 2);
b.mu = p.imm_mu0; b.last_t = t;
end

function tr = predict_track(tr, t, p)
remaining = max(t - tr.last_t, 0);
while remaining > 0
    dt = min(remaining, p.max_dt);
    if tr.angle.valid
        tr.angle = predict_angle_branch(tr.angle, dt, p);
    end
    if tr.space.valid
        tr.space = predict_space_branch(tr.space, dt, p);
    end
    remaining = max(remaining - dt, 0);
end
if tr.angle.valid, tr.angle.last_t = t; end
if tr.space.valid, tr.space.last_t = t; end
tr.last_t = t;
end

function b = predict_angle_branch(b, dt, p)
[x0, P0, cbar] = imm_mix(b.xm, b.Pm, b.mu, p.imm_tpm, true);
for j = 1:2
    if j == 1
        [F, Q] = angle_model(dt, p.angle_q_cv, true);
    else
        [F, Q] = angle_model(dt, p.angle_q_ca, false);
    end
    b.xm(:, j) = F * x0(:, j);
    b.xm(1, j) = wrap_az(b.xm(1, j));
    b.Pm(:, :, j) = make_spd(F * P0(:, :, j) * F' + Q);
end
b.mu = cbar;
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, true);
end

function b = predict_space_branch(b, dt, p)
[x0, P0, cbar] = imm_mix(b.xm, b.Pm, b.mu, p.imm_tpm, false);
for j = 1:2
    if j == 1
        [F, Q] = space_model(dt, p.space_sigma_a_cv, true);
    else
        [F, Q] = space_model(dt, p.space_sigma_j_ca, false);
    end
    b.xm(:, j) = F * x0(:, j);
    b.Pm(:, :, j) = make_spd(F * P0(:, :, j) * F' + Q);
end
b.mu = cbar;
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
end

function [F, Q] = angle_model(dt, q, is_cv)
mode_index = 1 + double(~is_cv);
persistent cached_dt cached_q cached_F cached_Q
if isempty(cached_dt)
    cached_dt = [NaN, NaN]; cached_q = [NaN, NaN];
    cached_F = cell(1, 2); cached_Q = cell(1, 2);
elseif cached_dt(mode_index) == dt && cached_q(mode_index) == q
    F = cached_F{mode_index}; Q = cached_Q{mode_index};
    return;
end
if is_cv
    f = [1 dt 0; 0 1 0; 0 0 0];
    q1 = q * [dt^3/3 dt^2/2 0; dt^2/2 max(dt, 1e-6) 0; 0 0 1];
else
    f = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1];
    q1 = q * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 max(dt, 1e-6)];
end
F = blkdiag(f, f); Q = blkdiag(q1, q1);
cached_dt(mode_index) = dt; cached_q(mode_index) = q;
cached_F{mode_index} = F; cached_Q{mode_index} = Q;
end

function [F, Q] = space_model(dt, q, is_cv)
mode_index = 1 + double(~is_cv);
persistent cached_dt cached_q cached_F cached_Q
if isempty(cached_dt)
    cached_dt = [NaN, NaN]; cached_q = [NaN, NaN];
    cached_F = cell(1, 2); cached_Q = cell(1, 2);
elseif cached_dt(mode_index) == dt && cached_q(mode_index) == q
    F = cached_F{mode_index}; Q = cached_Q{mode_index};
    return;
end
if is_cv
    f = [1 dt 0; 0 1 0; 0 0 0];
    q1 = q^2 * [dt^3/3 dt^2/2 0; dt^2/2 max(dt, 1e-6) 0; 0 0 1];
else
    f = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1];
    q1 = q^2 * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 max(dt, 1e-6)];
end
F = blkdiag(f, f, f); Q = blkdiag(q1, q1, q1);
cached_dt(mode_index) = dt; cached_q(mode_index) = q;
cached_F{mode_index} = F; cached_Q{mode_index} = Q;
end

function [x0, P0, cbar] = imm_mix(xm, Pm, mu, TPM, circular)
n = size(xm, 1); M = size(xm, 2);
cbar = TPM' * mu; cbar = max(cbar, realmin); cbar = cbar / sum(cbar);
x0 = zeros(n, M); P0 = zeros(n, n, M);
for j = 1:M
    w = TPM(:, j) .* mu / max(cbar(j), realmin);
    w = w / sum(w);
    x0(:, j) = xm * w;
    if circular
        ref = xm(1, 1);
        x0(1, j) = wrap_az(ref + sum(w(:)' .* angle_diff(xm(1, :), ref)));
    end
    for i = 1:M
        dx = xm(:, i) - x0(:, j);
        if circular, dx(1) = angle_diff(xm(1, i), x0(1, j)); end
        P0(:, :, j) = P0(:, :, j) + w(i) * (Pm(:, :, i) + dx * dx');
    end
    P0(:, :, j) = make_spd(P0(:, :, j));
end
end

function [x, P] = imm_combine(xm, Pm, mu, circular)
x = xm * mu;
if circular
    ref = xm(1, 1);
    x(1) = wrap_az(ref + sum(mu(:)' .* angle_diff(xm(1, :), ref)));
end
P = zeros(size(Pm, 1));
for j = 1:numel(mu)
    dx = xm(:, j) - x;
    if circular, dx(1) = angle_diff(xm(1, j), x(1)); end
    P = P + mu(j) * (Pm(:, :, j) + dx * dx');
end
P = make_spd(P);
end

function [pairs, C] = associate_active(tracks, meas, sensor, p)
C = inf(numel(tracks), meas.n_meas);
has_range = logical(meas.has_range(:).');
range_cols = find(has_range); angle_cols = find(~has_range);
trace_R_xyz = covariance_traces(meas.R_xyz, 3, meas.n_meas);
trace_R_ae = covariance_traces(meas.R_ae, 2, meas.n_meas);
needs_space_bearing = ~isempty(angle_cols);
for i = 1:numel(tracks)
    z3 = nan(2, 1); S0 = nan(2); space_ok = false;
    if needs_space_bearing && tracks(i).space.valid
        [z3, S0, space_ok] = space_bearing_association_prediction( ...
            tracks(i).space, sensor);
    end
    candidate = false(1, meas.n_meas);
    if ~isempty(range_cols)
        if tracks(i).space.valid
            candidate(range_cols) = space_position_fast_candidates( ...
                tracks(i).space, meas.xyz(:, range_cols), ...
                trace_R_xyz(range_cols), p.gate_3d);
        elseif tracks(i).angle.valid
            candidate(range_cols) = angle_fast_candidates( ...
                tracks(i).angle, meas.rae(2:3, range_cols), ...
                trace_R_ae(range_cols), p.gate_2d);
        end
    end
    if ~isempty(angle_cols)
        if starts_with(tracks(i).mode, '3d')
            candidate(angle_cols) = space_bearing_fast_candidates( ...
                meas.rae(2:3, angle_cols), trace_R_ae(angle_cols), ...
                z3, S0, space_ok, p.gate_2d);
        else
            if tracks(i).angle.valid
                candidate(angle_cols) = angle_fast_candidates( ...
                    tracks(i).angle, meas.rae(2:3, angle_cols), ...
                    trace_R_ae(angle_cols), p.gate_2d);
            end
            if tracks(i).space.valid
                candidate(angle_cols) = candidate(angle_cols) | ...
                    space_bearing_fast_candidates( ...
                    meas.rae(2:3, angle_cols), trace_R_ae(angle_cols), ...
                    z3, S0, space_ok, p.gate_2d);
            end
        end
    end
    for j = find(candidate)
        C(i, j) = active_cost(tracks(i), meas, j, p, z3, S0, space_ok);
    end
end

% Range-bearing observations first belong to spatially compatible tracks.
% Only measurements left after that stage may initialize an angle-only
% track. This prevents a confirmed 2-D track from pre-empting a valid
% spatial candidate solely because of confirmation priority.
available_tracks = true(1, numel(tracks));
available_meas = true(1, meas.n_meas);
has_space = arrayfun(@(x) x.space.valid, tracks);
[space_pairs, available_tracks, available_meas] = assignment_stage( ...
    C, tracks, find(has_space), find(has_range), ...
    available_tracks, available_meas, p.unmatched_cost);
[angle_range_pairs, available_tracks, available_meas] = assignment_stage( ...
    C, tracks, find(~has_space), find(has_range), ...
    available_tracks, available_meas, p.unmatched_cost);
[angle_only_pairs, ~, ~] = assignment_stage(C, tracks, 1:numel(tracks), ...
    find(~has_range), available_tracks, available_meas, p.unmatched_cost);
pairs = [space_pairs; angle_range_pairs; angle_only_pairs];
end

function c = active_cost(tr, m, j, p, z3, S0, space_ok)
if m.has_range(j) && tr.space.valid
    c = space_position_gated_nis( ...
        tr.space, m.xyz(:, j), m.R_xyz(:, :, j), p.gate_3d);
    if isfinite(c) && starts_with(tr.mode, '3d')
        c = max(0, c - p.mode_3d_prior_cost);
    end
    return;
end

c2 = inf;
if tr.angle.valid
    c2 = angle_gated_nis(tr.angle, m.rae(2:3, j), m.R_ae(:, :, j), p.gate_2d);
end
if m.has_range(j)
    c = c2;
    return;
end

c3 = inf;
if tr.space.valid
    c3 = space_bearing_association_nis( ...
        m.rae(2:3, j), m.R_ae(:, :, j), z3, S0, space_ok, p.gate_2d);
end
if starts_with(tr.mode, '3d')
    if isfinite(c3), c = max(0, c3 - p.mode_3d_prior_cost); else, c = inf; end
elseif isfinite(c2)
    c = c2;
else
    c = c3;
end
end

function [pairs, available_tracks, available_meas] = assignment_stage( ...
        C, tracks, rows, cols, available_tracks, available_meas, unmatched)
pairs = zeros(0, 2);
rows = rows(available_tracks(rows));
cols = cols(available_meas(cols));
if isempty(rows) || isempty(cols)
    return;
end
local = cascade_assignment(C(rows, cols), tracks(rows), unmatched);
if isempty(local)
    return;
end
matched_rows = rows(local(:, 1));
matched_cols = cols(local(:, 2));
pairs = [matched_rows(:), matched_cols(:)];
available_tracks(pairs(:, 1)) = false;
available_meas(pairs(:, 2)) = false;
end

function [pairs, C] = associate_passive(tracks, meas, sensor, p)
C = inf(numel(tracks), meas.n_meas);
trace_R = covariance_traces(meas.R_ae, 2, meas.n_meas);
for i = 1:numel(tracks)
    z3 = nan(2, 1); S0 = nan(2); space_ok = false;
    if tracks(i).space.valid
        [z3, S0, space_ok] = space_bearing_association_prediction( ...
            tracks(i).space, sensor);
    end
    if starts_with(tracks(i).mode, '3d')
        candidate = space_bearing_fast_candidates( ...
            meas.ang, trace_R, z3, S0, space_ok, p.gate_2d);
    else
        candidate = false(1, meas.n_meas);
        if tracks(i).angle.valid
            candidate = angle_fast_candidates( ...
                tracks(i).angle, meas.ang, trace_R, p.gate_2d);
        end
        if tracks(i).space.valid
            candidate = candidate | space_bearing_fast_candidates( ...
                meas.ang, trace_R, z3, S0, space_ok, p.gate_2d);
        end
    end
    for j = find(candidate)
        C(i, j) = passive_cost(tracks(i), meas, j, p, z3, S0, space_ok);
    end
end
pairs = cascade_assignment(C, tracks, p.unmatched_cost);
end

function c = passive_cost(tr, m, j, p, z3, S0, space_ok)
if starts_with(tr.mode, '3d')
    c3 = inf;
    if tr.space.valid
        c3 = space_bearing_association_nis( ...
            m.ang(:, j), m.R_ae(:, :, j), z3, S0, space_ok, p.gate_2d);
    end
    if isfinite(c3), c = max(0, c3 - p.mode_3d_prior_cost); else, c = inf; end
    return;
end

c2 = inf;
if tr.angle.valid
    c2 = angle_gated_nis(tr.angle, m.ang(:, j), m.R_ae(:, :, j), p.gate_2d);
end
if isfinite(c2), c = c2; return; end
if tr.space.valid
    c = space_bearing_association_nis( ...
        m.ang(:, j), m.R_ae(:, :, j), z3, S0, space_ok, p.gate_2d);
else
    c = inf;
end
end

function pairs = cascade_assignment(C, tracks, unmatched)
pairs = zeros(0, 2);
if isempty(C), return; end
available = true(1, size(C, 2));
tiers = {find([tracks.confirmed]), find(~[tracks.confirmed])};
for q = 1:2
    rows = tiers{q}; cols = find(available);
    if isempty(rows) || isempty(cols), continue; end
    local = solve_assignment(C(rows, cols), unmatched);
    if isempty(local), continue; end
    matched_rows = rows(local(:, 1));
    matched_cols = cols(local(:, 2));
    add = [matched_rows(:), matched_cols(:)];
    pairs = [pairs; add]; %#ok<AGROW>
    available(add(:, 2)) = false;
end
end

function pairs = solve_assignment(C, unmatched)
pairs = solve_global_assignment(C, unmatched);
end

function nis = angle_gated_nis(b, z, R, gate)
nu = [angle_diff(z(1), b.x(1)); z(2) - b.x(4)];
S = b.P([1, 4], [1, 4]) + R;
nis = gated_nis(nu, S, gate);
end

function nis = space_position_nis(b, z, R)
H = zeros(3, 9); H(:, [1, 4, 7]) = eye(3);
nu = z - H * b.x; S = make_spd(H * b.P * H' + R);
nis = nu' * (S \ nu);
end

function nis = space_position_gated_nis(b, z, R, gate)
idx = [1, 4, 7];
nu = z - b.x(idx);
S = b.P(idx, idx) + R;
nis = gated_nis(nu, S, gate);
end

function traces = covariance_traces(R, dimension, n_meas)
traces = zeros(1, n_meas);
for q = 1:dimension
    traces = traces + reshape(R(q, q, 1:n_meas), 1, n_meas);
end
end

function candidate = angle_fast_candidates(b, Z, trace_R, gate)
nu = [angle_diff(Z(1, :), b.x(1)); Z(2, :) - b.x(4)];
trace_S = trace(b.P([1, 4], [1, 4])) + trace_R;
candidate = fast_gate_candidates(nu, trace_S, gate);
end

function candidate = space_position_fast_candidates(b, Z, trace_R, gate)
idx = [1, 4, 7];
nu = Z - b.x(idx);
trace_S = trace(b.P(idx, idx)) + trace_R;
candidate = fast_gate_candidates(nu, trace_S, gate);
end

function candidate = space_bearing_fast_candidates(Z, trace_R, z3, S0, ok, gate)
if ~ok
    candidate = false(1, size(Z, 2));
    return;
end
nu = [angle_diff(Z(1, :), z3(1)); Z(2, :) - z3(2)];
trace_S = trace(S0) + trace_R;
candidate = fast_gate_candidates(nu, trace_S, gate);
end

function candidate = fast_gate_candidates(nu, trace_S, gate)
finite_nu = all(isfinite(nu), 1);
energy = sum(nu.^2, 1);
reject = finite_nu & isfinite(trace_S) & trace_S > 0 & ...
    isfinite(gate) & energy > gate .* trace_S;
candidate = finite_nu & ~reject;
end

function [tr, accepted, range_updated, space_nis] = update_track_active( ...
        tr, m, j, t, sensor, p)
z2 = m.rae(2:3, j);
accepted = false;
range_updated = false;
space_nis = NaN;
if m.has_range(j) && tr.space.valid
    space_nis = space_position_nis(tr.space, m.xyz(:, j), m.R_xyz(:, :, j));
    if ~isfinite(space_nis) || space_nis > p.gate_3d
        return;
    end
end
if ~tr.angle.valid
    tr.angle = init_angle_branch(z2, m.R_ae(:, :, j), t, p);
else
    tr.angle = update_angle_branch(tr.angle, z2, m.R_ae(:, :, j));
end
if m.has_range(j)
    if ~tr.space.valid
        tr.space = init_space_branch(m.xyz(:, j), m.R_xyz(:, :, j), t, p);
        range_updated = true;
    else
        tr.space = update_space_active(tr.space, m.xyz(:, j), m.R_xyz(:, :, j));
        range_updated = true;
    end
elseif tr.space.valid
    nis = space_bearing_nis(tr.space, z2, m.R_ae(:, :, j), sensor);
    if isfinite(nis) && nis <= p.gate_2d
        tr.space = update_space_bearing(tr.space, z2, m.R_ae(:, :, j), sensor);
    end
end
if range_updated
    % Only a range-valid measurement that passed association/innovation
    % gating and actually initialized or updated the spatial branch may
    % refresh the reliable-range timestamp. Preserve the newest actual
    % measurement time when asynchronous/old measurements are encountered.
    tr.last_valid_range_t = refresh_valid_range_time( ...
        tr.last_valid_range_t, measurement_time(m, j, t));
end
accepted = true;
end

function tr = update_track_passive(tr, m, j, t, sensor, p)
z = m.ang(:, j); R = m.R_ae(:, :, j);
if ~tr.angle.valid
    tr.angle = init_angle_branch(z, R, t, p);
else
    tr.angle = update_angle_branch(tr.angle, z, R);
end
if tr.space.valid
    nis = space_bearing_nis(tr.space, z, R, sensor);
    if isfinite(nis) && nis <= p.gate_2d
        tr.space = update_space_bearing(tr.space, z, R, sensor);
    end
end
end

function nis = space_bearing_nis(b, z, R, sensor)
[zp, S, ~, ok] = space_bearing_stats_sensor(b.x, b.P, sensor, R);
if ~ok
    nis = inf;
else
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
    nis = nu' * (S \ nu);
end
end

function [z3, S0, ok] = space_bearing_association_prediction(b, sensor)
[z3, S0, ~, ok] = space_bearing_moments(b.x, b.P, sensor);
ok = ok && all(isfinite(z3)) && all(isfinite(S0(:)));
end

function nis = space_bearing_association_nis(z, R, z3, S0, ok, gate)
nis = inf;
if ~ok, return; end
nu = [angle_diff(z(1), z3(1)); z(2) - z3(2)];
S = S0 + R;
if fast_gate_reject(nu, S, gate), return; end
S = make_spd(S);
if any(~isfinite(S(:))) || rcond(S) <= 1e-12, return; end
nis = nu' * (S \ nu);
if nis > gate, nis = inf; end
end

function b = update_angle_branch(b, z, R)
H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    nu = [angle_diff(z(1), x(1)); z(2) - x(4)];
    S = make_spd(H * P * H' + R); K = (P * H') / S;
    x = x + K * nu; x(1) = wrap_az(x(1));
    I = eye(6); P = (I - K * H) * P * (I - K * H)' + K * R * K';
    b.xm(:, j) = x; b.Pm(:, :, j) = make_spd(P);
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, true);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 2;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function b = update_space_active(b, z, R)
H = zeros(3, 9); H(:, [1, 4, 7]) = eye(3);
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    nu = z - H * x; S = make_spd(H * P * H' + R); K = (P * H') / S;
    x = x + K * nu; I = eye(9);
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    b.xm(:, j) = x; b.Pm(:, :, j) = make_spd(P);
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 3;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function b = update_space_bearing(b, z, R, sensor)
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    [zp, S, Pxz, ok] = space_bearing_stats_sensor(x, P, sensor, R);
    if ~ok, like(j) = realmin; continue; end
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
    K = Pxz / S; x = x + K * nu; P = make_spd(P - K * S * K');
    b.xm(:, j) = x; b.Pm(:, :, j) = P;
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 2;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function [zp, S, Pxz, ok] = space_bearing_stats_sensor(x, P, sensor, R)
[zp, S0, Pxz, ok] = space_bearing_moments(x, P, sensor);
if ~ok, S = nan(2); return; end
S = make_spd(S0 + R); ok = all(isfinite(S(:))) && rcond(S) > 1e-12;
end

function [zp, S0, Pxz, ok] = space_bearing_moments(x, P, sensor)
n = numel(x); [Xi, ok] = cubature_points(x, P);
if ~ok, zp = nan(2, 1); S0 = nan(2); Pxz = nan(n, 2); return; end
Zi = relative_to_ae(Xi([1, 4, 7], :), sensor);
w = 1 / (2*n);
zp = [atan2d(sum(w * sind(Zi(1, :))), sum(w * cosd(Zi(1, :)))); ...
    sum(w * Zi(2, :))];
dz = [angle_diff(Zi(1, :), zp(1)); Zi(2, :) - zp(2)];
dx = Xi - x;
S0 = w * (dz * dz');
Pxz = w * (dx * dz');
end

function [Xi, ok] = cubature_points(x, P)
n = numel(x); P = make_spd(P); ok = true;
[S, flag] = chol(P, 'lower');
if flag ~= 0, Xi = zeros(n, 0); ok = false; return; end
D = sqrt(n) * S; Xi = [x + D, x - D];
end

function ae = relative_to_ae(xyz, sensor)
d = xyz - sensor(:); rxy = hypot(d(1, :), d(2, :));
ae = [wrap_az(atan2d(d(1, :), d(2, :))); ...
    atan2d(d(3, :), max(rxy, realmin))];
end

function [tracks, n_merged, id_map] = merge_tentative_duplicates(tracks, t, sensor, p)
n_merged = 0;
id_map = zeros(0, 2);
if numel(tracks) < 2, return; end
keep = true(1, numel(tracks));
angle_valid = reshape(arrayfun(@(x) x.angle.valid, tracks), 1, []);
updated = abs(reshape([tracks.last_update_t], 1, []) - t) <= 1e-9;
for i = 1:numel(tracks)
    if ~keep(i) || ~angle_valid(i), continue; end
    candidates = i + 1:numel(tracks);
    candidates = candidates(keep(candidates) & angle_valid(candidates));
    if updated(i)
        candidates = candidates(~updated(candidates));
    end
    for j = candidates
        if ~keep(j), continue; end
        % Automatic duplicate removal is deliberately restricted to two
        % tentative tracks. Spatial/angle proximity cannot prove that a
        % confirmed track and a nearby target are the same physical target.
        if tracks(i).confirmed || tracks(j).confirmed
            continue;
        end
        if updated(i) && updated(j)
            % Two unique measurements updated two tracks in this event.
            continue;
        end
        da = hypot(angle_diff(tracks(i).angle.x(1), tracks(j).angle.x(1)), ...
            tracks(i).angle.x(4) - tracks(j).angle.x(4));
        if da > p.merge_angle_deg, continue; end
        dr = hypot(tracks(i).angle.x(2) - tracks(j).angle.x(2), ...
            tracks(i).angle.x(5) - tracks(j).angle.x(5));
        if dr > p.merge_rate_dps, continue; end
        ii = [1, 2, 4, 5];
        dx = tracks(i).angle.x(ii) - tracks(j).angle.x(ii);
        dx(1) = angle_diff(tracks(i).angle.x(1), tracks(j).angle.x(1));
        S = make_spd(tracks(i).angle.P(ii, ii) + tracks(j).angle.P(ii, ii));
        if dx' * (S \ dx) > p.merge_nis, continue; end
        if ~merge_space_consistent(tracks(i), tracks(j), sensor, p)
            continue;
        end
        drop_i = false;
        if tracks(j).confirmed && ~tracks(i).confirmed
            survivor_id = tracks(j).id; removed_id = tracks(i).id;
            tracks(j) = absorb_duplicate(tracks(j), tracks(i));
            updated(j) = updated(i) || updated(j);
            keep(i) = false; drop_i = true;
        elseif ~tracks(i).confirmed && ~tracks(j).confirmed && ...
                sum(tracks(j).hit_history) > sum(tracks(i).hit_history)
            survivor_id = tracks(j).id; removed_id = tracks(i).id;
            tracks(j) = absorb_duplicate(tracks(j), tracks(i));
            updated(j) = updated(i) || updated(j);
            keep(i) = false; drop_i = true;
        else
            survivor_id = tracks(i).id; removed_id = tracks(j).id;
            tracks(i) = absorb_duplicate(tracks(i), tracks(j));
            updated(i) = updated(i) || updated(j);
            keep(j) = false;
        end
        id_map(end + 1, :) = [removed_id, survivor_id]; %#ok<AGROW>
        n_merged = n_merged + 1;
        if drop_i, break; end
    end
end
tracks = tracks(keep);
end

function tf = merge_space_consistent(a, b, sensor, p)
tf = true;
if a.space.valid && b.space.valid
    ii = [1, 4, 7];
    dx = a.space.x(ii) - b.space.x(ii);
    S = make_spd(a.space.P(ii, ii) + b.space.P(ii, ii));
    tf = all(isfinite(dx)) && dx' * (S \ dx) <= p.gate_3d;
    return;
end
if a.space.valid && starts_with(a.mode, '3d')
    tf = angle_matches_space(b.angle, a.space, sensor, p);
elseif b.space.valid && starts_with(b.mode, '3d')
    tf = angle_matches_space(a.angle, b.space, sensor, p);
end
end

function tf = angle_matches_space(angle, space, sensor, p)
tf = false;
if ~angle.valid, return; end
[z3, S3, ~, ok] = space_bearing_stats_sensor(space.x, space.P, sensor, zeros(2));
if ~ok, return; end
z2 = angle.x([1, 4]); P2 = angle.P([1, 4], [1, 4]);
nu = [angle_diff(z2(1), z3(1)); z2(2) - z3(2)];
S = make_spd(p.switch_cov_inflate * (P2 + S3));
tf = nu' * (S \ nu) <= p.switch_gate;
end

function keep = absorb_duplicate(keep, drop)
keep.hit_history = max(keep.hit_history, drop.hit_history);
keep.age = max(keep.age, drop.age); keep.miss = min(keep.miss, drop.miss);
keep.last_update_t = max_finite(keep.last_update_t, drop.last_update_t);
keep.last_passive_t = max_finite(keep.last_passive_t, drop.last_passive_t);
keep.seen_active = keep.seen_active || drop.seen_active;
keep.seen_passive = keep.seen_passive || drop.seen_passive;
if ~keep.angle.valid && drop.angle.valid
    keep.angle = drop.angle;
elseif keep.angle.valid && drop.angle.valid
    drop_is_newer = finite_is_later(drop.angle.last_t, keep.angle.last_t);
    same_time = same_finite_time(drop.angle.last_t, keep.angle.last_t);
    if drop_is_newer || (same_time && trace(drop.angle.P) < trace(keep.angle.P))
        keep.angle = drop.angle;
    end
end
space_from_drop = ~keep.space.valid && drop.space.valid;
if ~space_from_drop && keep.space.valid && drop.space.valid
    drop_is_newer = finite_is_later(drop.last_active_t, keep.last_active_t);
    same_time = same_finite_time(drop.last_active_t, keep.last_active_t);
    space_from_drop = drop_is_newer || ...
        (same_time && trace(drop.space.P) < trace(keep.space.P));
end
if space_from_drop
    keep.space = drop.space;
elseif ~keep.space.valid
    keep.active_history(:) = 0;
    keep.last_active_t = NaN;
    keep.last_valid_range_t = NaN;
end
if keep.space.valid
    keep.active_history = max(keep.active_history, drop.active_history);
    keep.last_active_t = max_finite(keep.last_active_t, drop.last_active_t);
    keep.last_valid_range_t = max_finite( ...
        keep.last_valid_range_t, drop.last_valid_range_t);
end
if (~isfinite(keep.truth_id) || drop.truth_conf > keep.truth_conf) && isfinite(drop.truth_id)
    keep.truth_id = drop.truth_id; keep.truth_conf = drop.truth_conf;
end
end

function tf = finite_is_later(a, b)
tf = isfinite(a) && (~isfinite(b) || a > b + 1e-9);
end

function tf = same_finite_time(a, b)
tf = isfinite(a) && isfinite(b) && abs(a - b) <= 1e-9;
end

function [tr, reason] = manage_mode(tr, t, sensor, p)
reason = '';
if ~starts_with(tr.mode, '3d') && tr.space.valid && isfinite(tr.last_active_t) && ...
        t - tr.last_active_t > p.shadow_max_s
    tr.space = invalid_branch(9);
    tr.active_history(:) = 0;
end
tr.quality = assess_quality(tr, t, sensor, p);
if ~tr.confirmed && sum(tr.hit_history) >= p.confirm_M
    tr.confirmed = true;
    if space_birth_ready(tr, p) && tr.quality.recover3d
        tr.mode = '3d'; reason = 'logical_confirm_3d';
    elseif tr.quality.valid2d
        tr.mode = mode_2d_name(tr, t, p); reason = 'logical_confirm_2d';
    else
        tr.mode = 'hold'; reason = 'logical_confirm_hold';
    end
    return;
end
if ~tr.confirmed, return; end

if strcmp(tr.mode, 'hold')
    can_recover3d = tr.quality.valid3d && tr.quality.recover3d && ...
        space_upgrade_ready(tr, p) && tr.quality.switch_consistent;
    if can_recover3d
        tr.mode = '3d'; reason = 'space_quality_recovered';
    elseif tr.quality.degrade_request
        % A 3-D degradation request and the ability of the 2-D companion
        % branch to take over are deliberately separate decisions.
        if tr.quality.can_takeover2d
            tr.mode = '2d_shadow'; reason = '3d_degradation_2d_takeover_enabled';
        end
    elseif tr.quality.valid2d
        tr.mode = mode_2d_name(tr, t, p); reason = 'angle_quality_recovered';
    end
    return;
end

if starts_with(tr.mode, '2d')
    if ~tr.quality.valid2d
        tr.mode = 'hold'; tr.up_count = 0; tr.down_count = 0;
        reason = 'angle_quality_degraded';
        return;
    end
    ready = space_upgrade_ready(tr, p) && tr.quality.recover3d && ...
        tr.quality.switch_consistent;
    if ready, tr.up_count = tr.up_count + 1; else, tr.up_count = 0; end
    if tr.up_count >= p.up_consecutive
        tr.mode = '3d'; tr.up_count = 0; tr.down_count = 0;
        reason = '2d_to_3d_quality_confirmed';
    elseif tr.space.valid
        if isfinite(tr.last_active_t) && t - tr.last_active_t <= p.shadow_max_s
            tr.mode = '2d_to_3d';
        else
            tr.mode = '2d_shadow';
        end
    else
        tr.mode = '2d';
    end
    return;
end

if starts_with(tr.mode, '3d')
    if tr.quality.down3d
        tr.down_count = tr.down_count + 1;
    else
        tr.down_count = 0;
    end
    if tr.down_count >= p.down_consecutive
        if tr.quality.can_takeover2d
            tr.mode = '2d_shadow'; reason = '3d_quality_degraded';
        else
            tr.mode = 'hold'; reason = '3d_drop_blocked_no_2d_takeover';
        end
        tr.active_history(:) = 0;
        tr.down_count = 0; tr.up_count = 0;
    elseif tr.quality.warn3d
        tr.mode = '3d_warn';
    else
        tr.mode = '3d';
    end
end
end

function q = assess_quality(tr, t, sensor, p)
q = quality_template();
q.last_valid_range_t = tr.last_valid_range_t;
q.range_age_s = effective_range_age(t, tr.last_valid_range_t);
if tr.angle.valid
    s = sqrt(max([tr.angle.P(1, 1), tr.angle.P(4, 4)], 0));
    q.angle95_deg = 1.96 * max(s);
    q.valid2d = isfinite(q.angle95_deg) && q.angle95_deg <= p.angle95_max_deg;
end
if tr.space.valid
    [q.radial_sigma_m, q.range_m, Pp] = radial_range_uncertainty( ...
        tr.space.x, tr.space.P, sensor, p.space_layout);
    q.radial95_m = 1.96 * q.radial_sigma_m;
    q.position95_m = sqrt(7.8147279 * max(real(eig(Pp))));
    q.relative_range_sigma = q.radial_sigma_m / max(q.range_m, 1);
    nh = tr.space.nis_norm_history;
    nh = nh(max(1, end - p.space_nis_window + 1):end);
    nh = nh(isfinite(nh));
    if ~isempty(nh), q.nis_norm = median(nh); end

    % 3-D mode control intentionally uses only two physical quantities:
    % radial 1-sigma range uncertainty and age of the last reliable range
    % update. Position95, relative uncertainty and NIS remain diagnostic.
    q.radial_warn = q.radial_sigma_m >= p.radial_sigma_warn_m;
    q.range_age_warn = q.range_age_s >= p.range_age_warn_s;
    q.warn3d = q.radial_warn || q.range_age_warn;
    q.degrade_request = q.radial_sigma_m >= p.radial_sigma_drop_m && ...
        q.range_age_s >= p.range_age_drop_s;
    q.down3d = q.degrade_request; % compatibility alias for existing consumers
    q.valid3d = ~q.degrade_request;
    q.recover3d = q.radial_sigma_m < p.radial_sigma_warn_m && ...
        q.range_age_s < p.range_age_warn_s;
end
needs_switch = tr.confirmed && (strcmp(tr.mode, 'hold') || ...
    starts_with(tr.mode, '2d') || ...
    (starts_with(tr.mode, '3d') && q.degrade_request));
if needs_switch && tr.angle.valid && tr.space.valid
    z2 = tr.angle.x([1, 4]); P2 = tr.angle.P([1, 4], [1, 4]);
    [z3, S3, ~, ok] = space_bearing_stats_sensor( ...
        tr.space.x, tr.space.P, sensor, zeros(2));
    if ok
        nu = [angle_diff(z2(1), z3(1)); z2(2) - z3(2)];
        S = make_spd(p.switch_cov_inflate * (P2 + S3));
        q.switch_nis = nu' * (S \ nu);
        q.switch_consistent = q.switch_nis <= p.switch_gate;
    end
end
q.can_takeover2d = q.valid2d && q.switch_consistent;
end

function tf = space_birth_ready(tr, p)
w = tr.active_history(max(1, end - p.birth3_N + 1):end);
tf = sum(w) >= p.birth3_M;
end

function tf = space_upgrade_ready(tr, p)
w = tr.active_history(max(1, end - p.upgrade3_N + 1):end);
tf = sum(w) >= p.upgrade3_M;
end

function name = mode_2d_name(tr, t, p)
if tr.space.valid && isfinite(tr.last_active_t)
    if t - tr.last_active_t <= p.shadow_max_s, name = '2d_to_3d'; else, name = '2d_shadow'; end
else
    name = '2d';
end
end

function kind = transition_kind(from, to)
if mode_output_dimension(from) ~= mode_output_dimension(to)
    kind = 'output_dimension';
else
    kind = 'quality_state';
end
end

function dimension = mode_output_dimension(mode)
dimension = 0;
if starts_with(mode, '2d')
    dimension = 2;
elseif starts_with(mode, '3d')
    dimension = 3;
end
end

function q = quality_template()
q = struct('valid2d', false, 'valid3d', false, 'warn3d', false, ...
    'down3d', false, 'degrade_request', false, 'recover3d', false, ...
    'switch_consistent', false, 'can_takeover2d', false, ...
    'radial_warn', false, 'range_age_warn', false, ...
    'angle95_deg', inf, 'position95_m', inf, 'radial_sigma_m', inf, ...
    'radial95_m', inf, 'relative_range_sigma', inf, 'range_m', NaN, ...
    'range_age_s', inf, 'last_valid_range_t', NaN, ...
    'switch_nis', inf, 'nis_norm', NaN);
end

function [est, outputs] = store_event(est, tracks, assoc, k, t, sensor)
keep_diagnostics = any(strcmp(est.history_level, {'diagnostic', 'full'}));
keep_full = strcmp(est.history_level, 'full');
if keep_diagnostics
    snap = repmat(snapshot_template(), numel(tracks), 1);
    quality_log = repmat(quality_log_template(), numel(tracks), 1);
else
    snap = repmat(snapshot_template(), 0, 1);
    quality_log = repmat(quality_log_template(), 0, 1);
end
outputs = repmat(output_template(), 0, 1);
for i = 1:numel(tracks)
    if keep_diagnostics
        snap(i) = make_snapshot(tracks(i), t);
        quality_log(i) = make_quality_log(tracks(i), t);
    end
    if tracks(i).confirmed
        candidate = make_output(tracks(i), t, sensor);
        if candidate.output_dim > 0
            outputs(end + 1, 1) = candidate; %#ok<AGROW>
        end
    end
end
if keep_diagnostics
    est.logical_tracks{k} = snap;
    est.quality_history{k} = quality_log;
end
est.output{k} = outputs;
if strcmp(est.history_level, 'output')
    assoc = compact_association_history(assoc);
end
est.assoc{k} = assoc;
est.filter_times(k) = t;
est.N_total(k) = numel(outputs);

idx2 = find([outputs.output_dim] == 2);
idx3 = find([outputs.output_dim] == 3);
est.N2(k) = numel(idx2); est.N(k) = numel(idx3);
if ~isempty(idx2)
    est.X2{k} = cat(2, outputs(idx2).angle_state);
    est.P2{k} = cat(3, outputs(idx2).angle_cov);
    birth_events = [outputs(idx2).birth_event];
    output_ids = [outputs(idx2).id];
    est.L2{k} = [birth_events(:), output_ids(:)];
else
    est.X2{k} = zeros(6, 0); est.P2{k} = zeros(6, 6, 0); est.L2{k} = zeros(0, 2);
end
if ~isempty(idx3)
    est.X{k} = cat(2, outputs(idx3).state3d);
    est.P{k} = cat(3, outputs(idx3).cov3d);
    birth_events = [outputs(idx3).birth_event];
    output_ids = [outputs(idx3).id];
    est.L{k} = [birth_events(:), output_ids(:)];
else
    est.X{k} = zeros(9, 0); est.P{k} = zeros(9, 9, 0); est.L{k} = zeros(0, 2);
end

est.mode_counts.n2d(k) = numel(idx2);
est.mode_counts.n3d(k) = numel(idx3);
est.mode_counts.nhold(k) = nnz([tracks.confirmed] & strcmp({tracks.mode}, 'hold'));
est.mode_counts.n3d_warn(k) = nnz([tracks.confirmed] & strcmp({tracks.mode}, '3d_warn'));
est.mode_counts.ndrop_request(k) = nnz(arrayfun( ...
    @(x) x.confirmed && x.quality.degrade_request, tracks));

% The all-track legacy snapshot is expensive and only retained in full mode.
if keep_full
    valid3 = find(arrayfun(@(x) x.space.valid, tracks));
    if isempty(valid3)
        est.tracks{k} = struct('m', zeros(9, 0), 'P', zeros(9, 9, 0), ...
            'L', zeros(2, 0), 'conf', zeros(1, 0), 'mode', {cell(1, 0)});
    else
        m3 = zeros(9, numel(valid3)); P3 = zeros(9, 9, numel(valid3));
        for q = 1:numel(valid3)
            m3(:, q) = tracks(valid3(q)).space.x;
            P3(:, :, q) = tracks(valid3(q)).space.P;
        end
        est.tracks{k} = struct('m', m3, 'P', P3, ...
            'L', [[tracks(valid3).birth_event]; [tracks(valid3).id]], ...
            'conf', [tracks(valid3).confirmed], 'mode', {{tracks(valid3).mode}});
    end
end
end

function a = compact_association_history(a)
% Measurement values remain in events; keep only routing/provenance fields.
a.tid = zeros(1, 0);
a.ang = zeros(2, 0);
a.xyz = zeros(3, 0);
a.cost = zeros(1, 0);
a.update_accepted = false(1, 0);
a.space_nis = zeros(1, 0);
end

function s = make_snapshot(tr, t)
s = snapshot_template();
s.id = tr.id; s.birth_event = tr.birth_event; s.confirmed = tr.confirmed;
s.t_sec = t; s.mode = tr.mode; s.last_update_t = tr.last_update_t;
s.last_valid_range_t = tr.last_valid_range_t; s.truth_id = tr.truth_id;
s.quality = tr.quality; s.hit_history = tr.hit_history;
if tr.angle.valid, s.angle_state = tr.angle.x; s.angle_cov = tr.angle.P; end
if tr.space.valid, s.state3d = tr.space.x; s.cov3d = tr.space.P; end
end

function s = snapshot_template()
s = struct('id', 0, 'birth_event', 0, 'confirmed', false, 't_sec', NaN, 'mode', '', ...
    'last_update_t', NaN, 'last_valid_range_t', NaN, ...
    'truth_id', NaN, 'quality', quality_template(), ...
    'hit_history', zeros(1, 0), 'angle_state', nan(6, 1), ...
    'angle_cov', nan(6), 'state3d', nan(9, 1), 'cov3d', nan(9));
end

function h = make_quality_log(tr, t)
h = quality_log_template();
h.id = tr.id; h.t_sec = t; h.radial_sigma_m = tr.quality.radial_sigma_m;
h.range_age_s = tr.quality.range_age_s;
h.last_valid_range_t = tr.last_valid_range_t;
h.mode = tr.mode; h.degrade_request = tr.quality.degrade_request;
h.warn3d = tr.quality.warn3d; h.can_takeover2d = tr.quality.can_takeover2d;
end

function h = quality_log_template()
h = struct('id', 0, 't_sec', NaN, 'radial_sigma_m', inf, ...
    'range_age_s', inf, 'last_valid_range_t', NaN, 'mode', '', ...
    'degrade_request', false, 'warn3d', false, 'can_takeover2d', false);
end

function o = make_output(tr, t, sensor)
o = output_template();
o.id = tr.id; o.birth_event = tr.birth_event; o.t_sec = t; o.mode = tr.mode;
o.confirmed = tr.confirmed; o.truth_id = tr.truth_id; o.quality = tr.quality;
if tr.angle.valid
    o.angle_state = tr.angle.x; o.angle_cov = tr.angle.P;
end
if starts_with(tr.mode, '3d') && tr.space.valid
    o.output_dim = 3; o.state3d = tr.space.x; o.cov3d = tr.space.P;
    o.position_enu = tr.space.x([1, 4, 7]);
    o.velocity_enu = tr.space.x([2, 5, 8]);
    o.acceleration_enu = tr.space.x([3, 6, 9]);
    ae = relative_to_ae(o.position_enu, sensor);
    o.az_deg = ae(1); o.el_deg = ae(2); o.range_m = tr.quality.range_m;
elseif starts_with(tr.mode, '2d') && tr.angle.valid
    o.output_dim = 2; o.az_deg = tr.angle.x(1); o.el_deg = tr.angle.x(4);
end
end

function o = output_template()
o = struct('id', 0, 'birth_event', 0, 't_sec', NaN, 'mode', '', ...
    'output_dim', 0, 'confirmed', false, 'truth_id', NaN, ...
    'az_deg', NaN, 'el_deg', NaN, 'range_m', NaN, ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9), ...
    'position_enu', nan(3, 1), 'velocity_enu', nan(3, 1), ...
    'acceleration_enu', nan(3, 1), 'quality', quality_template());
end

function est = init_estimate(K, events, p)
est = struct();
est.X = cell(K, 1); est.P = cell(K, 1); est.L = cell(K, 1); est.N = zeros(K, 1);
est.X2 = cell(K, 1); est.P2 = cell(K, 1); est.L2 = cell(K, 1); est.N2 = zeros(K, 1);
est.N_total = zeros(K, 1); est.logical_tracks = cell(K, 1); est.output = cell(K, 1);
est.tracks = cell(K, 1); est.assoc = cell(K, 1); est.quality_history = cell(K, 1);
est.measurement_disposition = cell(K, 1);
est.measurement_disposition_codes = struct( ...
    'associated', uint8(1), 'birth', uint8(2), 'suppressed_explained', uint8(3));
if K > 0 && isstruct(events) && isfield(events, 't_sec')
    est.filter_times = reshape([events.t_sec], [], 1);
else
    est.filter_times = zeros(K, 1);
end
est.history_level = p.history_level;
if strcmp(p.history_level, 'full')
    est.event_meta = events;
else
    est.event_meta = [];
end
est.mode_counts = struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1), 'n3d_warn', zeros(K, 1), ...
    'ndrop_request', zeros(K, 1));
est.timing = struct('total', 0);
end

function t = primary_event_time(e)
if e.has_active && ~isempty(e.active.t_sec)
    t = median(e.active.t_sec);
elseif e.has_passive && ~isempty(e.passive.t_sec)
    t = median(e.passive.t_sec);
else
    t = e.t_sec;
end
end

function t = passive_event_time(e, fallback)
if e.has_passive && ~isempty(e.passive.t_sec)
    % Active keeps processing priority inside the synchronization window;
    % never propagate the filter backward if the passive stamp is earlier.
    t = max(median(e.passive.t_sec), fallback);
else
    t = fallback;
end
end

function a = assoc_template()
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, 'meas_index', zeros(1, 0), ...
    'tid', zeros(1, 0), 'src', zeros(1, 0), 'ang', zeros(2, 0), 'xyz', zeros(3, 0), ...
    'cost', zeros(1, 0), 'measurement_dim', zeros(1, 0), ...
    'filter_dim', zeros(1, 0), ...
    'update_accepted', false(1, 0), 'range_updated', false(1, 0), ...
    'space_nis', zeros(1, 0));
end

function a = append_assoc(a, id, type, mi, tid, ang, xyz, cost, ...
        measurement_dim, update_accepted, range_updated, space_nis, source)
a.id(end + 1) = id; a.type{end + 1} = type; a.meas_index(end + 1) = mi;
a.tid(end + 1) = tid; a.src(end + 1) = source;
a.ang(:, end + 1) = ang; a.xyz(:, end + 1) = xyz;
a.cost(end + 1) = cost;
a.measurement_dim(end + 1) = measurement_dim;
a.filter_dim(end + 1) = 0;
a.update_accepted(end + 1) = update_accepted;
a.range_updated(end + 1) = range_updated;
a.space_nis(end + 1) = space_nis;
end

function a = attach_assoc_filter_dimensions(a, tracks)
if isempty(a.id), return; end
track_ids = reshape([tracks.id], 1, []);
track_dims = zeros(1, numel(tracks));
for j = 1:numel(tracks)
    if starts_with(tracks(j).mode, '3d')
        track_dims(j) = 3;
    elseif starts_with(tracks(j).mode, '2d')
        track_dims(j) = 2;
    elseif tracks(j).space.valid
        track_dims(j) = 3;
    elseif tracks(j).angle.valid
        track_dims(j) = 2;
    end
end
[present, location] = ismember(a.id, track_ids);
a.filter_dim(present) = track_dims(location(present));
end

function a = remap_assoc_ids(a, id_map)
if isempty(id_map) || isempty(a.id), return; end
for q = 1:numel(a.id)
    id = a.id(q);
    for iteration = 1:size(id_map, 1)
        j = find(id_map(:, 1) == id, 1, 'last');
        if isempty(j), break; end
        id = id_map(j, 2);
    end
    a.id(q) = id;
end
end

function tr = vote_truth_id(tr, tid)
if ~isfinite(tid), return; end
if ~isfinite(tr.truth_id) || tr.truth_conf <= 0
    tr.truth_id = tid; tr.truth_conf = 1;
elseif tr.truth_id == tid
    tr.truth_conf = min(tr.truth_conf + 1, 100);
else
    tr.truth_conf = tr.truth_conf - 1;
    if tr.truth_conf <= 0, tr.truth_id = tid; tr.truth_conf = 1; end
end
end

function tf = active_measurement_explained(tracks, C, j, attempted_rows, has_range, gate)
rows = setdiff(1:size(C, 1), unique(attempted_rows(:).'));
if has_range && ~isempty(rows)
    rows = rows(arrayfun(@(q) tracks(q).space.valid, rows));
end
tf = ~isempty(rows) && j <= size(C, 2) && ...
    any(isfinite(C(rows, j)) & C(rows, j) <= gate);
end

function tf = explained_available_measurement(C, j, attempted_rows, gate)
rows = setdiff(1:size(C, 1), unique(attempted_rows(:).'));
tf = ~isempty(rows) && j <= size(C, 2) && ...
    any(isfinite(C(rows, j)) & C(rows, j) <= gate);
end

function groups = measurement_source_groups(meas)
n = meas.n_meas;
groups = cell(1, 0);
if n == 0, return; end
src = measurement_row_field(meas, 'src', n, NaN);
keys = unique(src(isfinite(src)));
for q = 1:numel(keys)
    groups{end + 1} = find(src == keys(q)); %#ok<AGROW>
end
unknown = find(~isfinite(src));
if ~isempty(unknown)
    groups{end + 1} = unknown;
end
end

function subset = subset_active_measurements(meas, indices)
subset = meas;
subset.t_sec = subset_row(meas.t_sec, indices, meas.n_meas, NaN);
subset.xyz = meas.xyz(:, indices);
subset.rae = meas.rae(:, indices);
subset.R_xyz = meas.R_xyz(:, :, indices);
subset.R_ae = meas.R_ae(:, :, indices);
subset.has_range = logical(subset_row(meas.has_range, indices, meas.n_meas, false));
subset.ids = subset_row(meas.ids, indices, meas.n_meas, NaN);
subset.src = subset_row(meas.src, indices, meas.n_meas, NaN);
subset.n_meas = numel(indices);
end

function subset = subset_passive_measurements(meas, indices)
subset = meas;
subset.t_sec = subset_row(meas.t_sec, indices, meas.n_meas, NaN);
subset.ang = meas.ang(:, indices);
subset.R_ae = meas.R_ae(:, :, indices);
subset.ids = subset_row(meas.ids, indices, meas.n_meas, NaN);
subset.src = subset_row(meas.src, indices, meas.n_meas, NaN);
subset.n_meas = numel(indices);
end

function values = subset_row(values, indices, n, fallback)
if isempty(values)
    values = fallback * ones(1, numel(indices));
    return;
end
values = reshape(values, 1, []);
if isscalar(values) && n > 1
    values = repmat(values, 1, n);
elseif numel(values) < n
    values = [values, fallback * ones(1, n - numel(values))];
end
values = values(indices);
end

function values = measurement_row_field(meas, name, n, fallback)
if ~isfield(meas, name)
    values = fallback * ones(1, n);
else
    values = subset_row(meas.(name), 1:n, n, fallback);
end
end

function source = measurement_source(meas, index)
source_values = measurement_row_field(meas, 'src', meas.n_meas, NaN);
source = source_values(index);
end

function tf = lifecycle_miss_opportunity(tr, event)
tf = (event.has_active && tr.seen_active) || ...
    (event.has_passive && tr.seen_passive);
end

function dim = measurement_dimension(meas, index)
dim = 2;
if index <= numel(meas.has_range) && meas.has_range(index)
    dim = 3;
end
end

function w = shift_window(w, value)
if isempty(w), w = value; else, w = [w(2:end), value]; end
end

function v = append_history(v, x, n)
v = [v, x]; if numel(v) > n, v = v(end - n + 1:end); end
end

function p = normalize_probability(p)
p(~isfinite(p) | p < 0) = 0;
if sum(p) <= realmin, p = ones(size(p)) / numel(p); else, p = p / sum(p); end
end

function y = weighted_finite(x, w, fallback)
good = isfinite(x) & isfinite(w) & w >= 0;
if ~any(good) || sum(w(good)) <= 0
    y = fallback;
else
    wg = w(good) / sum(w(good));
    y = sum(wg .* x(good));
end
end

function y = max_finite(a, b)
if isfinite(a) && isfinite(b)
    y = max(a, b);
elseif isfinite(a)
    y = a;
else
    y = b;
end
end

function t_meas = measurement_time(meas, index, fallback)
t_meas = fallback;
if ~isstruct(meas) || ~isfield(meas, 't_sec') || isempty(meas.t_sec)
    return;
end
times = meas.t_sec(:).';
if index <= numel(times) && isfinite(times(index))
    t_meas = times(index);
end
end

function t_valid = refresh_valid_range_time(t_valid, candidate)
if ~isfinite(candidate)
    return;
end
if ~isfinite(t_valid)
    t_valid = candidate;
else
    t_valid = max(t_valid, candidate);
end
end

function age = effective_range_age(t, t_valid)
if isfinite(t) && isfinite(t_valid)
    age = max(t - t_valid, 0);
else
    age = inf;
end
end

function layout = joint_space_state_layout()
% The project spatial state is axis-major CA:
% [E, vE, aE, N, vN, aN, U, vU, aU]. Centralizing this contract avoids
% embedding an assumed position index list in radial-quality calculations.
state_dim = 9;
states_per_axis = 3;
layout = struct('state_dim', state_dim, 'states_per_axis', states_per_axis, ...
    'position', 1:states_per_axis:state_dim, ...
    'velocity', 2:states_per_axis:state_dim, ...
    'acceleration', 3:states_per_axis:state_dim);
end

function [sigma_r, range_m, Pp] = radial_range_uncertainty(x, P, sensor, layout)
idx = layout.position;
if numel(x) < max(idx) || size(P, 1) < max(idx) || size(P, 2) < max(idx)
    error('run_filter_joint_2d3d:InvalidSpaceStateLayout', ...
        'Spatial state/covariance does not satisfy the declared CA state layout.');
end
position = x(idx);
Pp = make_spd(P(idx, idx));
rel = position - sensor;
range_m = norm(rel);
range_epsilon_m = 1e-6 * max([1; norm(position); norm(sensor)]);
if ~isfinite(range_m) || range_m <= range_epsilon_m || any(~isfinite(Pp(:)))
    sigma_r = inf;
    return;
end
u = rel / range_m;
radial_variance = real(u' * Pp * u);
if ~isfinite(radial_variance)
    sigma_r = inf;
else
    sigma_r = sqrt(max(radial_variance, 0));
end
end

function L = gaussian_likelihood(nu, S)
d = numel(nu); S = make_spd(S);
logL = -0.5 * (nu' * (S \ nu) + log(max(det(S), realmin)) + d * log(2*pi));
L = max(exp(max(logL, log(realmin))), realmin);
end

function nis = gated_nis(nu, S, gate)
if fast_gate_reject(nu, S, gate)
    nis = inf;
    return;
end
S = make_spd(S);
if any(~isfinite(S(:)))
    nis = inf;
else
    nis = nu' * (S \ nu);
    if nis > gate, nis = inf; end
end
end

function tf = fast_gate_reject(nu, S, gate)
tf = false;
if ~isfinite(gate) || any(~isfinite(nu)) || any(~isfinite(S(:))), return; end
S = 0.5 * (S + S');
lambda_upper = trace(S);
if lambda_upper > 0
    tf = sum(nu.^2) > gate * lambda_upper;
end
end

function sensor = platform_enu(t, platform, cfg)
if isempty(platform) || ~isfield(platform, 'interp_lat')
    sensor = zeros(3, 1); return;
end
lat = platform.interp_lat(t); lon = platform.interp_lon(t); alt = platform.interp_alt(t);
if ischar(cfg.local_origin) && strcmp(cfg.local_origin, 'first_platform')
    lat0 = platform.lat_deg(1); lon0 = platform.lon_deg(1); alt0 = platform.alt_m(1);
else
    lat0 = cfg.local_origin(1); lon0 = cfg.local_origin(2); alt0 = cfg.local_origin(3);
end
ecef = llh_to_ecef(lat, lon, alt); ecef0 = llh_to_ecef(lat0, lon0, alt0);
sensor = ecef_to_enu_rot(lat0, lon0) * (ecef - ecef0);
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137.0; f = 1 / 298.257223563; e2 = f * (2 - f);
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
N = a / sqrt(1 - e2 * sin(lat)^2);
ecef = [(N + alt_m) * cos(lat) * cos(lon); ...
    (N + alt_m) * cos(lat) * sin(lon); ...
    (N * (1 - e2) + alt_m) * sin(lat)];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
R = [-sin(lon), cos(lon), 0; ...
    -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat); ...
    cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)];
end

function A = make_spd(A)
A = 0.5 * (A + A');
if any(~isfinite(A(:))), return; end
[~, flag] = chol(A);
if flag == 0, return; end
[V, D] = eig(A); d = max(real(diag(D)), 1e-10);
A = real(V * diag(d) * V'); A = 0.5 * (A + A');
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function a = wrap_az(a)
a = mod(a + 180, 360) - 180;
end

function tf = starts_with(s, prefix)
tf = ischar(s) && numel(s) >= numel(prefix) && strcmp(s(1:numel(prefix)), prefix);
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end
