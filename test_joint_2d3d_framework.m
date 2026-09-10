function test_joint_2d3d_framework()
%TEST_JOINT_2D3D_FRAMEWORK Regression tests for the unified track schema.

fprintf('Running joint 2-D/3-D framework tests...\n');
test_platform_time_coverage();
test_pure_passive_wrap();
test_pure_active_3d();
test_2d_to_3d_same_id();
test_active_ae_only_stays_2d();
test_active_ae_only_updates_existing_space();
test_active_ae_only_does_not_refresh_space_shadow();
test_3d_degrades_to_2d();
test_shard_event_dedup();
test_target_id_does_not_control_dedup();
test_independent_sources_share_logical_track();
test_multisource_cardinality_at_scale();
test_birth_covariance_preserves_cross_terms();
test_output_history_level_contract();
test_scan_pairing_preserves_causal_time();
test_cohere_preallocated_sources();
test_condensation_is_independent_of_passive_timing();
test_condensation_assignment_preserves_cardinality();
test_valid_zero_boresight_is_not_placeholder();
test_empty_and_nonmonotonic_contract();
test_time_parser_and_midnight_unwrap();
test_joint_config_validation();
test_synchronized_event_counts_once();
test_long_gap_enters_hold();
test_nis_quality_is_diagnostic_only();
test_3d_rejects_angle_only_range_outlier();
test_2d_space_shadow_rejects_range_outlier();
test_spatial_candidate_precedes_confirmed_angle_track();
test_unrelated_passive_events_do_not_exhaust_active_candidate();
test_merge_remaps_current_association();
test_confirmed_tracks_are_never_auto_merged();
test_close_targets_not_merged();
test_active_range_continuous_normal();
test_active_range_long_missing_degrades();
test_active_range_short_missing_then_recovers();
test_active_ae_only_does_not_refresh_valid_range();
test_failed_active_association_does_not_refresh_valid_range();
test_radial_only_does_not_request_degradation();
test_range_age_only_does_not_request_degradation();
test_radial_and_range_age_request_degradation();
test_dual_threshold_without_2d_takeover_enters_hold();
test_metrics_and_plots();
test_joint_metric_dimension_separation();
test_formal_output_coverage_is_separate();
test_joint_metric_contract();
test_track_correctness_uses_truth_denominator();
test_joint_metric_cross_dimension_flow();
test_start_delay_not_backdated_before_correct_association();
test_birth_measurements_are_counted();
test_capacity_deletion_is_accounted();
test_measurement_dispositions_without_formal_output();
test_joint_truth_target_count();
test_split_truth_labels_drive_metrics_and_viewer();
test_plot_mode_separation();
test_assignment_index_orientation();
test_exact_global_assignment();
test_sparse_component_assignment();
test_random_global_assignment();
test_random_mixed_sequence_invariants();
fprintf('Joint 2-D/3-D framework tests passed.\n');
end

function test_empty_and_nonmonotonic_contract()
cfg = test_cfg(); platform = static_platform();
est = run_filter_joint_2d3d(repmat(empty_event(), 0, 1), platform, cfg);
assert(strcmp(est.framework, 'joint_2d3d') && isempty(est.transition_log) && ...
    isempty(est.output), 'Empty input returned an incomplete estimate contract.');
events = [passive_event(1, 1, [10; 2], 1); passive_event(2, 0, [10; 2], 1)];
failed = false;
try
    run_filter_joint_2d3d(events, platform, cfg);
catch ME
    failed = strcmp(ME.identifier, 'run_filter_joint_2d3d:NonmonotonicEvents');
end
assert(failed, 'Nonmonotonic events were silently processed with zero elapsed time.');
end

function test_joint_config_validation()
cfg = test_cfg();
cfg.data_dir = pwd; cfg.active_files = {'config_fusion.m'};
cfg.passive_files = {'config_fusion.m'}; cfg.platform_file = 'config_fusion.m';
cfg = validate_config_fusion(cfg);
assert(strcmp(cfg.processing_framework, 'joint_2d3d'));
cfg.metrics_max_print = 0;
cfg = validate_config_fusion(cfg);
assert(cfg.metrics_max_print == 0, 'metrics_max_print=0 should disable printing.');
bad = cfg; bad.joint_pos95_recover_m = -1;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'Negative 3-D quality thresholds passed configuration validation.');
bad = cfg; bad.joint_radial_sigma_warn_m = bad.joint_radial_sigma_drop_m;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'Invalid radial sigma warn/drop ordering passed validation.');
bad = cfg; bad.joint_range_age_warn_s = bad.joint_range_age_drop_s;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'Invalid reliable-range age warn/drop ordering passed validation.');
bad = cfg; bad.joint_unmatched_cost = min(cfg.joint_gate_2d, cfg.joint_gate_3d) / 2;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'An unmatched cost below the association gate passed validation.');
bad = cfg; bad.joint_plot_min_life = [1, 2];
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'A non-scalar joint plot length passed configuration validation.');
bad = cfg; bad.joint_sync_tolerance_s = bad.joint_scan_tolerance_s + 0.01;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'A synchronization tolerance above the scan tolerance passed validation.');
bad = cfg; bad.merge_pos_dist_m = -1;
failed = false;
try
    validate_config_fusion(bad);
catch
    failed = true;
end
assert(failed, 'A negative legacy merge radius passed configuration validation.');
end

function test_time_parser_and_midnight_unwrap()
assert(abs(parse_fusion_time('14:30:00.250', 'hms') - 52200.25) < 1e-12);
assert(abs(parse_fusion_time('143000.250', 'hms') - 52200.25) < 1e-12, ...
    'Compact numeric HMS was interpreted as elapsed seconds.');
assert(isnan(parse_fusion_time('14:61:00', 'hms')), ...
    'An invalid HMS timestamp was accepted.');
t = unwrap_fusion_clock_times([86399; 1; 2]);
assert(isequal(t, [86399; 86401; 86402]), ...
    'Midnight rollover was not expanded monotonically.');
end

function test_cohere_preallocated_sources()
cfg = test_cfg(); platform = static_platform();
a = struct('t_sec', [0; 0.02], 'az_deg', [10; 11], 'el_deg', [2; 2], ...
    'range_m', [50000; NaN], 'range_valid', [true; false], ...
    'target_id', [101; 102], 'source', 'active.txt', 'n_meas', 2);
p1 = struct('t_sec', [0.008; 0.028], 'az_deg', [10; 11], 'el_deg', [2; 2], ...
    'target_id', [101; 102], 'source', 'p1.txt', 'n_meas', 2);
p2 = p1; p2.source = 'p2.txt';
frames = cohere_measurements({a}, {p1, p2}, platform, cfg);
assert(sum(arrayfun(@(f) size(f.active_xyz, 2), frames)) == 1);
assert(sum(arrayfun(@(f) size(f.active_ae_only, 2), frames)) == 1);
assert(sum(arrayfun(@(f) size(f.passive_ang, 2), frames)) == 4);
end

function test_condensation_assignment_preserves_cardinality()
cfg = test_cfg(); cfg.frame_time_window_s = 0.01;
cfg.condense_enable = true; cfg.condense_method = 'spatiotemporal';
cfg.condense_gate_gamma = 1e6; cfg.condense_birth_merge_enabled = false;
cfg.condense_res_range_m = 1; cfg.condense_res_az_deg = 1e-6;
cfg.condense_res_el_deg = 1e-6;
a1 = struct('t_sec', [0; 1], 'az_deg', [10; 10], 'el_deg', [2; 2], ...
    'range_m', [50000; 50000], 'range_valid', [true; true], ...
    'target_id', [1; 1], 'source', 'a1.txt', 'n_meas', 2);
a2 = struct('t_sec', 1, 'az_deg', 10, 'el_deg', 2, ...
    'range_m', 50200, 'range_valid', true, ...
    'target_id', 2, 'source', 'a2.txt', 'n_meas', 1);
frames = cohere_measurements({a1, a2}, {}, static_platform(), cfg);
assert(numel(frames) == 2 && size(frames(2).active_xyz, 2) == 2, ...
    'Two measurements were assigned to one condensation micro-track.');
for i = 1:size(frames(2).active_xyz, 2)
    assert(abs(frames(2).active_rae(1, i) - ...
        norm(frames(2).active_xyz(:, i))) < 1e-6, ...
        'Condensed XYZ and RAE do not represent the same physical point.');
end
end

function test_valid_zero_boresight_is_not_placeholder()
cfg = test_cfg();
active_file = [tempname, '.txt']; passive_file = [tempname, '.txt'];
cleanup = onCleanup(@() delete_test_files(active_file, passive_file)); %#ok<NASGU>

fields = repmat({''}, 1, 23);
fields{1} = '00:00:00.000'; fields{5} = '1'; fields{6} = '7';
fields{12} = '1'; fields{13} = '1'; fields{14} = '1';
fields{19} = '0'; fields{20} = '0'; fields{23} = '5000';
write_test_line(active_file, strjoin(fields, ','));

fields = repmat({''}, 1, 36);
fields{1} = '00:00:00.000'; fields{5} = '1'; fields{6} = '7';
fields{33} = '1'; fields{34} = '1'; fields{35} = '0'; fields{36} = '0';
write_test_line(passive_file, strjoin(fields, ','));

active = parse_active_wide_txt(active_file, cfg);
passive = parse_passive_wide_txt(passive_file, cfg);
assert(active.n_meas == 1 && active.az_deg(1) == 0 && active.el_deg(1) == 0, ...
    'A valid active boresight measurement was treated as a zero placeholder.');
assert(passive.n_meas == 1 && passive.az_deg(1) == 0 && passive.el_deg(1) == 0, ...
    'A valid passive boresight measurement was treated as a zero placeholder.');
end

function write_test_line(path, line)
fid = fopen(path, 'w'); assert(fid >= 0, 'Could not create parser test file.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', line);
end

function delete_test_files(varargin)
for q = 1:nargin
    if exist(varargin{q}, 'file') == 2, delete(varargin{q}); end
end
end

function test_close_targets_not_merged()
cfg = test_cfg(); platform = static_platform();
cfg.joint_merge_angle_deg = 0.08;
events = repmat(empty_event(), 5, 1);
for k = 1:5
    e = passive_event(k, k - 1, [10 + 0.02*k; 2], 91);
    e.passive.ang = [10 + 0.02*k, 10.06 + 0.02*k; 2, 2.01];
    e.passive.R_ae = repmat(diag([0.005^2, 0.005^2]), 1, 1, 2);
    e.passive.ids = [91, 92]; e.passive.src = [1, 1]; e.passive.n_meas = 2;
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(est.N2(end) == 2 && numel(est.logical_tracks{end}) == 2, ...
    'Distinct close-angle measurements were merged into one logical track.');
end

function test_long_gap_enters_hold()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_confirmed_timeout_s = 100;
cfg.joint_angle_q_cv = 1e-4; cfg.joint_angle_q_ca = 1e-4;
cfg.joint_angle_rate_birth_std_dps = 0.01;
cfg.joint_angle_acc_birth_std_dps2 = 0.01;
cfg.joint_angle95_max_deg = 0.5;
cfg.joint_max_predict_dt_s = 1;
events = repmat(empty_event(), 2, 1);
events(1) = passive_event(1, 0, [20; 3], 81);
events(2).cycle_id = 2; events(2).t_sec = 20;
events(2).t_start = 20; events(2).t_end = 20;
est = run_filter_joint_2d3d(events, platform, cfg);
assert(est.N2(1) == 1 && est.N_total(2) == 0, ...
    'An angle track with invalid long-gap uncertainty remained a formal output.');
assert(strcmp(est.logical_tracks{2}(1).mode, 'hold'), ...
    'Long-gap angle uncertainty did not move the logical track to hold.');
assert(est.logical_tracks{2}(1).angle_cov(1, 1) > 0.1, ...
    'Prediction did not propagate across the complete long time interval.');
end

function test_nis_quality_is_diagnostic_only()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_down_consecutive = 2; cfg.joint_angle95_max_deg = 10;
cfg.joint_pos95_recover_m = 1e8; cfg.joint_pos95_warn_m = 2e8; cfg.joint_pos95_down_m = 3e8;
cfg.joint_radial95_recover_m = 1e8; cfg.joint_radial95_warn_m = 2e8; cfg.joint_radial95_down_m = 3e8;
cfg.joint_relative_range_down = 10;
cfg.joint_space_nis_window = 1;
cfg.joint_space_nis_recover = 1e-4;
cfg.joint_space_nis_warn = 2e-4;
cfg.joint_space_nis_down = 3e-4;
cfg.joint_radial_sigma_warn_m = 1e8;
cfg.joint_radial_sigma_drop_m = 2e8;
cfg.joint_range_age_warn_s = 10;
cfg.joint_range_age_drop_s = 20;
events = repmat(empty_event(), 4, 1);
base = ae_to_xyz(12, 3, 50000);
for k = 1:4
    events(k) = active_event(k, 0.1*(k - 1), base + [120*(k - 1); 0; 0], 82);
end
est = run_filter_joint_2d3d(events, platform, cfg);
reasons = {est.transition_log.reason};
q = est.logical_tracks{end}(1).quality;
assert(isfinite(q.nis_norm) && q.nis_norm > cfg.joint_space_nis_down, ...
    'Spatial NIS diagnostic was not retained.');
assert(~q.degrade_request && ~any(strcmp(reasons, '3d_quality_degraded')), ...
    'Spatial NIS still directly participated in 3-D degradation.');
end

function test_3d_rejects_angle_only_range_outlier()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_confirmed_timeout_s = 100;
near = ae_to_xyz(25, 4, 50000);
far = ae_to_xyz(25, 4, 200000);
events = [active_event(1, 0, near, 83); active_event(2, 0.1, far, 84)];
est = run_filter_joint_2d3d(events, platform, cfg);
out = est.output{2};
j = find([out.id] == 1, 1);
assert(~isempty(j) && out(j).output_dim == 3, 'The original 3-D track was lost.');
assert(norm(out(j).position_enu - near) < 2000, ...
    'A range outlier that only passed the angle gate corrupted the 3-D state.');
assert(numel(est.logical_tracks{2}) == 2 && est.stats.active_births == 2, ...
    'A spatially incompatible collinear target did not start a separate track.');
end

function test_2d_space_shadow_rejects_range_outlier()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_upgrade_M = 3; cfg.joint_3d_upgrade_N = 4;
cfg.joint_confirmed_timeout_s = 100;
near = ae_to_xyz(25, 4, 50000);
far = ae_to_xyz(25, 4, 200000);
events = [passive_event(1, 0, [25; 4], 201); ...
    active_event(2, 0.1, near, 201); ...
    active_event(3, 0.2, far, 202)];
est = run_filter_joint_2d3d(events, platform, cfg);
original = logical_track_by_id(est, 3, 1);
a = est.assoc{3};
assert(abs(original.last_valid_range_t - 0.1) < 1e-12, ...
    'A 2-D-mode track consumed a spatially rejected range measurement.');
assert(numel(est.logical_tracks{3}) == 2 && ...
       any(strcmp(a.type, 'active_birth')) && ~any(a.id == 1), ...
    'A spatially inconsistent range measurement did not start a new 3-D track.');
end

function test_spatial_candidate_precedes_confirmed_angle_track()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 2; cfg.joint_confirm_N = 3;
cfg.joint_3d_birth_M = 3; cfg.joint_3d_birth_N = 4;
events = [passive_event(1, 0, [10; 3], 301); ...
    passive_event(2, 0.1, [10.01; 3], 301); ...
    active_event(3, 0.2, ae_to_xyz(20, 3, 50000), 302); ...
    active_event(4, 0.3, ae_to_xyz(10, 3, 50000), 302)];
events(4).active.R_xyz = diag([10000^2, 10000^2, 10000^2]);
est = run_filter_joint_2d3d(events, platform, cfg);
a = est.assoc{4};
j = find(strcmp(a.type, 'active'), 1);
assert(~isempty(j) && a.id(j) == 2 && a.range_updated(j), ...
    'A confirmed angle-only track pre-empted a spatially compatible candidate.');
end

function test_unrelated_passive_events_do_not_exhaust_active_candidate()
cfg = test_cfg(); platform = static_platform();
cfg.joint_tentative_timeout_s = 100;
xyz = ae_to_xyz(15, 3, 50000);
events = repmat(empty_event(), 9, 1);
events(1) = active_event(1, 0, xyz, 401);
for k = 2:4
    events(k) = passive_event(k, 0.1*(k - 1), [100; 10], 402);
end
events(5) = active_event(5, 0.4, xyz, 401);
for k = 6:8
    events(k) = passive_event(k, 0.1*(k - 1), [100; 10], 402);
end
events(9) = active_event(9, 0.8, xyz, 401);
est = run_filter_joint_2d3d(events, platform, cfg);
out = est.output{9};
j = find([out.id] == 1, 1);
assert(~isempty(j) && out(j).output_dim == 3, ...
    'Unrelated passive scans exhausted the active candidate M/N window.');
end

function test_synchronized_event_counts_once()
cfg = test_cfg(); platform = static_platform();
events = repmat(empty_event(), 4, 1);
for k = 1:4
    ta = k - 1;
    xyz = ae_to_xyz(25 + 0.05*k, 4, 45000);
    e = active_event(k, ta, xyz, 71);
    e.has_passive = true;
    e.passive.t_sec = ta + 0.008;
    e.passive.ang = xyz_to_ae(xyz);
    e.passive.R_ae = diag([0.05^2, 0.04^2]);
    e.passive.ids = 71; e.passive.src = 1; e.passive.n_meas = 1;
    e.t_end = ta + 0.008;
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(est.N_total(2) == 0 && est.N_total(3) == 1, ...
    'Synchronized active/passive updates were counted as two confirmation opportunities.');
assert(max(abs(est.filter_times - ((0:3)' + 0.008))) < 1e-12, ...
    'The passive subscan timestamp was not retained as the final event state time.');
tr = logical_track_by_id(est, numel(events), 1);
assert(abs(tr.last_valid_range_t - 3) < 1e-12 && ...
    abs(tr.quality.range_age_s - 0.008) < 1e-12, ...
    'Reliable-range age did not use the active measurement actual timestamp.');
assert(numel(est.logical_tracks{end}) == 1, ...
    'A synchronized active/passive pair created duplicate logical tracks.');
end

function test_condensation_is_independent_of_passive_timing()
cfg = test_cfg(); platform = static_platform();
a = struct('t_sec', [0; 0.16; 0.24; 0.32], 'az_deg', 20*ones(4, 1), ...
    'el_deg', 3*ones(4, 1), 'range_m', [50000; 50000; 60000; 60000], ...
    'range_valid', true(4, 1), 'target_id', [101; 101; 202; 202], 'n_meas', 4);
pt = [0.11; (0.001:0.01:0.3)'];
p = struct('t_sec', pt, 'az_deg', 100*ones(size(pt)), ...
    'el_deg', 8*ones(size(pt)), 'target_id', 303*ones(size(pt)), 'n_meas', numel(pt));
for enabled = [false, true]
    cfg.condense_enable = enabled;
    for method = {'spatiotemporal', 'resolution', 'radius'}
        cfg.condense_method = method{1};
        [alone, ai] = cohere_measurements({a}, {}, platform, cfg);
        [joint, ji] = cohere_measurements({a}, {p}, platform, cfg);
        passive = cohere_measurements({}, {p}, platform, cfg);
        is_active = arrayfun(@(f) ~isempty(f.active_xyz), joint);
        is_passive = arrayfun(@(f) ~isempty(f.passive_ang), joint);
        assert(isequaln(alone, joint(is_active)), ...
            'Passive timing changed active condensation, covariance or representative time.');
        assert(isequaln(passive, joint(is_passive)), ...
            'Active timing changed passive frame membership or timestamps.');
        assert(ji.n_active_input == 4 && ji.n_active_after_condensation == 4 && ...
            ji.n_active_condensed == 0 && ai.n_active_after_condensation == 4 && ...
            ji.n_passive_after_condensation == numel(pt));
        ea = build_joint_measurement_events(alone, cfg);
        ej = build_joint_measurement_events(joint, cfg);
        assert(isequaln([ea.active], [ej([ej.has_active]).active]), ...
            'Adding passive data changed the active stream consumed by the joint filter.');
    end
end

% Same-modality condensation remains enabled and explicitly counted.
a.t_sec(4) = 0.25;
cfg.condense_enable = true;
[~, info] = cohere_measurements({a}, {p}, platform, cfg);
assert(info.n_active_input == 4 && info.n_active_after_condensation == 3 && ...
    info.n_active_condensed == 1 && info.n_passive_after_condensation == numel(pt), ...
    'Independent framing disabled intended active condensation or lost passive measurements.');
end

function test_merge_remaps_current_association()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 5; cfg.joint_confirm_N = 6;
cfg.joint_gate_2d = 0.05;
cfg.joint_merge_angle_deg = 1;
cfg.joint_merge_rate_dps = 10;
cfg.joint_merge_nis = 1e6;
events = repmat(empty_event(), 4, 1);
events(1) = passive_event(1, 0, [10; 2], 501);
events(2) = passive_event(2, 0.1, [10; 2], 501);
events(3) = passive_event(3, 0.2, [10; 2], 501);
events(3).passive.ang = [10, 10.2; 2, 2];
events(3).passive.R_ae = repmat(diag([0.01^2, 0.01^2]), 1, 1, 2);
events(3).passive.ids = [501, 501];
events(3).passive.src = [1, 1]; events(3).passive.n_meas = 2;
events(4) = passive_event(4, 0.3, [10.2; 2], 501);
events(4).passive.R_ae = diag([0.01^2, 0.01^2]);
est = run_filter_joint_2d3d(events, platform, cfg);
alive = [est.logical_tracks{4}.id];
assert(isequal(alive, 1) && all(ismember(est.assoc{4}.id, alive)), ...
    'A duplicate merge left the current association on a removed logical ID.');
assert(est.stats.duplicates_merged >= 1, ...
    'A duplicate merge that removed the outer-loop track was not counted.');
end

function test_confirmed_tracks_are_never_auto_merged()
cfg = range_quality_cfg(); platform = static_platform();
cfg.joint_confirm_M = 2; cfg.joint_confirm_N = 2;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_gate_3d = 1e6;
cfg.joint_merge_angle_deg = 1;
cfg.joint_merge_rate_dps = 10;
cfg.joint_merge_nis = 1e6;
p1 = ae_to_xyz(10, 2, 50000);
p2 = ae_to_xyz(10, 2, 50200);
events = [multi_active_event(1, 0, [p1, p2], [901, 902]); ...
          multi_active_event(2, 1, [p1, p2], [901, 902]); ...
          active_event(3, 2, p1, 901)];
est = run_filter_joint_2d3d(events, platform, cfg);
tracks = est.logical_tracks{3};
assert(numel(tracks) == 2 && nnz([tracks.confirmed]) == 2, ...
    'A confirmed track was removed by automatic duplicate merging.');
assert(est.stats.duplicates_merged == 0, ...
    'Confirmed-track proximity was incorrectly counted as duplicate merging.');
end

function test_assignment_index_orientation()
cfg = range_quality_cfg(); platform = static_platform();
xyz0 = [ae_to_xyz(-20, 3, 50000), ae_to_xyz(35, 5, 70000)];
xyz1 = [ae_to_xyz(-19.99, 3, 50020), ae_to_xyz(35.01, 5, 70020)];
events = [multi_active_event(1, 0, xyz0, [701, 702]); ...
          multi_active_event(2, 0.1, xyz1, [701, 702])];

est = run_filter_joint_2d3d(events, platform, cfg);
a = est.assoc{2};
assert(numel(a.id) == 2 && numel(unique(a.id)) == 2 && ...
       numel(unique(a.meas_index)) == 2 && all(strcmp(a.type, 'active')), ...
    'Multi-track active assignment did not return two valid N-by-2 index pairs.');
assert(est.stats.active_births == 2 && est.stats.active_assigned == 2, ...
    'Multi-track active assignment changed birth or association accounting.');
end

function test_exact_global_assignment()
C = [1, 2; 1.1, 100];
pairs = solve_global_assignment(C, 50);
assert(isequal(sortrows(pairs), [1, 2; 2, 1]), ...
    'Assignment fallback is greedy rather than globally optimal.');
end

function test_sparse_component_assignment()
C = inf(6, 7); unmatched = 10;
C(1, 1:2) = [1, 4]; C(2, 1:2) = [2, 1];
C(3, 4) = 3; C(4, 5) = 50;
C(5, 6:7) = [5, 2]; C(6, 6:7) = [1, 5];
pairs = solve_global_assignment(C, unmatched);
expected = [1, 1; 2, 2; 3, 4; 5, 7; 6, 6];
assert(isequal(pairs, expected), ...
    'Disconnected gated assignment components changed the global optimum.');
end

function test_random_global_assignment()
old_rng = rng; cleanup = onCleanup(@() rng(old_rng)); %#ok<NASGU>
rng(17);
for trial = 1:80
    n = randi(4); m = randi(4); unmatched = 8;
    C = 20 * rand(n, m); C(rand(n, m) < 0.25) = inf;
    pairs = solve_global_assignment(C, unmatched);
    assert(numel(unique(pairs(:, 1))) == size(pairs, 1));
    assert(numel(unique(pairs(:, 2))) == size(pairs, 1));
    got = unmatched * (n - size(pairs, 1));
    for q = 1:size(pairs, 1), got = got + C(pairs(q, 1), pairs(q, 2)); end
    expected = brute_assignment_cost(C, unmatched, 1, false(1, m));
    assert(abs(got - expected) < 1e-9, ...
        'Global assignment differs from exhaustive optimum on trial %d.', trial);
end
end

function best = brute_assignment_cost(C, unmatched, row, used)
if row > size(C, 1), best = 0; return; end
best = unmatched + brute_assignment_cost(C, unmatched, row + 1, used);
for j = 1:size(C, 2)
    if used(j) || ~isfinite(C(row, j)), continue; end
    next_used = used; next_used(j) = true;
    best = min(best, C(row, j) + ...
        brute_assignment_cost(C, unmatched, row + 1, next_used));
end
end

function test_random_mixed_sequence_invariants()
old_rng = rng; cleanup = onCleanup(@() rng(old_rng)); %#ok<NASGU>
rng(23); cfg = test_cfg(); platform = static_platform();
K = 36; events = repmat(empty_event(), K, 1);
base_az = [-55, 5, 70]; el = [2, 4, -1]; ranges = [45000, 60000, 80000];
for k = 1:K
    t = 0.2 * (k - 1); e = empty_event();
    e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
    true_az = base_az + [0.05, -0.03, 0.02] * k;
    if mod(k, 5) ~= 0
        e.has_passive = true; e.passive.n_meas = 3;
        e.passive.t_sec = t + 0.005 * ones(1, 3);
        e.passive.ang = [true_az + 0.02*randn(1, 3); el + 0.015*randn(1, 3)];
        e.passive.R_ae = repmat(diag([0.05^2, 0.04^2]), 1, 1, 3);
        e.passive.ids = 201:203; e.passive.src = ones(1, 3); e.t_end = t + 0.005;
    end
    if k >= 9 && mod(k, 2) == 0
        e.has_active = true; e.active.n_meas = 3;
        e.active.t_sec = t * ones(1, 3); e.active.ids = 201:203;
        e.active.src = ones(1, 3); e.active.has_range = true(1, 3);
        e.active.xyz = zeros(3, 3); e.active.rae = zeros(3, 3);
        e.active.R_xyz = repmat(diag([100^2, 100^2, 100^2]), 1, 1, 3);
        e.active.R_ae = repmat(diag([0.08^2, 0.06^2]), 1, 1, 3);
        for j = 1:3
            xyz = ae_to_xyz(true_az(j), el(j), ranges(j) + 20*k);
            e.active.xyz(:, j) = xyz;
            e.active.rae(:, j) = [norm(xyz); xyz_to_ae(xyz)];
        end
    end
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert_estimate_invariants(est);
assert(any(est.N2 > 0) && any(est.N > 0), ...
    'Random mixed sequence did not exercise both formal dimensions.');
end

function assert_estimate_invariants(est)
n_range_updated = 0;
for k = 1:numel(est.output)
    out = est.output{k}; dims = [out.output_dim]; ids = [out.id];
    assert(numel(unique(ids)) == numel(ids), 'Duplicate logical ID in one output event.');
    assert(est.N_total(k) == numel(out) && est.N_total(k) == est.N2(k) + est.N(k));
    assert(size(est.X2{k}, 2) == est.N2(k) && size(est.L2{k}, 1) == est.N2(k));
    assert(size(est.X{k}, 2) == est.N(k) && size(est.L{k}, 1) == est.N(k));
    assert(all(dims == 2 | dims == 3));
    for q = 1:numel(out)
        assert(all(isfinite([out(q).az_deg; out(q).el_deg])));
        assert_spd(out(q).angle_cov);
        if out(q).output_dim == 3
            assert(all(isfinite(out(q).state3d)) && all(isfinite(out(q).position_enu)));
            assert_spd(out(q).cov3d);
        end
    end
    a = est.assoc{k};
    alive_ids = [est.logical_tracks{k}.id];
    assert(all(ismember(a.id, alive_ids)), ...
        'An association references a logical ID removed in the same event.');
    if isfield(a, 'measurement_dim') && isfield(a, 'range_updated')
        range_assoc = a.measurement_dim == 3;
        assert(all(a.update_accepted(range_assoc)) && all(a.range_updated(range_assoc)), ...
            'An associated active range measurement did not update the spatial branch.');
        n_range_updated = n_range_updated + nnz(a.range_updated);
    end
end
if isfield(est, 'stats') && isfield(est.stats, 'active_range_updates')
    assert(n_range_updated == est.stats.active_range_updates && ...
           est.stats.active_range_unaccounted == 0 && ...
           est.stats.active_unaccounted == 0 && est.stats.passive_unaccounted == 0, ...
        'Measurement accounting fields are inconsistent with association history.');
end
end

function assert_spd(P)
assert(all(isfinite(P(:))) && norm(P - P', 'fro') < 1e-7 * max(norm(P, 'fro'), 1));
assert(min(real(eig(0.5 * (P + P')))) > 0, 'Covariance is not positive definite.');
end

function test_active_ae_only_stays_2d()
cfg = test_cfg(); platform = static_platform();
events = repmat(empty_event(), 6, 1);
for k = 1:6
    e = active_event(k, k - 1, ae_to_xyz(20 + 0.1*k, 4, 40000), 41);
    e.active.has_range = false;
    e.active.xyz(:) = NaN; e.active.rae(1) = NaN; e.active.R_xyz(:) = NaN;
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(any(est.N2 >= 1), 'Active AE-only measurements did not confirm a 2-D track.');
assert(all(est.N == 0), 'Active AE-only measurements created a false 3-D output.');
end

function test_active_ae_only_updates_existing_space()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
events = repmat(empty_event(), 2, 1);
events(1) = active_event(1, 0, ae_to_xyz(25, 4, 50000), 43);
e = active_event(2, 0.1, ae_to_xyz(25.2, 4, 50000), 43);
e.active.has_range = false;
e.active.xyz(:) = NaN; e.active.rae(1) = NaN; e.active.R_xyz(:) = NaN;
events(2) = e;
est = run_filter_joint_2d3d(events, platform, cfg);
assert(est.N(2) == 1 && est.output{2}(1).az_deg > 25.05, ...
    'Active AE-only measurement did not update the existing 3-D bearing branch.');
end

function test_active_ae_only_does_not_refresh_space_shadow()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_down_consecutive = 1; cfg.joint_shadow_max_s = 0.5;
cfg.joint_confirmed_timeout_s = 100; cfg.joint_angle95_max_deg = 10;
cfg.joint_space_sigma_a_cv = 300; cfg.joint_space_sigma_j_ca = 400;
cfg.joint_radial_sigma_warn_m = 120;
cfg.joint_radial_sigma_drop_m = 180;
cfg.joint_range_age_warn_s = 0.1;
cfg.joint_range_age_drop_s = 0.3;
cfg.joint_pos95_recover_m = 400; cfg.joint_pos95_warn_m = 500; cfg.joint_pos95_down_m = 600;
cfg.joint_radial95_recover_m = 300; cfg.joint_radial95_warn_m = 450; cfg.joint_radial95_down_m = 550;
events = repmat(empty_event(), 6, 1);
xyz = ae_to_xyz(18, 3, 50000);
events(1) = active_event(1, 0, xyz, 42);
for k = 2:6
    e = active_event(k, 0.2*(k - 1), xyz, 42);
    e.active.has_range = false;
    e.active.xyz(:) = NaN; e.active.rae(1) = NaN; e.active.R_xyz(:) = NaN;
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(any(est.N == 1), 'Test setup never established a formal 3-D branch.');
assert(any(strcmp({est.transition_log.to}, '2d_shadow')), ...
    'Test setup never degraded the 3-D branch to a spatial shadow.');
assert(all(isnan(est.logical_tracks{end}(1).state3d)), ...
    'Active AE-only hits incorrectly kept an expired spatial shadow alive.');
end

function test_3d_degrades_to_2d()
cfg = test_cfg(); platform = static_platform();
cfg.joint_space_sigma_a_cv = 80; cfg.joint_space_sigma_j_ca = 100;
cfg.joint_pos95_recover_m = 1000; cfg.joint_pos95_warn_m = 1800; cfg.joint_pos95_down_m = 2800;
cfg.joint_radial95_recover_m = 800; cfg.joint_radial95_warn_m = 1400; cfg.joint_radial95_down_m = 2400;
cfg.joint_radial_sigma_warn_m = 800;
cfg.joint_radial_sigma_drop_m = 1200;
cfg.joint_range_age_warn_s = 2;
cfg.joint_range_age_drop_s = 5;
cfg.joint_down_consecutive = 2;
events = repmat(empty_event(), 24, 1);
xyz = ae_to_xyz(15, 3, 50000);
for k = 1:5, events(k) = active_event(k, k - 1, xyz, 51); end
for k = 6:22, events(k) = passive_event(k, k - 1, [15; 3], 51); end
events(23) = active_event(23, 22, xyz, 51);
events(24) = active_event(24, 23, xyz, 51);
est = run_filter_joint_2d3d(events, platform, cfg);
to = {est.transition_log.to};
assert(any(strcmp(to, '3d')), 'Track never reached formal 3-D mode before degradation.');
assert(any(strcmp(to, '2d_shadow')), 'Radial uncertainty did not trigger 3-D to 2-D degradation.');
assert(est.N2(23) == 1 && est.N(23) == 0, ...
    'A stale active-history vote caused one-hit 3-D recovery after degradation.');
assert(est.N(24) == 1, 'Two fresh active hits did not recover the 3-D branch.');
end

function test_active_range_continuous_normal()
cfg = range_quality_cfg(); platform = static_platform();
xyz = ae_to_xyz(15, 3, 50000);
events = repmat(empty_event(), 6, 1);
for k = 1:6
    events(k) = active_event(k, 0.2 * (k - 1), xyz, 101);
end
est = run_filter_joint_2d3d(events, platform, cfg);
tr = logical_track_by_id(est, numel(events), 1);
assert(strcmp(tr.mode, '3d') && ~tr.quality.warn3d && ...
    ~tr.quality.degrade_request, ...
    'Continuous reliable range updates did not keep normal 3-D output.');
assert(abs(tr.last_valid_range_t - events(end).active.t_sec) < 1e-12 && ...
    tr.quality.range_age_s < 1e-12, ...
    'Reliable-range timestamp/age was not refreshed by a real 3-D update.');
assert(isfield(est, 'quality_history') && ...
    all(isfield(est.quality_history{end}, {'t_sec','radial_sigma_m','range_age_s','mode','degrade_request'})), ...
    'Per-track 3-D quality history contract is incomplete.');
end

function test_active_range_long_missing_degrades()
cfg = range_quality_cfg(); cfg.joint_down_consecutive = 2;
platform = static_platform(); xyz = ae_to_xyz(15, 3, 50000);
events = [active_event(1, 0, xyz, 102); ...
    passive_event(2, 2, [15; 3], 102); ...
    passive_event(3, 3, [15; 3], 102)];
est = run_filter_joint_2d3d(events, platform, cfg);
tr2 = logical_track_by_id(est, 2, 1);
tr3 = logical_track_by_id(est, 3, 1);
assert(tr2.quality.degrade_request && strcmp(tr2.mode, '3d_warn'), ...
    'First dual-threshold exceedance did not enter hysteretic 3-D warning.');
assert(strcmp(tr3.mode, '2d_shadow') && est.N2(3) == 1 && est.N(3) == 0, ...
    'Sustained loss of reliable range did not execute 3-D to 2-D takeover.');
end

function test_active_range_short_missing_then_recovers()
cfg = range_quality_cfg(); platform = static_platform();
xyz = ae_to_xyz(15, 3, 50000);
events = [active_event(1, 0, xyz, 103); ...
    passive_event(2, 1, [15; 3], 103); ...
    active_event(3, 1.5, xyz, 103)];
est = run_filter_joint_2d3d(events, platform, cfg);
mid = logical_track_by_id(est, 2, 1);
last = logical_track_by_id(est, 3, 1);
assert(mid.quality.warn3d && ~mid.quality.degrade_request, ...
    'Short range outage did not remain a warning-only condition.');
assert(strcmp(last.mode, '3d') && ~last.quality.degrade_request && ...
    abs(last.last_valid_range_t - 1.5) < 1e-12, ...
    'Reliable range recovery did not refresh quality without a false downgrade.');
end

function test_active_ae_only_does_not_refresh_valid_range()
cfg = range_quality_cfg(); platform = static_platform();
xyz = ae_to_xyz(15, 3, 50000);
e2 = active_event(2, 1.5, xyz, 104);
e2.active.has_range = false;
e2.active.xyz(:) = NaN; e2.active.rae(1) = NaN; e2.active.R_xyz(:) = NaN;
est = run_filter_joint_2d3d([active_event(1, 0, xyz, 104); e2], platform, cfg);
tr = logical_track_by_id(est, 2, 1);
assert(abs(tr.last_valid_range_t) < 1e-12 && abs(tr.quality.range_age_s - 1.5) < 1e-12, ...
    'AE-only active measurement incorrectly refreshed reliable-range time.');
end

function test_failed_active_association_does_not_refresh_valid_range()
cfg = range_quality_cfg(); platform = static_platform();
near = ae_to_xyz(15, 3, 50000); far = ae_to_xyz(15, 3, 200000);
est = run_filter_joint_2d3d([active_event(1, 0, near, 105); ...
    active_event(2, 1.5, far, 106)], platform, cfg);
original = logical_track_by_id(est, 2, 1);
assert(abs(original.last_valid_range_t) < 1e-12 && ...
    abs(original.quality.range_age_s - 1.5) < 1e-12, ...
    'Unassociated active range incorrectly refreshed the original track.');
assert(numel(est.logical_tracks{2}) == 2, ...
    'Failed active association test did not create the expected independent track.');
end

function test_radial_only_does_not_request_degradation()
cfg = range_quality_cfg(); platform = static_platform();
xyz = ae_to_xyz(15, 3, 50000);
est = run_filter_joint_2d3d([active_event(1, 0, xyz, 107); ...
    passive_event(2, 1, [15; 3], 107)], platform, cfg);
tr = logical_track_by_id(est, 2, 1); q = tr.quality;
assert(q.radial_sigma_m >= cfg.joint_radial_sigma_drop_m && ...
    q.range_age_s < cfg.joint_range_age_drop_s && ~q.degrade_request, ...
    'Radial uncertainty alone incorrectly requested 3-D degradation.');
end

function test_range_age_only_does_not_request_degradation()
cfg = range_quality_cfg(); platform = static_platform();
cfg.joint_radial_sigma_warn_m = 1e8; cfg.joint_radial_sigma_drop_m = 2e8;
xyz = ae_to_xyz(15, 3, 50000);
est = run_filter_joint_2d3d([active_event(1, 0, xyz, 108); ...
    passive_event(2, 3, [15; 3], 108)], platform, cfg);
tr = logical_track_by_id(est, 2, 1); q = tr.quality;
assert(q.range_age_s >= cfg.joint_range_age_drop_s && ...
    q.radial_sigma_m < cfg.joint_radial_sigma_drop_m && ~q.degrade_request, ...
    'Reliable-range age alone incorrectly requested 3-D degradation.');
end

function test_radial_and_range_age_request_degradation()
cfg = range_quality_cfg(); platform = static_platform();
xyz = ae_to_xyz(15, 3, 50000);
est = run_filter_joint_2d3d([active_event(1, 0, xyz, 109); ...
    passive_event(2, 3, [15; 3], 109)], platform, cfg);
tr = logical_track_by_id(est, 2, 1); q = tr.quality;
assert(q.radial_sigma_m >= cfg.joint_radial_sigma_drop_m && ...
    q.range_age_s >= cfg.joint_range_age_drop_s && q.degrade_request, ...
    'The dual radial-uncertainty/range-age condition did not request degradation.');
assert(strcmp(tr.mode, '2d_shadow') && q.can_takeover2d && ...
    est.N2(2) == 1 && est.N(2) == 0, ...
    'Dual-threshold request did not execute the enabled 2-D takeover.');
end

function test_dual_threshold_without_2d_takeover_enters_hold()
cfg = range_quality_cfg(); platform = static_platform();
cfg.joint_angle95_max_deg = 1e-6;
xyz = ae_to_xyz(15, 3, 50000);
e2 = empty_event(); e2.cycle_id = 2; e2.t_sec = 3; e2.t_start = 3; e2.t_end = 3;
est = run_filter_joint_2d3d([active_event(1, 0, xyz, 110); e2], platform, cfg);
tr = logical_track_by_id(est, 2, 1);
assert(tr.quality.degrade_request && ~tr.quality.can_takeover2d && ...
    strcmp(tr.mode, 'hold') && est.N_total(2) == 0, ...
    'A degradation request without a qualified 2-D branch did not enter hold.');
assert(any(strcmp({est.transition_log.reason}, '3d_drop_blocked_no_2d_takeover')), ...
    'Blocked 2-D takeover was not recorded in the transition log.');
end

function test_metrics_and_plots()
cfg = test_cfg(); platform = static_platform();
events = repmat(empty_event(), 7, 1);
for k = 1:7
    e = passive_event(k, k - 1, [30 + 0.1*k; 5], 61);
    e.passive.ang = [30 + 0.1*k, 50 - 0.05*k; 5, 8];
    e.passive.R_ae = repmat(diag([0.05^2, 0.04^2]), 1, 1, 2);
    e.passive.ids = [61, 62]; e.passive.src = [1, 1]; e.passive.n_meas = 2;
    events(k) = e;
end
est = run_filter_joint_2d3d(events, platform, cfg);
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.two_d.angle.n > 0 && isfinite(m.two_d.angle.rmse_az_deg), ...
    'Pure 2-D angle metrics are invalid.');
assert(m.three_d.output.n_outputs == 0 && m.three_d.position.n == 0, ...
    'Pure 2-D input incorrectly produced 3-D evaluation samples.');
assert(m.overall.angle.n == m.two_d.angle.n, ...
    'Overall angle metrics do not match the pure 2-D subset.');
assert(m.two_d.accuracy.n_labeled_assoc == m.overall.accuracy.n_labeled_assoc, ...
    'Tentative associations were not backtracked into the final 2-D track metrics.');

old_vis = get(groot, 'DefaultFigureVisible');
cleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', old_vis)); %#ok<NASGU>
set(groot, 'DefaultFigureVisible', 'off'); close all;
plot_joint_tracking_results(est, events, cfg);
assert(numel(findall(groot, 'Type', 'figure')) >= 3, 'Joint overview plots were not created.');
close all;
selected = [est.output{end}.id];
info = view_joint_tracks(est, events, struct('track_ids', selected, ...
    'show_reference', true, 'show_angle', true, 'show_enu', true));
assert(numel(info.selected_ids) == 2, 'Joint track viewer did not retain multiple selected tracks.');
close all;
end

function test_joint_metric_dimension_separation()
cfg = test_cfg();
cfg.track_accuracy_min_assoc = 1;
cfg.track_accuracy_purity_th = 0.9;
events = repmat(empty_event(), 4, 1);
events(1) = passive_event(1, 0, [10; 2], 71);
events(2) = passive_event(2, 1, [11; 2.2], 71);
xyz3 = ae_to_xyz(12, 2.4, 50000);
xyz4 = ae_to_xyz(13, 2.6, 50500);
events(3) = active_event(3, 2, xyz3, 71);
events(4) = active_event(4, 3, xyz4, 71);

est = struct();
est.output = cell(4, 1);
est.assoc = cell(4, 1);
est.transition_log = [];
for k = 1:4
    if k <= 2
        ae = events(k).passive.ang(:, 1);
        dim = 2;
        pos = nan(3, 1);
        type = 'passive';
        xyz = nan(3, 1);
    else
        ae = events(k).active.rae(2:3, 1);
        dim = 3;
        pos = events(k).active.xyz(:, 1);
        type = 'active';
        xyz = pos;
    end
    est.output{k} = struct('id', 1, 'truth_id', 71, 'output_dim', dim, ...
        't_sec', k - 1, 'az_deg', ae(1), 'el_deg', ae(2), ...
        'position_enu', pos);
    est.assoc{k} = struct('id', 1, 'type', {{type}}, 'meas_index', 1, ...
        'tid', 71, 'ang', ae, 'xyz', xyz, 'cost', 0);
end

m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.two_d.angle.n == 2 && m.three_d.angle.n == 2 && m.overall.angle.n == 4, ...
    'Angle evaluation samples were not separated by output dimension.');
assert(m.three_d.position.n == 2 && m.three_d.position.rmse_3d_m < 1e-10, ...
    'Formal 3-D position RMSE is missing or incorrect.');
assert(m.two_d.angle.rmse_los_deg < 1e-10 && m.three_d.angle.rmse_los_deg < 1e-10, ...
    'Dimension-specific angle RMSE is incorrect.');
assert(abs(m.two_d.association.rate_all_tracks - 1) < 1e-12 && ...
       abs(m.three_d.association.rate_all_tracks - 1) < 1e-12 && ...
       abs(m.overall.association.rate_all_tracks - 1) < 1e-12, ...
    'Dimension-specific or overall association rate is incorrect.');
assert(abs(m.two_d.accuracy.accuracy - 1) < 1e-12 && ...
       abs(m.three_d.accuracy.accuracy - 1) < 1e-12 && ...
       abs(m.overall.accuracy.accuracy - 1) < 1e-12, ...
    'Dimension-specific or overall association consistency is incorrect.');
assert(m.two_d.start_time.mean_track_start_delay_s == 0 && ...
       m.three_d.start_time.mean_track_start_delay_s == 0 && ...
       m.overall.start_time.mean_track_start_delay_s == 0, ...
    'Dimension-specific or overall track start delay is incorrect.');
end

function test_birth_measurements_are_counted()
cfg = test_cfg(); platform = static_platform();
cfg.metrics_max_print = 0;
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;

ep = passive_event(1, 0, [12; 3], 81);
estp = run_filter_joint_2d3d(ep, platform, cfg);
mp = evaluate_joint_tracking_metrics(estp, ep, cfg);
assert(mp.two_d.association.n_measurements == 1 && ...
       mp.two_d.association.n_assigned == 1 && ...
       mp.measurement_accounting.passive.unaccounted == 0 && ...
       mp.measurement_accounting.passive.n_to_2d == 1 && ...
       mp.measurement_accounting.passive.destination_gap == 0, ...
    'A passive birth measurement was omitted from association accounting.');

ea = active_event(1, 0, ae_to_xyz(12, 3, 50000), 82);
esta = run_filter_joint_2d3d(ea, platform, cfg);
ma = evaluate_joint_tracking_metrics(esta, ea, cfg);
assert(ma.three_d.association.n_measurements == 1 && ...
       ma.three_d.association.n_assigned == 1 && ...
       ma.measurement_flow.n_range_updated == 1 && ...
       ma.measurement_flow.n_range_associated_without_update == 0 && ...
       ma.measurement_accounting.active_range.unaccounted == 0 && ...
       ma.measurement_accounting.active_range.n_to_3d == 1 && ...
       ma.measurement_accounting.active_range.destination_gap == 0, ...
    'An active range birth measurement was omitted or marked as not updated.');
end

function test_capacity_deletion_is_accounted()
cfg = test_cfg(); platform = static_platform();
cfg.metrics_max_print = 0;
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.max_tracks = 1;
e = passive_event(1, 0, [10; 2], 501);
e.passive.ang = [10, 40; 2, 6];
e.passive.R_ae = repmat(diag([0.05^2, 0.04^2]), 1, 1, 2);
e.passive.ids = [501, 502]; e.passive.src = [1, 1]; e.passive.n_meas = 2;
est = run_filter_joint_2d3d(e, platform, cfg);
m = evaluate_joint_tracking_metrics(est, e, cfg);
row = m.measurement_accounting.passive;
assert(est.stats.deleted_capacity == 1 && row.n_input == 2 && ...
       row.n_to_2d == 1 && row.n_associated_not_retained == 1 && ...
       row.n_explicitly_suppressed == 0 && row.unaccounted == 0 && ...
       row.destination_gap == 0, ...
    'Capacity-deleted association was hidden or counted as a lost measurement.');
end

function test_formal_output_coverage_is_separate()
cfg = test_cfg(); cfg.metrics_max_print = 0;
cfg.track_accuracy_min_assoc = 1;
events = repmat(empty_event(), 4, 1);
est = struct('output', {cell(4, 1)}, 'assoc', {cell(4, 1)}, ...
    'transition_log', []);
for k = 1:4
    xyz = ae_to_xyz(20 + 0.01*k, 3, 50000);
    events(k) = active_event(k, k - 1, xyz, 601);
    ae = events(k).active.rae(2:3, 1);
    est.assoc{k} = struct('id', 1, 'type', {{'active'}}, ...
        'meas_index', 1, 'tid', 601, 'ang', ae, 'xyz', xyz, 'cost', 0, ...
        'measurement_dim', 3, 'filter_dim', 3, ...
        'range_updated', true);
end
est.output{4} = struct('id', 1, 'truth_id', 601, 'output_dim', 3, ...
    't_sec', 3, 'az_deg', events(4).active.rae(2, 1), ...
    'el_deg', events(4).active.rae(3, 1), ...
    'position_enu', events(4).active.xyz(:, 1));
m = evaluate_joint_tracking_metrics(est, events, cfg);
d = m.track_details(1).three_d;
assert(abs(d.coverage - 1) < 1e-12 && ...
       abs(d.formal_output_coverage - 0.25) < 1e-12 && ...
       abs(m.three_d.output_coverage.rate - 0.25) < 1e-12, ...
    'Association history coverage was still reported as formal-output coverage.');
assert(d.is_correct && m.three_d.track_accuracy.accuracy_vs_output == 1, ...
    'Formal-output coverage was incorrectly imposed on truth-association correctness.');
end

function test_track_correctness_uses_truth_denominator()
cfg = test_cfg(); cfg.metrics_max_print = 0;
cfg.track_accuracy_min_assoc = 1; cfg.track_accuracy_purity_th = 0.9;
for dim = [2, 3]
    events = repmat(empty_event(), 10, 1);
    est = struct('output', {cell(10, 1)}, 'assoc', {cell(10, 1)}, 'transition_log', []);
    for k = 1:10
        ae = [20 + 0.01*k; 3]; xyz = ae_to_xyz(ae(1), ae(2), 50000);
        if dim == 2
            events(k) = passive_event(k, k - 1, ae, 601);
            type = 'passive'; xyz(:) = NaN;
        else
            events(k) = active_event(k, k - 1, xyz, 601);
            type = 'active';
        end
        est.assoc{k} = struct('id', 1, 'type', {{type}}, ...
            'meas_index', 1, 'tid', 601, 'ang', ae, 'xyz', xyz, 'cost', 0, ...
            'measurement_dim', dim, 'filter_dim', dim, 'range_updated', dim == 3);
        est.output{k} = struct('id', 1, 'truth_id', 601, 'output_dim', dim, ...
            't_sec', k - 1, 'az_deg', ae(1), 'el_deg', ae(2), 'position_enu', xyz);
    end
    if dim == 2
        scope_name = 'two_d';
        for k = 1:10, est.measurement_disposition{k}.passive.input_dim = uint8(2); end
    else
        scope_name = 'three_d';
    end
    for n_used = [1, 9, 10]
        partial = est;
        partial.assoc(1:10 - n_used) = {[]};
        partial.output(1:9) = {[]};
        m = evaluate_joint_tracking_metrics(partial, events, cfg);
        for name = {scope_name, 'overall'}
            s = m.(name{1}); d = m.track_details(1).(name{1});
            expected = n_used / 10;
            assert(abs(s.track_accuracy.purity - expected) < 1e-12 && ...
                abs(d.coverage - expected) < 1e-12 && d.truth_total == 10, ...
                'Correctness denominator omitted unused measurements of the matched truth.');
            assert(s.track_accuracy.accuracy_vs_output == (expected >= 0.9) && ...
                d.is_correct == (expected >= 0.9), ...
                'Aggregate and per-track correctness disagree with the truth coverage threshold.');
            assert(d.association_consistency == 1 && d.formal_output_coverage == 0.1);
        end
    end
    % Complete truth split across two tracks must not give either track 100%.
    split = est;
    for k = 6:10
        split.assoc{k}.id = 2; split.output{k}.id = 2;
    end
    m = evaluate_joint_tracking_metrics(split, events, cfg);
    assert(m.(scope_name).track_accuracy.n_correct_tracks == 0 && ...
        m.overall.track_accuracy.n_correct_tracks == 0, ...
        'One-to-one matching accepted a short, uncontaminated fragment as a complete track.');
end
end

function test_measurement_dispositions_without_formal_output()
cfg = test_cfg(); cfg.joint_history_level = 'output';
cfg.joint_confirm_M = 5; cfg.joint_confirm_N = 8;
xyz = ae_to_xyz(20, 3, 50000);
events = repmat(empty_event(), 2, 1);
events(1) = active_event(1, 0, xyz, 701);
events(2) = multi_active_event(2, 0.1, [xyz, xyz], [701, 701]);
for k = 1:2
    p = passive_event(k, events(k).t_sec, [20; 3], 701);
    events(k).has_passive = true; events(k).passive = p.passive;
end
events(2).passive.t_sec = [0.1, 0.1];
events(2).passive.ang = [20, 20; 3, 3];
events(2).passive.R_ae = repmat(diag([0.05^2, 0.04^2]), 1, 1, 2);
events(2).passive.ids = [701, 701]; events(2).passive.src = [1, 1];
events(2).passive.n_meas = 2;
est = run_filter_joint_2d3d(events, static_platform(), cfg);
assert(all(est.N_total == 0), 'Fixture must remain unconfirmed.');
assert(est.measurement_disposition{1}.active.action == 2 && ...
    est.measurement_disposition{1}.passive.action == 1);
assert(isequal(sort(est.measurement_disposition{2}.active.action), uint8([1, 2])) && ...
    all(est.measurement_disposition{2}.passive.action == 1), ...
    'Same-source extra returns lost their association/birth disposition.');
assert(est.stats.active_unaccounted == 0 && est.stats.passive_unaccounted == 0, ...
    'Unconfirmed tracks were incorrectly treated as unconsumed measurements.');

cfg.max_tracks = 1;
e = passive_event(1, 0, [10; 2], 801);
e.passive.ang = [10, 40; 2, 6]; e.passive.t_sec = [0, 0];
e.passive.R_ae = repmat(diag([0.05^2, 0.04^2]), 1, 1, 2);
e.passive.ids = [801, 802]; e.passive.src = [1, 1]; e.passive.n_meas = 2;
est = run_filter_joint_2d3d(e, static_platform(), cfg);
d = est.measurement_disposition{1}.passive;
assert(all(d.action == 2) && all(d.track_id > 0) && nnz(d.filter_dim == 0) == 1, ...
    'Capacity deletion removed the birth measurement disposition.');

% Exercise reconciliation of an explicit suppression independently of the
% association policy, which normally retains extra same-source returns.
a = struct('type', {{'active'}}, 'meas_index', 1, 'id', 7, ...
    'filter_dim', 3, 'range_updated', true);
d = joint_measurement_disposition(2, a, 'active', [false, true], 1);
assert(isequal(d.action, uint8([1, 3])) && isequal(d.track_id, [7, 0]));
for suppressed = {[false, false], [true, true]}
    caught = false;
    try
        joint_measurement_disposition(2, a, 'active', suppressed{1}, 1);
    catch ME
        caught = strcmp(ME.identifier, 'run_filter_joint_2d3d:MeasurementDispositionMismatch');
    end
    assert(caught, 'Missing or double-counted measurement disposition was not rejected.');
end
end

function test_joint_metric_cross_dimension_flow()
cfg = test_cfg(); cfg.metrics_max_print = 0;
event = active_event(1, 0, ae_to_xyz(18, 3, 50000), 83);
ae = event.active.rae(2:3, 1);
est = struct();
est.output = {struct('id', 1, 'truth_id', 83, ...
    'output_dim', 2, 't_sec', 0, 'az_deg', ae(1), 'el_deg', ae(2), ...
    'position_enu', nan(3, 1))};
est.assoc = {struct('id', 1, 'type', {{'active'}}, 'meas_index', 1, ...
    'tid', 83, 'ang', ae, 'xyz', event.active.xyz, 'cost', 0, ...
    'measurement_dim', 3, 'range_updated', true)};
est.transition_log = [];
m = evaluate_joint_tracking_metrics(est, event, cfg);
assert(m.three_d.association.n_assigned == 1 && ...
       m.three_d.accuracy.n_labeled_assoc == 0 && ...
       m.measurement_flow.counts(2, 2) == 1, ...
    'A 3-D measurement associated to a 2-D-mode track was hidden by scope statistics.');
assert(m.two_d.accuracy.n_labeled_assoc == 0 && ...
       m.two_d.output.n_outputs == 1 && m.overall.angle.n == 1 && ...
       m.two_d.output_coverage.n_reference_measurements == 0 && ...
       m.three_d.output_coverage.n_reference_measurements == 1, ...
    'RAE entering a 2-D-mode track was incorrectly counted as passive AE.');
end

function test_start_delay_not_backdated_before_correct_association()
cfg = test_cfg(); cfg.metrics_max_print = 0; cfg.track_accuracy_min_assoc = 1;
events = repmat(empty_event(), 5, 1);
est = struct('output', {cell(5, 1)}, 'assoc', {cell(5, 1)}, ...
    'transition_log', []);
p1 = ae_to_xyz(10, 2, 50000); p2 = ae_to_xyz(30, 4, 60000);
for k = 1:5
    events(k) = passive_event(k, k - 1, xyz_to_ae(p1), 1);
    events(k).passive.ang = [xyz_to_ae(p1), xyz_to_ae(p2)];
    events(k).passive.ids = [1, 2]; events(k).passive.n_meas = 2;
    events(k).passive.t_sec = [k-1, k-1];
    est.measurement_disposition{k}.passive.input_dim = uint8([2, 2]);
    mi = 1 + (k >= 3);
    ae = events(k).passive.ang(:, mi);
    est.assoc{k} = struct('id', 10, 'type', {{'passive'}}, ...
        'meas_index', mi, 'tid', mi, 'ang', ae, ...
        'xyz', nan(3, 1), 'cost', 0, ...
        'measurement_dim', 2, 'filter_dim', 2, 'range_updated', false);
    est.output{k} = struct('id', 10, 'truth_id', mi, 'output_dim', 2, ...
        't_sec', k - 1, 'az_deg', ae(1), 'el_deg', ae(2), ...
        'position_enu', nan(3, 1));
end
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.two_d.start_time.n_main_confirmed == 1 && ...
       abs(m.two_d.start_time.mean_main_track_start_delay_s - 2) < 1e-12, ...
    'Track start was backdated before its first correct truth association.');
end

function test_joint_truth_target_count()
cfg = test_cfg();
cfg.metrics_max_print = 0;
cfg.truth_id_split_enabled = true;
cfg.truth_id_split_dist_m = 15000;
cfg.truth_id_split_max_gap_s = 60;
cfg.truth_assoc_map_max_dist_m = inf;

positions = {[0; 50000; 0], [100; 50000; 0], ...
    [40000; 50000; 0], [40100; 50000; 0]};
events = repmat(empty_event(), 4, 1);
for k = 1:4
    events(k) = active_event(k, k - 1, positions{k}, 91);
end
events(1).has_passive = true;
events(1).passive.t_sec = 0;
events(1).passive.ang = [20; 3];
events(1).passive.R_ae = diag([0.05^2, 0.04^2]);
events(1).passive.ids = 92;
events(1).passive.src = 1;
events(1).passive.n_meas = 1;

est = struct('output', {cell(4, 1)}, 'assoc', {cell(4, 1)}, ...
    'transition_log', []);
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.truth_targets.raw_id_count == 2, ...
    'Raw truth target count must include active and passive IDs.');
assert(m.truth_targets.instance_count == 3 && ...
       m.truth_targets.n_split_raw_ids == 1, ...
    'Spatially discontinuous truth ID was not split into two instances.');
assert(m.truth_targets.n_spatial_raw_ids == 1 && ...
       m.truth_targets.n_angle_only_raw_ids == 1, ...
    'Angle-only truth target count is incorrect.');
assert(isequal(m.id_split, m.truth_targets), ...
    'Joint truth-count compatibility field is inconsistent.');
assert(m.overall.output_coverage.n_reference_measurements == 5 && ...
       m.overall.output_coverage.n_covered_measurements == 0 && ...
       m.overall.output_coverage.rate == 0, ...
    'Truth targets without any formal output were excluded from output coverage.');

cfg.metrics_max_print = 1;
report_text = evalc('evaluate_joint_tracking_metrics(est, events, cfg);');
assert(contains(report_text, '航迹级正确率(比输出/比参考)') && ...
       contains(report_text, '正确航迹数=') && ...
       ~contains(report_text, '航迹身份纯度合格率'), ...
    'Joint report must preserve the track-level accuracy metric name.');
assert(contains(report_text, ...
    '量测标签参考数: 原始编号=2, 拆分后实例=3, 被拆分原始编号=1'), ...
    'Joint metric report no longer prints raw and split truth target counts.');

cfg.truth_id_split_enabled = false;
cfg.metrics_max_print = 0;
m_no_split = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m_no_split.truth_targets.raw_id_count == 2 && ...
       m_no_split.truth_targets.instance_count == 2 && ...
       m_no_split.truth_targets.n_split_raw_ids == 0, ...
    'Disabling truth-ID splitting did not restore raw target count.');
end

function test_split_truth_labels_drive_metrics_and_viewer()
cfg = test_cfg(); cfg.metrics_max_print = 0;
cfg.track_accuracy_min_assoc = 1;
cfg.truth_id_split_enabled = true;
cfg.truth_id_split_dist_m = 15000;
cfg.truth_id_split_max_gap_s = 60;
positions = {[0; 50000; 0], [100; 50000; 0], ...
    [40000; 50000; 0], [40100; 50000; 0]};
events = repmat(empty_event(), 4, 1);
est = struct('output', {cell(4, 1)}, 'assoc', {cell(4, 1)}, ...
    'transition_log', []);
for k = 1:4
    events(k) = active_event(k, k - 1, positions{k}, 91);
    id = 1 + (k > 2);
    ae = events(k).active.rae(2:3, 1);
    est.assoc{k} = struct('id', id, 'type', {{'active'}}, ...
        'meas_index', 1, 'tid', 91, 'ang', ae, 'xyz', positions{k}, ...
        'cost', 0, 'measurement_dim', 3, 'filter_dim', 3, ...
        'range_updated', true);
    est.output{k} = struct('id', id, 'truth_id', 91, 'output_dim', 3, ...
        't_sec', k - 1, 'az_deg', ae(1), 'el_deg', ae(2), ...
        'position_enu', positions{k});
end

m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.three_d.accuracy.n_truth == 2 && ...
       m.three_d.accuracy.n_correct == 4 && ...
       m.three_d.accuracy.mixed_track_count == 0 && ...
       m.three_d.accuracy.fragmented_truth_count == 0, ...
    'Split truth instances were not used by association metrics.');
info = view_joint_tracks(est, events, struct('track_ids', [1, 2], ...
    'show_reference', false, 'show_angle', false, 'show_enu', false, ...
    'cfg', cfg));
assert(all(isfinite(info.matched_truth_3d)) && ...
       numel(unique(info.matched_truth_3d)) == 2, ...
    'Selected-track viewer rejoined distinct split truth instances.');
end

function test_plot_mode_separation()
cfg = test_cfg();
cfg.joint_plot_min_life = 1;
dims = [2, 2, 3, 3, 2, 3];
K = numel(dims);
est = struct();
est.output = cell(K, 1);
est.filter_times = (0:K-1).';
est.N2 = (dims == 2).';
est.N = (dims == 3).';
est.N_total = ones(K, 1);
for k = 1:K
    est.output{k} = struct('id', 77, 't_sec', k - 1, ...
        'az_deg', 10 + k, 'el_deg', 2 + 0.1*k, 'output_dim', dims(k), ...
        'position_enu', [100*k; 200*k; 10*k], 'truth_id', NaN);
end
events = repmat(empty_event(), K, 1);
for k = 1:K
    events(k).cycle_id = k;
    events(k).t_sec = k - 1;
    events(k).t_start = k - 1;
    events(k).t_end = k - 1;
end

old_vis = get(groot, 'DefaultFigureVisible');
cleanup_vis = onCleanup(@() set(groot, 'DefaultFigureVisible', old_vis));
cleanup_fig = onCleanup(@() close('all'));
set(groot, 'DefaultFigureVisible', 'off'); close all;

plot_joint_tracking_results(est, events, cfg);
fig2 = findall(groot, 'Type', 'figure', 'Name', '二维纯角度滤波航迹总体图');
fig3 = findall(groot, 'Type', 'figure', 'Name', '三维主动空间滤波航迹');
assert(numel(fig2) == 1 && numel(fig3) == 1, ...
    'Mixed-mode overview did not create separate 2-D and 3-D figures.');
line2 = findobj(fig2, 'Type', 'line', 'DisplayName', 'Track 77');
line3 = findobj(fig3, 'Type', 'line', 'DisplayName', 'Track 77');
assert(numel(line2) == 1 && numel(line3) == 1, ...
    'Mixed-mode overview did not retain the logical track in both mode figures.');
x2 = get(line2, 'XData'); x3 = get(line3, 'XData');
assert(numel(x2) == K && all(isfinite(x2(dims == 2))) && all(isnan(x2(dims == 3))), ...
    'The 2-D overview contains formal 3-D samples or lost formal 2-D samples.');
assert(numel(x3) == K && all(isfinite(x3(dims == 3))) && all(isnan(x3(dims == 2))), ...
    'The 3-D overview contains formal 2-D samples or lost formal 3-D samples.');

close all;
info = view_joint_tracks(est, events, struct('track_ids', 77, ...
    'show_reference', false, 'show_angle', true, 'show_enu', true));
assert(isequal(info.angle_ids, 77) && isequal(info.enu_ids, 77), ...
    'The selected mixed-mode track was not classified into both dimensional views.');
view2 = findall(groot, 'Type', 'figure', 'Name', '选中二维纯角度航迹 AE');
view3 = findall(groot, 'Type', 'figure', 'Name', '选中三维主动航迹 ENU');
line2 = findobj(view2, 'Type', 'line', 'DisplayName', 'Track 77');
line3 = findobj(view3, 'Type', 'line', 'DisplayName', 'Track 77');
x2 = get(line2, 'XData'); x3 = get(line3, 'XData');
assert(all(isnan(x2(dims == 3))) && all(isnan(x3(dims == 2))), ...
    'The selected-track viewer connected samples across dimensional modes.');
close all;
end

function test_pure_passive_wrap()
cfg = test_cfg(); platform = static_platform();
t = 0:0.5:4;
az = [179.0, 179.4, 179.8, -179.8, -179.4, -179.0, -178.6, -178.2, -177.8];
events = repmat(empty_event(), numel(t), 1);
for k = 1:numel(t)
    events(k) = passive_event(k, t(k), [az(k); 3 + 0.05*k], 11);
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(any(est.N2 >= 1), 'Pure passive input did not confirm a 2-D track.');
ids = collect_ids(est, 2);
assert(numel(unique(ids)) == 1, 'Azimuth wrap created a false second logical ID.');
assert(all(isfinite(est.X2{end}(:))), '2-D output contains non-finite state values.');
assert(isempty(est.X{end}) || size(est.X{end}, 2) == 0, ...
    'Pure passive input must not create a formal 3-D output.');
end

function test_pure_active_3d()
cfg = test_cfg(); platform = static_platform();
t = 0:0.5:3;
events = repmat(empty_event(), numel(t), 1);
for k = 1:numel(t)
    xyz = [10000 + 100*k; 50000 + 40*k; 2000];
    events(k) = active_event(k, t(k), xyz, 21);
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(any(est.N >= 1), 'Pure active input did not confirm a 3-D track.');
assert(size(est.X{end}, 1) == 9, 'Legacy 3-D state contract must remain 9-D.');
end

function test_2d_to_3d_same_id()
cfg = test_cfg(); platform = static_platform();
events = repmat(empty_event(), 10, 1);
for k = 1:5
    events(k) = passive_event(k, k - 1, [10 + 0.05*k; 2], 31);
end
for k = 6:10
    xyz = ae_to_xyz(10 + 0.05*k, 2, 50000 + 100*k);
    events(k) = active_event(k, k - 1, xyz, 31);
end
est = run_filter_joint_2d3d(events, platform, cfg);
id2 = first_output_id(est, 2);
id3 = first_output_id(est, 3);
assert(isfinite(id2) && isfinite(id3), 'Expected both 2-D and 3-D formal output stages.');
assert(id2 == id3, 'Dimension upgrade changed the logical track ID.');
assert(any(strcmp({est.transition_log.to}, '3d')), 'Missing formal 2-D to 3-D transition.');
end

function test_shard_event_dedup()
cfg = test_cfg();
f = empty_frame();
f.t_sec = 1;
f.passive_ang = [12, 12.0002; 3, 3.0002];
f.passive_src = [1, 2]; f.passive_ids = [8, 8]; f.passive_t = [1, 1];
[events, info] = build_joint_measurement_events(f, cfg);
assert(numel(events) == 1, 'Same passive scan was split into multiple events.');
assert(events(1).passive.n_meas == 1, 'Cross-shard duplicate was not condensed.');
assert(info.n_passive_duplicates == 1, 'Duplicate accounting is inconsistent.');
assert(events(1).passive.ang(1) > 12 && events(1).passive.ang(1) < 12.0002, ...
    'Cross-shard condensation kept one file instead of fusing the records.');
end

function test_target_id_does_not_control_dedup()
cfg = test_cfg();
f = empty_frame(); f.t_sec = 1;
f.passive_ang = [12, 12.0002; 3, 3.0002];
f.passive_src = [1, 2]; f.passive_t = [1, 1];
f.passive_ids = [8, 99];
[different_id, ~] = build_joint_measurement_events(f, cfg);
f.passive_ids = [8, 8];
[same_id, ~] = build_joint_measurement_events(f, cfg);
assert(different_id(1).passive.n_meas == same_id(1).passive.n_meas && ...
    different_id(1).passive.n_meas == 1, ...
    'Changing target_id changed geometric retransmission de-duplication.');

f.passive_ang(1, 2) = 12.005;
[not_exact, ~] = build_joint_measurement_events(f, cfg);
assert(not_exact(1).passive.n_meas == 2, ...
    'A shared target_id incorrectly condensed distinct source observations.');

cfg.condense_enable = true; cfg.condense_method = 'resolution';
a = struct('t_sec', [0; 0], 'az_deg', [10; 10], 'el_deg', [2; 2], ...
    'range_m', [50000; 50000], 'range_valid', [true; true], ...
    'target_id', [8; 99], 'source', 'active.txt', 'n_meas', 2);
different_id_frames = cohere_measurements({a}, {}, static_platform(), cfg);
a.target_id = [8; 8];
same_id_frames = cohere_measurements({a}, {}, static_platform(), cfg);
n_different = sum(arrayfun(@(x) size(x.active_xyz, 2), different_id_frames));
n_same = sum(arrayfun(@(x) size(x.active_xyz, 2), same_id_frames));
assert(n_different == n_same && n_same == 1, ...
    'Changing target_id changed optional measurement condensation.');
end

function test_independent_sources_share_logical_track()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_merge_angle_deg = 0;
events = repmat(empty_event(), 5, 1);
base = [10000; 50000; 1000];
for k = 1:numel(events)
    t = 0.1 * (k - 1);
    events(k) = two_source_active_event(k, t, base + [k; 0; 0]);
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(numel(est.logical_tracks{end}) == 1 && est.N(end) == 1, ...
    'Independent sources created duplicate logical tracks for one target.');
assert(est.stats.active_births == 1 && est.stats.active_assigned == 9, ...
    'Source-wise association did not route all observations to one logical track.');
assert(numel(unique(est.assoc{end}.src)) == 2 && ...
    numel(unique(est.assoc{end}.id)) == 1, ...
    'Per-source association provenance or logical identity was lost.');
end

function test_multisource_cardinality_at_scale()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_merge_angle_deg = 0;
n_targets = 8; n_sources = 3; n_events = 5;
events = repmat(empty_event(), n_events, 1);
base = [1000 * (1:n_targets); 40000 + 3000 * (1:n_targets); ...
    500 + 50 * (1:n_targets)];
for k = 1:n_events
    positions = base + [20 * (k - 1) * ones(1, n_targets); ...
        zeros(2, n_targets)];
    events(k) = multisource_active_event(k, 0.1 * (k - 1), positions, n_sources);
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(numel(est.logical_tracks{end}) == n_targets && est.N(end) == n_targets, ...
    'Multi-source association did not preserve target cardinality at scale.');
assert(est.stats.active_births == n_targets && ...
    est.stats.active_assigned == n_targets * n_sources * n_events - n_targets, ...
    'Scaled multi-source measurement routing is not conservative.');
end

function test_birth_covariance_preserves_cross_terms()
cfg = test_cfg(); platform = static_platform();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
R = [10000, 3500, -1200; 3500, 14400, 2100; -1200, 2100, 8100];
e = active_event(1, 0, [10000; 50000; 1000], 1);
e.active.R_xyz = R;
est = run_filter_joint_2d3d(e, platform, cfg);
P = est.logical_tracks{1}(1).cov3d([1, 4, 7], [1, 4, 7]);
assert(norm(P - R, 'fro') < 1e-6, ...
    'Spatial birth discarded ENU covariance cross terms.');

p = passive_event(1, 0, [10; 2], 1);
p.passive.R_ae = [0.04, 0.012; 0.012, 0.025];
est = run_filter_joint_2d3d(p, platform, cfg);
P = est.logical_tracks{1}(1).angle_cov([1, 4], [1, 4]);
assert(abs(P(1, 2) - 0.012) < 1e-9, ...
    'Angle birth discarded azimuth/elevation covariance cross terms.');
end

function test_output_history_level_contract()
cfg = test_cfg(); platform = static_platform();
cfg.joint_history_level = 'output';
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
events = repmat(active_event(1, 0, [10000; 50000; 1000], 1), 3, 1);
for k = 1:3
    events(k).cycle_id = k; events(k).t_sec = k - 1;
    events(k).t_start = k - 1; events(k).t_end = k - 1;
    events(k).active.t_sec = k - 1;
end
est = run_filter_joint_2d3d(events, platform, cfg);
assert(strcmp(est.history_level, 'output') && all(cellfun(@isempty, est.logical_tracks)) && ...
    all(cellfun(@isempty, est.quality_history)) && all(cellfun(@isempty, est.tracks)), ...
    'Output history level retained expensive all-track snapshots.');
assert(~isempty(est.output{end}) && ~isempty(est.X{end}) && ...
    all(est.assoc{end}.filter_dim == 3), ...
    'Output history level broke formal output or association contracts.');
end

function e = two_source_active_event(k, t, xyz)
e = active_event(k, t, xyz, 17);
xyz2 = xyz + [8; -5; 3];
ae2 = xyz_to_ae(xyz2);
e.active.t_sec = [t, t];
e.active.xyz = [xyz, xyz2];
e.active.rae = [e.active.rae, [norm(xyz2); ae2]];
e.active.R_xyz = repmat(diag([100^2, 100^2, 100^2]), 1, 1, 2);
e.active.R_ae = repmat(diag([0.08^2, 0.06^2]), 1, 1, 2);
e.active.has_range = [true, true]; e.active.ids = [17, 17];
e.active.src = [1, 2]; e.active.n_meas = 2;
end

function e = multisource_active_event(k, t, positions, n_sources)
n_targets = size(positions, 2); n = n_targets * n_sources;
e = empty_event(); e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
e.has_active = true; e.active.t_sec = t * ones(1, n);
e.active.xyz = zeros(3, n); e.active.rae = zeros(3, n);
e.active.R_xyz = repmat(diag([100^2, 100^2, 100^2]), 1, 1, n);
e.active.R_ae = repmat(diag([0.08^2, 0.06^2]), 1, 1, n);
e.active.has_range = true(1, n); e.active.ids = zeros(1, n);
e.active.src = zeros(1, n); p = 0;
offsets = [0, 5, -4; 0, -3, 2; 0, 2, -1];
for source = 1:n_sources
    ii = p + (1:n_targets); xyz = positions + offsets(:, source);
    e.active.xyz(:, ii) = xyz;
    e.active.rae(:, ii) = [sqrt(sum(xyz.^2, 1)); ...
        atan2d(xyz(1, :), xyz(2, :)); ...
        atan2d(xyz(3, :), hypot(xyz(1, :), xyz(2, :)))];
    e.active.ids(ii) = 1:n_targets; e.active.src(ii) = source;
    p = p + n_targets;
end
e.active.n_meas = n;
end

function test_scan_pairing_preserves_causal_time()
cfg = test_cfg();
f = empty_frame();
xyz1 = ae_to_xyz(10, 2, 40000); xyz2 = ae_to_xyz(20, 3, 45000);
f.active_xyz = [xyz1, xyz2];
f.active_rae = [[norm(xyz1); xyz_to_ae(xyz1)], [norm(xyz2); xyz_to_ae(xyz2)]];
f.active_R = repmat(diag([100^2, 100^2, 100^2]), 1, 1, 2);
f.active_src = [1, 1]; f.target_ids = [1, 2]; f.active_t = [0, 0.016];
f.passive_ang = xyz_to_ae(xyz2); f.passive_src = 1;
f.passive_ids = 2; f.passive_t = 0.009;
[events, info] = build_joint_measurement_events(f, cfg);
assert(numel(events) == 2 && info.n_events == 2, 'Ambiguous scans changed the event count.');
assert(events(1).has_active && events(1).has_passive && ...
    abs(events(1).active.t_sec - 0) < 1e-12, ...
    'The passive scan was not paired with the preceding causal active scan.');
assert(events(2).has_active && ~events(2).has_passive && ...
    abs(events(2).active.t_sec - 0.016) < 1e-12, ...
    'An active scan later than the passive timestamp was paired out of order.');
end

function cfg = test_cfg()
cfg = config_fusion();
cfg.processing_framework = 'joint_2d3d';
cfg.joint_history_level = 'full';
cfg.joint_confirm_M = 3; cfg.joint_confirm_N = 5;
cfg.joint_3d_birth_M = 3; cfg.joint_3d_birth_N = 5;
cfg.joint_3d_upgrade_M = 2; cfg.joint_3d_upgrade_N = 3;
cfg.joint_pos95_recover_m = 20000;
cfg.joint_pos95_warn_m = 40000;
cfg.joint_pos95_down_m = 80000;
cfg.joint_radial95_recover_m = 15000;
cfg.joint_radial95_warn_m = 30000;
cfg.joint_radial95_down_m = 60000;
cfg.joint_angle95_max_deg = 2;
cfg.joint_merge_angle_deg = 0.01;
cfg.joint_shard_duplicate_angle_deg = 0.02;
cfg.truth_cross_sensor_id_consistent = true;
cfg.max_tracks = 50;
end

function cfg = range_quality_cfg()
cfg = test_cfg();
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_3d_upgrade_M = 2; cfg.joint_3d_upgrade_N = 3;
cfg.joint_down_consecutive = 1;
cfg.joint_confirmed_timeout_s = 1000;
cfg.joint_shadow_max_s = 100;
cfg.joint_angle95_max_deg = 10;
cfg.joint_radial_sigma_warn_m = 150;
cfg.joint_radial_sigma_drop_m = 250;
cfg.joint_range_age_warn_s = 1;
cfg.joint_range_age_drop_s = 2;
end

function tr = logical_track_by_id(est, k, id)
tracks = est.logical_tracks{k};
j = find([tracks.id] == id, 1);
assert(~isempty(j), 'Expected logical track ID %d at event %d.', id, k);
tr = tracks(j);
end

function p = static_platform()
p = struct();
p.n_rows = 2; p.t_sec = [0; 100];
p.lat_deg = [0; 0]; p.lon_deg = [0; 0]; p.alt_m = [0; 0];
p.interp_lat = @(t) zeros(size(t));
p.interp_lon = @(t) zeros(size(t));
p.interp_alt = @(t) zeros(size(t));
end

function e = empty_event()
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'has_active', false, 'has_passive', false, ...
    'active', active_block(), 'passive', passive_block());
end

function a = active_block()
a = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'R_xyz', zeros(3, 3, 0), 'R_ae', zeros(2, 2, 0), ...
    'has_range', false(1, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
end

function p = passive_block()
p = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), ...
    'src', zeros(1, 0), 'n_meas', 0);
end

function e = passive_event(k, t, ae, tid)
e = empty_event(); e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
e.has_passive = true; e.passive.t_sec = t; e.passive.ang = ae;
e.passive.R_ae = diag([0.05^2, 0.04^2]); e.passive.ids = tid;
e.passive.src = 1; e.passive.n_meas = 1;
end

function e = active_event(k, t, xyz, tid)
e = empty_event(); e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
ae = xyz_to_ae(xyz); r = norm(xyz);
e.has_active = true; e.active.t_sec = t; e.active.xyz = xyz;
e.active.rae = [r; ae]; e.active.R_xyz = diag([100^2, 100^2, 100^2]);
e.active.R_ae = diag([0.08^2, 0.06^2]); e.active.has_range = true;
e.active.ids = tid; e.active.src = 1; e.active.n_meas = 1;
end

function e = multi_active_event(k, t, xyz, tids)
n = size(xyz, 2);
e = empty_event(); e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
e.has_active = true; e.active.t_sec = repmat(t, 1, n); e.active.xyz = xyz;
e.active.rae = [sqrt(sum(xyz.^2, 1)); ...
    atan2d(xyz(1, :), xyz(2, :)); ...
    atan2d(xyz(3, :), hypot(xyz(1, :), xyz(2, :)))];
e.active.R_xyz = repmat(diag([100^2, 100^2, 100^2]), 1, 1, n);
e.active.R_ae = repmat(diag([0.08^2, 0.06^2]), 1, 1, n);
e.active.has_range = true(1, n); e.active.ids = tids(:).';
e.active.src = ones(1, n); e.active.n_meas = n;
end

function f = empty_frame()
f = struct('t_sec', 0, 'active_xyz', zeros(3, 0), 'active_rae', zeros(3, 0), ...
    'active_R', zeros(3, 3, 0), 'active_src', zeros(1, 0), ...
    'target_ids', zeros(1, 0), 'active_t', zeros(1, 0), ...
    'active_ae_only', zeros(2, 0), 'active_ae_only_src', zeros(1, 0), ...
    'active_ae_only_ids', zeros(1, 0), 'active_ae_only_t', zeros(1, 0), ...
    'passive_ang', zeros(2, 0), 'passive_src', zeros(1, 0), ...
    'passive_ids', zeros(1, 0), 'passive_t', zeros(1, 0));
end

function xyz = ae_to_xyz(az, el, r)
xyz = [r*cosd(el)*sind(az); r*cosd(el)*cosd(az); r*sind(el)];
end

function ae = xyz_to_ae(xyz)
ae = [atan2d(xyz(1), xyz(2)); atan2d(xyz(3), hypot(xyz(1), xyz(2)))];
end

function ids = collect_ids(est, dim)
ids = zeros(1, 0);
for k = 1:numel(est.output)
    o = est.output{k};
    if isempty(o), continue; end
    jj = find([o.output_dim] == dim);
    ids = [ids, [o(jj).id]]; %#ok<AGROW>
end
end

function id = first_output_id(est, dim)
id = NaN;
for k = 1:numel(est.output)
    o = est.output{k};
    if isempty(o), continue; end
    j = find([o.output_dim] == dim, 1);
    if ~isempty(j), id = o(j).id; return; end
end
end
