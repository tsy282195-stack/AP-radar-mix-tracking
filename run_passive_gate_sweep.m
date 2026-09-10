function results = run_passive_gate_sweep(read_end_percent, candidates)
%RUN_PASSIVE_GATE_SWEEP Compare current-core passive association/switch gates.

if nargin < 1 || isempty(read_end_percent)
    read_end_percent = 20;
end
if nargin < 2 || isempty(candidates)
    candidates = [ ...
        16.000, 16.000; ...
        16.000, 9.2103; ...
         9.2103, 9.2103; ...
         5.991, 9.2103; ...
         5.991, 5.991; ...
         3.000, 5.991];
end
if size(candidates, 2) ~= 2
    error('candidates must be [joint_gate_2d, joint_switch_gate_2d].');
end

cfg = config_fusion();
cfg.read_start_percent = 0;
cfg.read_end_percent = read_end_percent;
cfg.read_percent = read_end_percent;
cfg.do_plot = false;
cfg.metrics_enabled = true;
cfg.metrics_progress_enabled = false;
cfg.joint_streaming_quiet = true;
cfg = validate_config_fusion(cfg);
cfg.joint_shard_dedup_enabled = false;

fprintf('Preparing shared %.1f%% input slice...\n', read_end_percent);
[active_list, passive_list] = load_radar_measurement_files(cfg);
platform_cfg = cfg;
platform_cfg.read_start_percent = 0;
platform_cfg.read_end_percent = 100;
platform_cfg.read_percent = 100;
platform_cfg.max_rows = inf;
platform = load_platform_txt(resolve_data_path(cfg.data_dir, cfg.platform_file), platform_cfg);
frames = cohere_measurements(active_list, passive_list, platform, cfg);
[joint_events, ~] = build_joint_measurement_events(frames, cfg);

n = size(candidates, 1);
template = struct('bearing_gate_nis', NaN, 'switch_gate_nis', NaN, ...
    'passive_input', 0, ...
    'runtime_s', NaN, 'passive_to_3d', 0, 'passive_to_2d', 0, ...
    'passive_unassigned', 0, 'n2_tracks', 0, 'n3_tracks', 0, ...
    'acc2_output', NaN, 'acc2_truth', NaN, ...
    'acc3_output', NaN, 'acc3_truth', NaN, ...
    'acc_all_output', NaN, 'acc_all_truth', NaN, ...
    'coverage2', NaN, 'coverage3', NaN, 'coverage_all', NaN);
results = repmat(template, n, 1);

for q = 1:n
    run_cfg = cfg;
    run_cfg.joint_gate_2d = candidates(q, 1);
    run_cfg.joint_switch_gate_2d = candidates(q, 2);
    run_cfg = validate_config_fusion(run_cfg);
    fprintf('\n[%d/%d] bearing_gate_nis=%.4g, switch_gate_nis=%.4g\n', ...
        q, n, candidates(q, 1), candidates(q, 2));
    t_run = tic;
    est = run_filter_joint_2d3d(joint_events, platform, run_cfg);
    metrics = evaluate_joint_tracking_metrics(est, joint_events, run_cfg);

    r = template;
    r.bearing_gate_nis = candidates(q, 1);
    r.switch_gate_nis = candidates(q, 2);
    r.runtime_s = toc(t_run);
    pm = metrics.measurement_accounting.passive;
    r.passive_input = pm.n_input;
    r.passive_to_3d = pm.n_to_3d;
    r.passive_to_2d = pm.n_to_2d;
    r.passive_unassigned = pm.n_input - pm.n_to_2d - pm.n_to_3d;
    r.n2_tracks = metrics.two_d.output.n_unique_tracks;
    r.n3_tracks = metrics.three_d.output.n_unique_tracks;
    r.acc2_output = metrics.two_d.track_accuracy.accuracy_vs_output;
    r.acc2_truth = metrics.two_d.track_accuracy.accuracy_vs_truth;
    r.acc3_output = metrics.three_d.track_accuracy.accuracy_vs_output;
    r.acc3_truth = metrics.three_d.track_accuracy.accuracy_vs_truth;
    r.acc_all_output = metrics.overall.track_accuracy.accuracy_vs_output;
    r.acc_all_truth = metrics.overall.track_accuracy.accuracy_vs_truth;
    r.coverage2 = metrics.two_d.output_coverage.rate;
    r.coverage3 = metrics.three_d.output_coverage.rate;
    r.coverage_all = metrics.overall.output_coverage.rate;
    results(q) = r;
    fprintf(['  time=%.1fs, passive 3D/2D/unassigned=%d/%d/%d, tracks 2D/3D=%d/%d, ' ...
        'accuracy truth 2D/3D/all=%.1f/%.1f/%.1f%%\n'], ...
        r.runtime_s, r.passive_to_3d, r.passive_to_2d, r.passive_unassigned, ...
        r.n2_tracks, r.n3_tracks, 100*r.acc2_truth, 100*r.acc3_truth, 100*r.acc_all_truth);
    clear est metrics
end
results = struct2table(results);
disp(results);
end
