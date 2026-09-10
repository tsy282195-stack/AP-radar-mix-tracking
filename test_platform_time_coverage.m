function test_platform_time_coverage()
%TEST_PLATFORM_TIME_COVERAGE Bounded navigation interpolation regressions.
cfg = config_fusion();
cfg.platform.angle_unit = 'deg'; cfg.platform.time_format = 'seconds';
cfg.read_percent = 100; cfg.read_start_percent = 0; cfg.read_end_percent = 100;
cfg.time_range_s = []; cfg.max_rows = inf;
cfg.platform_max_extrapolation_s = 0.1;
times = [52299.882; 52299.933; 52299.984];
p = fixture_platform(times, cfg);
assert(isequal(p.t_sec, times) && p.n_rows == 3, ...
    'Extrapolation must not add navigation samples or move the coordinate origin.');
assert(abs(p.max_extrapolation_s - 0.051) < 1e-9);

% The reported 16 ms shortfall must use motion, including for vector queries.
query = [times(1) - 0.016, times(1); times(2), 52300];
tau = query - times(1);
assert_close(p.interp_lat(query), 30 + 0.002 * tau);
assert_close(p.interp_lon(query), 120 + 0.001 * tau);
assert_close(p.interp_alt(query), 1000 + 2 * tau);
assert(p.interp_alt(52300) > p.alt_m(end), 'Endpoint position was clamped.');
assert_close(p.interp_alt(times), p.alt_m);
assert_close(p.interp_alt(times(1) + 0.0255), 1000.051);
assert(isequal(size(p.interp_lat(zeros(0, 1))), [0, 1]));
assert(isequal(size(p.interp_lon(zeros(1, 0))), [1, 0]));

active = {struct('t_sec', [times(1); 52300])};
passive = {struct('t_sec', times(1) - 0.016)};
assert_platform_covers_measurements(p, active, passive);
assert_platform_covers_measurements(p, active, {});
assert_platform_covers_measurements(p, {}, passive);
assert_platform_covers_measurements(p, {}, {});
assert_platform_covers_measurements(p, {struct('t_sec', [NaN; inf])}, {});

edge = [times(1) - p.max_extrapolation_s, times(end) + p.max_extrapolation_s];
assert_close(p.interp_alt(edge), 1000 + 2 * (edge - times(1)));
assert_platform_covers_measurements(p, {struct('t_sec', edge)}, {});
for q = [edge(1) - 0.001, edge(2) + 0.001, times(end) + 10]
    expect_error(@() p.interp_lat(q), 'load_platform_txt:PlatformTimeCoverage');
    expect_error(@() p.interp_lon(q), 'load_platform_txt:PlatformTimeCoverage');
    expect_error(@() p.interp_alt(q), 'load_platform_txt:PlatformTimeCoverage');
    expect_error(@() assert_platform_covers_measurements(p, ...
        {struct('t_sec', [times(1), q])}, {}), 'load_platform_txt:PlatformTimeCoverage');
    expect_error(@() assert_platform_covers_measurements(p, {}, ...
        {struct('t_sec', q)}), 'load_platform_txt:PlatformTimeCoverage');
end
expect_error(@() p.interp_lat(NaN), 'load_platform_txt:InvalidQueryTime');
expect_error(@() p.interp_alt(inf), 'load_platform_txt:InvalidQueryTime');

strict_cfg = cfg; strict_cfg.platform_max_extrapolation_s = 0;
strict = fixture_platform(times, strict_cfg);
assert_close(strict.interp_lat(times), p.lat_deg);
expect_error(@() assert_platform_covers_measurements(strict, active, {}), ...
    'load_platform_txt:PlatformTimeCoverage');
old_cfg = rmfield(cfg, 'platform_max_extrapolation_s');
old = fixture_platform(times, old_cfg);
assert_close(old.interp_alt(52300), p.interp_alt(52300));
slow = fixture_platform([0; 1; 2], cfg);
assert(slow.max_extrapolation_s == 0.1, 'Slow navigation exceeded the configured cap.');
assert_close(slow.interp_alt(2.1), 1004.2);
expect_error(@() slow.interp_alt(2.101), 'load_platform_txt:PlatformTimeCoverage');
single = fixture_platform(times(end), cfg);
assert(single.max_extrapolation_s == 0 && single.interp_alt(times(end)) == 1000);
expect_error(@() single.interp_alt(52300), 'load_platform_txt:PlatformTimeCoverage');
dedup = fixture_platform([0; 0.05; 0.05; 0.1], cfg);
assert(dedup.n_rows == 3 && abs(dedup.max_extrapolation_s - 0.05) < 1e-12);
assert_close(dedup.interp_alt(0.116), 1000.232);

% Exercise the reported endpoint through coordinate conversion and filtering.
cfg.do_plot = false; cfg.metrics_max_print = 0;
cfg.frame_time_window_s = 0.015;
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
t_meas = [52299.94; 52299.96; 52299.98; 52300];
a = struct('t_sec', t_meas, 'az_deg', 20 * ones(4, 1), ...
    'el_deg', 2 * ones(4, 1), 'range_m', 50000 * ones(4, 1), ...
    'range_valid', true(4, 1), 'target_id', ones(4, 1), ...
    'source', 'endpoint_test', 'n_meas', 4);
assert_platform_covers_measurements(p, {a}, {});
frames = cohere_measurements({a}, {}, p, cfg);
[events, ~] = build_joint_measurement_events(frames, cfg);
assert(sum(arrayfun(@(e) e.active.n_meas, events)) == 4, ...
    'Platform endpoint handling discarded measurements.');
est = run_filter_joint_2d3d(events, p, cfg);
assert(est.filter_times(end) == 52300 && est.N(end) == 1);
assert(all(isfinite(est.X{end}(:))) && est.stats.active_range_unaccounted == 0);

validation_cfg = cfg;
validation_cfg.data_dir = fileparts(mfilename('fullpath'));
validation_cfg.active_files = {'config_fusion.m'};
validation_cfg.passive_files = {'config_fusion.m'};
validation_cfg.platform_file = 'config_fusion.m';
validated = validate_config_fusion(rmfield(validation_cfg, 'platform_max_extrapolation_s'));
assert(validated.platform_max_extrapolation_s == 0.1);
validation_cfg.platform_max_extrapolation_s = 0;
validated = validate_config_fusion(validation_cfg);
assert(validated.platform_max_extrapolation_s == 0);
for value = {-1, NaN, inf, [0.01, 0.02], '0', 1i, true}
    bad = validation_cfg; bad.platform_max_extrapolation_s = value{1};
    expect_error(@() validate_config_fusion(bad), '');
    expect_error(@() fixture_platform(times, bad), 'load_platform_txt:InvalidExtrapolationLimit');
end
fprintf('Platform time coverage tests passed.\n');
end

function p = fixture_platform(times, cfg)
path = [tempname, '.csv'];
cleanup = onCleanup(@() delete_if_exists(path)); %#ok<NASGU>
fid = fopen(path, 'w');
assert(fid >= 0, 'Cannot create navigation fixture.');
close_file = onCleanup(@() fclose(fid));
times = times(:); tau = times - times(1);
rows = [times, 30 + 0.002 * tau, 120 + 0.001 * tau, 1000 + 2 * tau];
fprintf(fid, '%.12f,%.12f,%.12f,%.12f\n', rows.');
clear close_file;
p = load_platform_txt(path, cfg);
end

function delete_if_exists(path)
if exist(path, 'file') == 2, delete(path); end
end

function assert_close(actual, expected)
assert(isequal(size(actual), size(expected)), 'Interpolation changed the query shape.');
assert(all(abs(actual(:) - expected(:)) < 1e-8), 'Unexpected interpolated platform position.');
end

function expect_error(fn, identifier)
try
    fn();
catch ME
    assert(isempty(identifier) || strcmp(ME.identifier, identifier), ...
        'Unexpected error: %s', ME.identifier);
    return;
end
error('test_platform_time_coverage:MissingError', 'Expected error was not raised: %s', identifier);
end
