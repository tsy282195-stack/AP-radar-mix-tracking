function results = run_fusion_stress_tests(profile, only_case, report_dir, suite_dir)
%RUN_FUSION_STRESS_TESTS Run isolated, reproducible fusion stress jobs.
%   run_fusion_stress_tests('quick')
%   run_fusion_stress_tests('full', 'mixed_seed_003')
%   Use simulation_cases/run_fusion_stress.py for hard per-process timeouts.
if nargin < 1 || isempty(profile), profile = 'quick'; end
if nargin < 2, only_case = ''; end
root = fileparts(mfilename('fullpath'));
if nargin < 4 || isempty(suite_dir)
    suite_dir = fullfile(root,'simulation_cases','fusion_stress_suite');
end
old_path = path; cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(root, fullfile(root,'stress_tests'));
tiers = {'quick','full','torture'};
rank = find(strcmp(profile,tiers),1);
assert(~isempty(rank), 'stress:Profile', 'Profile must be quick, full or torture.');
assert(exist(fullfile(suite_dir,'suite.json'),'file') == 2, ...
    'stress:MissingData', 'Generate the stress suite before running MATLAB tests.');
suite = jsondecode(fileread(fullfile(suite_dir,'suite.json')));
generated_rank = find(strcmp(suite.generated_profile,tiers),1);
assert(~isempty(generated_rank) && rank <= generated_rank, ...
    'stress:ProfileNotGenerated', 'Requested profile has not been generated.');
jobs = suite.cases([suite.cases.tier] <= rank);
if ~isempty(only_case), jobs = jobs(strcmp({jobs.name},only_case)); end
assert(~isempty(jobs), 'stress:NoJobs', 'No matching stress case in the selected profile.');
if nargin < 3 || isempty(report_dir)
    [~,token] = fileparts(tempname);
    report_dir = fullfile(root,'stress_reports',[datestr(now,'yyyymmdd_HHMMSS') '_' token]);
end
if exist(report_dir,'dir') ~= 7, mkdir(report_dir); end
write_text(fullfile(report_dir,'source_hashes.json'), jsonencode(source_hashes(root)));
base_cfg = config_fusion();
save(fullfile(report_dir,'baseline_config.mat'),'base_cfg');
write_text(fullfile(report_dir,'environment.txt'), sprintf('MATLAB %s\nComputer %s\nProfile %s\nSuite %s\n', ...
    version, computer, profile, suite_dir));
results = repmat(result_template(),0,1);
for i = 1:numel(jobs)
    job = jobs(i);
    case_dir = fullfile(report_dir,job.name);
    assert(exist(case_dir,'dir') ~= 7, 'stress:ExistingReport', 'Refusing to overwrite case report: %s',case_dir);
    mkdir(case_dir);
    write_text(fullfile(case_dir,'job.json'),jsonencode(job));
    fprintf('[%d/%d] START %s (seed=%d)\n',i,numel(jobs),job.name,job.seed);
    log_text = evalc('entry = execute_job(job, base_cfg, suite_dir, case_dir);');
    write_text(fullfile(case_dir,'console.txt'),log_text);
    write_text(fullfile(case_dir,'result.json'),jsonencode(entry));
    results(end+1,1) = entry; %#ok<AGROW>
    write_text(fullfile(report_dir,'summary.json'),jsonencode(results));
    save(fullfile(report_dir,'summary.mat'),'results','profile','suite_dir');
    fprintf('  %s %.2fs %s\n',upper(entry.status),entry.elapsed_s,entry.message);
end
summary = struct2table(rmfield(results,'details'),'AsArray',true);
writetable(summary,fullfile(report_dir,'summary.csv'));
failed = nnz(strcmp({results.status},'failed'));
fprintf('Stress run complete: %d passed, %d failed. Reports: %s\n',numel(results)-failed,failed,report_dir);
if failed > 0
    error('stress:SuiteFailed','%d stress jobs failed. Review %s',failed,report_dir);
end
end

function entry = execute_job(job, base_cfg, suite_dir, case_dir)
entry = result_template(); entry.name = job.name; entry.seed = job.seed;
t0 = tic;
try
    switch job.kind
        case 'dataset'
            entry.details = run_dataset(job,base_cfg,suite_dir,case_dir);
        case 'point'
            entry.details = run_fusion_point_stress(job.name,job.seed,case_dir);
        case 'regression'
            allowed = {'test_joint_2d3d_framework','test_joint_real_truth', ...
                'test_track_birth_lifecycle','test_active_assoc_cascade', ...
                'test_passive_async_multimeas','test_quant_metrics_refactor'};
            assert(any(strcmp(job.name,allowed)), 'stress:UnknownRegression','Unrecognized regression entry.');
            feval(job.name);
        otherwise
            error('stress:JobKind','Unknown job kind: %s',job.kind);
    end
    entry.status = 'passed';
catch ME
    entry.status = 'failed'; entry.error_identifier = ME.identifier;
    entry.message = ME.message;
    write_text(fullfile(case_dir,'failure.txt'),getReport(ME,'extended','hyperlinks','off'));
end
entry.elapsed_s = toc(t0);
end

function details = run_dataset(job, cfg, suite_dir, report_dir)
data_dir = fullfile(suite_dir,job.name);
spec = jsondecode(fileread(fullfile(data_dir,'case.json')));
names = fieldnames(spec.overrides);
for i = 1:numel(names), cfg.(names{i}) = spec.overrides.(names{i}); end
cfg.data_dir = fullfile(data_dir,'fusion_wide_input');
cfg.active_files = {'meas_sensor1.txt','meas_sensor2.txt','meas_sensor3.txt'};
cfg.passive_files = {'passive1.txt','passive2.txt','passive3.txt','passive4.txt','passive5.txt','passive6.txt'};
cfg.platform_file = 'platform.csv'; cfg.truth_file = spec.truth_file;
cfg.processing_framework = 'joint_2d3d'; cfg.metrics_max_print = 0;
cfg.result_save_file = ''; cfg.plot_save_dir = ''; cfg.time_range_s = [];
cfg.max_rows = inf;
cfg = validate_config_fusion(cfg);
save(fullfile(report_dir,'effective_config.mat'),'cfg','spec');
details = struct('stage','parse'); stage_start = tic;
try
    [active_list,passive_list] = load_radar_measurement_files(cfg);
    details.parse_s = toc(stage_start);
    na = sum(cellfun(@(x) x.n_meas,active_list));
    np = sum(cellfun(@(x) x.n_meas,passive_list));
    nr = sum(cellfun(@(x) nnz(x.range_valid),active_list));
    nt = sum(cellfun(@(x) x.n_truncated_targets,[active_list(:);passive_list(:)]));
    assert(na == spec.expected.active && np == spec.expected.passive && ...
        nr == spec.expected.ranges && nt == spec.expected.truncated, ...
        'stress:ParserCounts', 'Parsed counts differ from the independent fixture manifest.');
    details.input_active = na; details.input_passive = np; details.truncated = nt;
    if strcmp(spec.expected.stage,'parser'), return; end
    details.stage = 'platform';
    platform = load_platform_txt(fullfile(cfg.data_dir,cfg.platform_file),cfg);
    if ~isempty(spec.expected.error_id)
        try
            assert_platform_covers_measurements(platform,active_list,passive_list);
        catch ME
            assert(strcmp(ME.identifier,spec.expected.error_id), ...
                'stress:WrongRejection', 'Unexpected platform rejection: %s',ME.identifier);
            details.expected_rejection = ME.identifier;
            return;
        end
        error('stress:MissingRejection','Out-of-domain platform query was accepted.');
    end
    assert_platform_covers_measurements(platform,active_list,passive_list);
    stage_start = tic; details.stage = 'events';
    [frames,coherence_info] = cohere_measurements(active_list,passive_list,platform,cfg);
    [events,event_info] = build_joint_measurement_events(frames,cfg);
    details.events_s = toc(stage_start); details.n_events = numel(events);
    assert(~isempty(events),'stress:EmptyEvents','Valid fixture produced no events.');
    event_na = sum(arrayfun(@(e) e.active.n_meas,events));
    event_np = sum(arrayfun(@(e) e.passive.n_meas,events));
    assert(event_info.n_active == coherence_info.n_active_after_condensation && ...
        event_info.n_passive == np && ...
        event_na+event_info.n_active_duplicates+coherence_info.n_active_condensed == na && ...
        event_np+event_info.n_passive_duplicates == np, ...
        'stress:EventAccounting','Condensation/event construction lost measurements.');
    event_info.preprocessing = coherence_info;
    clear active_list passive_list frames;
    details.stage = 'filter';
    est = run_filter_joint_2d3d(events,platform,cfg);
    assert_fusion_stress_invariants(est,cfg);
    details.filter_s = est.timing.total; details.filter_stages = est.timing.stages;
    details.output_2d = sum(est.N2); details.output_3d = sum(est.N);
    info = whos('est'); details.estimate_bytes = info.bytes;
    details.stats = est.stats;
    if spec.expected.require_outputs
        assert(sum(est.N_total)>0,'stress:NoOutput','Fixture produced no formal output.');
    end
    if spec.expected.only_2d
        assert(all(est.N==0) && any(est.N2>0),'stress:FalseRange','Angle-only input fabricated a 3-D output.');
    end
    if strcmp(job.name,'exact_retransmissions')
        assert(event_info.n_active_duplicates>0 && event_info.n_passive_duplicates>0, ...
            'stress:Retransmission','Exact cross-file retransmissions were not detected.');
    end
    if strcmp(job.name,'capacity_450')
        assert(est.stats.deleted_capacity>0,'stress:CapacityNotExercised','Capacity fixture did not reach the configured cap.');
    end
    details.stage = 'metrics'; stage_start = tic;
    metrics = evaluate_joint_tracking_metrics(est,events,cfg,platform);
    details.metrics_s = toc(stage_start);
    details.real_truth = metrics.real_truth;
    details.measurement_consistency = metrics.measurement_consistency.overall.track_accuracy;
    assert(strcmp(metrics.real_truth.status,spec.expected.truth_status), ...
        'stress:TruthStatus','Unexpected truth evaluation status: %s (%s)',metrics.real_truth.status,metrics.real_truth.reason);
    if ~isempty(spec.expected.rmse_limit_m)
        assert(metrics.real_truth.position.n>0 && ...
            metrics.real_truth.position.rmse_3d_m <= spec.expected.rmse_limit_m, ...
            'stress:BaselineAccuracy','Clean baseline exceeds the declared position RMSE bound.');
        assert(metrics.real_truth.target_coverage_rate == 1, ...
            'stress:BaselineCoverage','Clean separated targets were not all covered.');
    end
    save(fullfile(report_dir,'metrics.mat'),'metrics','event_info');
    details.stage = 'complete';
catch ME
    error_report = getReport(ME,'extended','hyperlinks','off'); %#ok<NASGU>
    replay_names = {'cfg','spec','details','error_report'};
    for variable = {'events','est','platform'}
        if exist(variable{1},'var'), replay_names{end+1} = variable{1}; end %#ok<AGROW>
    end
    save(fullfile(report_dir,'dataset_failure.mat'),replay_names{:},'-v7.3');
    rethrow(ME);
end
end

function result = result_template()
result = struct('name','','seed',0,'status','pending','elapsed_s',0, ...
    'error_identifier','','message','','details',struct());
end

function write_text(path,text)
fid = fopen(path,'w','n','UTF-8');
assert(fid>=0,'stress:ReportIO','Cannot write report: %s',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',text);
end

function records = source_hashes(root)
files = [dir(fullfile(root,'*.m')); dir(fullfile(root,'stress_tests','*.m'))];
records = repmat(struct('file','','sha256',''),numel(files),1);
for i = 1:numel(files)
    file = fullfile(files(i).folder,files(i).name);
    fid = fopen(file,'rb'); assert(fid>=0);
    bytes = fread(fid,inf,'*uint8'); fclose(fid);
    hash = java.security.MessageDigest.getInstance('SHA-256');
    hash.update(typecast(bytes,'int8'));
    raw = typecast(hash.digest(),'uint8');
    records(i).file = file;
    records(i).sha256 = lower(reshape(dec2hex(raw,2).',1,[]));
end
end
