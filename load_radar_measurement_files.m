function [active_list, passive_list, info] = load_radar_measurement_files(cfg)
%LOAD_RADAR_MEASUREMENT_FILES Parse independent radar files in parallel.

n_active = numel(cfg.active_files); n_passive = numel(cfg.passive_files);
n_tasks = n_active + n_passive;
active_list = cell(n_active, 1); passive_list = cell(n_passive, 1);
info = struct('mode', 'serial', 'n_workers', 1, 'elapsed_s', 0);
if n_tasks == 0, return; end

paths = cell(n_tasks, 1); kinds = zeros(n_tasks, 1);
for i = 1:n_active
    paths{i} = resolve_data_path(cfg.data_dir, cfg.active_files{i});
    kinds(i) = 1;
end
for i = 1:n_passive
    q = n_active + i;
    paths{q} = resolve_data_path(cfg.data_dir, cfg.passive_files{i});
    kinds(q) = 2;
end

use_parallel = isfield(cfg, 'parallel_file_loading') && cfg.parallel_file_loading && ...
    n_tasks > 1 && license('test', 'Distrib_Computing_Toolbox') && ...
    exist('parpool', 'file') == 2;
t0 = tic;
if use_parallel
    try
        pool = gcp('nocreate');
        if isempty(pool)
            cluster = parcluster('local');
            job_dir = fullfile(tempdir, 'fusion_parallel_jobs');
            if exist(job_dir, 'dir') ~= 7, mkdir(job_dir); end
            cluster.JobStorageLocation = job_dir;
            requested = cfg.parallel_file_workers;
            if requested <= 0, requested = cluster.NumWorkers; end
            requested = min([requested, n_tasks, cluster.NumWorkers]);
            pool = parpool(cluster, max(1, requested));
        end
        parsed = cell(n_tasks, 1);
        parfor q = 1:n_tasks
            if kinds(q) == 1
                parsed{q} = parse_active_wide_txt(paths{q}, cfg);
            else
                parsed{q} = parse_passive_wide_txt(paths{q}, cfg);
            end
        end
        active_list = parsed(1:n_active);
        passive_list = parsed(n_active + 1:end);
        info.mode = 'parallel'; info.n_workers = pool.NumWorkers;
    catch ME
        warning('load_radar_measurement_files:ParallelFallback', ...
            '并行文件解析失败，回退串行: %s', ME.message);
        [active_list, passive_list] = parse_serial(paths, kinds, n_active, n_passive, cfg);
        info.mode = 'serial_fallback'; info.n_workers = 1;
    end
else
    [active_list, passive_list] = parse_serial(paths, kinds, n_active, n_passive, cfg);
end
info.elapsed_s = toc(t0);
end

function [active_list, passive_list] = parse_serial(paths, kinds, n_active, n_passive, cfg)
active_list = cell(n_active, 1); passive_list = cell(n_passive, 1);
for q = 1:numel(paths)
    if kinds(q) == 1
        active_list{q} = parse_active_wide_txt(paths{q}, cfg);
    else
        passive_list{q - n_active} = parse_passive_wide_txt(paths{q}, cfg);
    end
end
end
