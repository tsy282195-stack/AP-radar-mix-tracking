function info = export_track_diagnostics(est, events, cfg, track_ids, opts)
%EXPORT_TRACK_DIAGNOSTICS Export selected logical-track metrics and history.

if nargin < 1 || isempty(est), est = struct(); end
if nargin < 2 || isempty(events), events = repmat(empty_event(), 0, 1); end
if nargin < 3 || isempty(cfg), cfg = struct(); end
if nargin < 4, track_ids = []; end
if nargin < 5 || isempty(opts), opts = struct(); end

output_dir = get_opt(opts, 'output_dir', fullfile(pwd, 'track_reports'));
print_console = logical(get_opt(opts, 'print_console', true));
metrics = get_opt(opts, 'metrics', struct());

available_ids = output_track_ids(est);
if isempty(track_ids)
    selected_ids = available_ids;
else
    selected_ids = intersect(reshape(track_ids, 1, []), available_ids, 'stable');
end
missing_ids = setdiff(reshape(track_ids, 1, []), available_ids);
if print_console && ~isempty(missing_ids)
    fprintf(2, '[单轨报告] 以下航迹没有正式输出，已跳过: %s\n', mat2str(missing_ids));
end

info = struct('selected_ids', selected_ids, 'missing_ids', missing_ids, ...
    'files', {cell(1, numel(selected_ids))}, 'output_dir', output_dir, ...
    'n_reports', 0);
if isempty(selected_ids)
    if print_console, fprintf('[单轨报告] 没有可导出的正式逻辑航迹。\n'); end
    return;
end
metric_ids = zeros(1, 0);
if isstruct(metrics) && isfield(metrics, 'evaluation_version') && ...
        metrics.evaluation_version == 2 && isfield(metrics, 'track_details') && ...
        ~isempty(metrics.track_details) && isfield(metrics.track_details, 'track_id')
    metric_ids = [metrics.track_details.track_id];
end
if ~all(ismember(selected_ids, metric_ids))
    cfg_eval = cfg;
    cfg_eval.metrics_max_print = 0;
    cfg_eval.metrics_progress_enabled = false;
    metrics = evaluate_joint_tracking_metrics(est, events, cfg_eval);
end
if ~exist(output_dir, 'dir')
    [ok, msg] = mkdir(output_dir);
    if ~ok, error('export_track_diagnostics:CreateDir', '无法创建报告目录: %s', msg); end
end

truth_labels = build_joint_truth_labels(events, cfg);
for i = 1:numel(selected_ids)
    id = selected_ids(i);
    detail = find_track_detail(metrics, id);
    file_path = fullfile(output_dir, sprintf('track_%s_report.txt', safe_id_text(id)));
    write_track_report(file_path, id, detail, est, events, truth_labels);
    info.files{i} = file_path;
    info.n_reports = info.n_reports + 1;
    if print_console
        print_track_console(detail, file_path);
    end
end
end

function write_track_report(path, id, detail, est, events, truth_labels)
[fid, msg] = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0, error('export_track_diagnostics:OpenFile', '无法写入 %s: %s', path, msg); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '逻辑航迹诊断报告\n');
fprintf(fid, '生成时间\t%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '航迹ID\t%.15g\n', id);
fprintf(fid, '说明\t航迹正确判定使用一对一匹配、最少关联点和真值量测覆盖门限。\n');
fprintf(fid, '判定公式\t正确关联到匹配真值的量测数/该真值在对应评价范围的全部量测数。\n');
fprintf(fid, '评价范围\t总体为全部输入，三维为RAE，二维被动为进入二维处理支路的被动AE，确认和输出不影响参考点计数。\n');
fprintf(fid, '说明\t关联覆盖率与同事件正式输出覆盖率分别统计，不可互相替代。\n');
fprintf(fid, '误差参考\t同事件同真值的量测均值；不是外部真实真值精度，逐点与汇总均使用总体一对一匹配。\n');
fprintf(fid, '说明\t新息为滤波更新前的量测减预测；出生量测没有更新新息，记为NaN。\n\n');

fprintf(fid, '[航迹基本信息]\n');
fprintf(fid, '输出点数\t%d\n', detail.n_output_points);
fprintf(fid, '输出事件/帧数\t%d\n', detail.n_output_events);
fprintf(fid, '二维输出点数\t%d\n', detail.n_output_2d);
fprintf(fid, '三维输出点数\t%d\n', detail.n_output_3d);
fprintf(fid, '起始时间_s\t%.9f\n', detail.start_time_s);
fprintf(fid, '结束时间_s\t%.9f\n', detail.end_time_s);
fprintf(fid, '持续时间_s\t%.9f\n', detail.duration_s);
fprintf(fid, '关联总数\t%d\n', detail.n_associations);
fprintf(fid, '主动三维关联数\t%d\n', detail.n_active_range_assoc);
fprintf(fid, '主动纯角度关联数\t%d\n', detail.n_active_angle_assoc);
fprintf(fid, '被动角度关联数\t%d\n\n', detail.n_passive_assoc);

write_scope_summary(fid, '总体主被动评价', detail.overall);
write_scope_summary(fid, '三维主动评价', detail.three_d);
write_scope_summary(fid, '二维被动AE评价', detail.two_d);

fprintf(fid, '[单轨RMSE：量测参考一致性]\n');
fprintf(fid, 'RMSE_reference\tmeasurement_event_mean\n');
fprintf(fid, '匹配真值实例\t%.15g\n', detail.overall.matched_truth_id);
fprintf(fid, '角度样本数\t%d\n', detail.angle_rmse.n);
fprintf(fid, '方位RMSE_deg\t%.9f\n', detail.angle_rmse.az_deg);
fprintf(fid, '俯仰RMSE_deg\t%.9f\n', detail.angle_rmse.el_deg);
fprintf(fid, 'LOS_RMSE_deg\t%.9f\n', detail.angle_rmse.los_deg);
fprintf(fid, '位置样本数\t%d\n', detail.position_rmse.n);
fprintf(fid, 'East_RMSE_m\t%.9f\n', detail.position_rmse.e_m);
fprintf(fid, 'North_RMSE_m\t%.9f\n', detail.position_rmse.n_m);
fprintf(fid, 'Up_RMSE_m\t%.9f\n', detail.position_rmse.u_m);
fprintf(fid, '3D_RMSE_m\t%.9f\n\n', detail.position_rmse.three_d_m);

write_output_history(fid, id, detail, est, events, truth_labels);
write_association_history(fid, id, detail, est, events, truth_labels);
end

function write_scope_summary(fid, label, d)
fprintf(fid, '[%s]\n', label);
fprintf(fid, '适用\t%d\n', d.applicable);
fprintf(fid, '存在一对一匹配\t%d\n', d.has_match);
fprintf(fid, '匹配真值实例\t%.15g\n', d.matched_truth_id);
fprintf(fid, '带标签关联数\t%d\n', d.n_labeled_assoc);
fprintf(fid, '正确关联数\t%d\n', d.n_correct);
fprintf(fid, '不一致关联数\t%d\n', d.n_inconsistent);
fprintf(fid, '关联一致率\t%.9f\n', d.association_consistency);
fprintf(fid, '对应真值全部量测数\t%d\n', d.truth_total);
fprintf(fid, '关联覆盖率\t%.9f\n', d.coverage);
fprintf(fid, '未正确覆盖量测数\t%d\n', d.n_truth_not_correct);
fprintf(fid, '正式输出同步覆盖率\t%.9f\n', d.formal_output_coverage);
fprintf(fid, '正式输出覆盖量测数\t%d\n', d.n_formal_output_covered);
fprintf(fid, '正式输出参考量测数\t%d\n', d.n_formal_output_reference);
fprintf(fid, '真值量测覆盖门限\t%.9f\n', d.purity_threshold);
fprintf(fid, '最少带标签关联点\t%d\n', d.min_labeled_assoc);
fprintf(fid, '航迹级正确判定\t%d\n', d.is_correct);
fprintf(fid, '单轨航迹级正确率\t%.2f%%\n\n', 100 * d.track_level_score);
end

function write_output_history(fid, id, detail, est, events, truth_labels)
fprintf(fid, '[逐输出点]\n');
fprintf(fid, ['event\ttime_s\toutput_dim\tmode\tstatus\taz_deg\tel_deg\trange_m' ...
    '\tE_m\tN_m\tU_m\tVE_mps\tVN_mps\tVU_mps\tangle_truth\tref_az_deg' ...
    '\tref_el_deg\taz_error_deg\tel_error_deg\tlos_error_deg\tposition_truth' ...
    '\tref_E_m\tref_N_m\tref_U_m\terr_E_m\terr_N_m\terr_U_m\terr_3D_m\n']);

for k = 1:min(numel(events), numel_field(est, 'output'))
    out = est.output{k};
    for q = 1:numel(out)
        if field_num(out(q), 'id', NaN) ~= id, continue; end
        angle_truth = detail.overall.matched_truth_id;
        position_truth = detail.overall.matched_truth_id;
        [ref_angle, ~] = event_reference(events(k), truth_labels, k, angle_truth);
        [~, ref_position] = event_reference(events(k), truth_labels, k, position_truth);
        az = field_num(out(q), 'az_deg', NaN);
        el = field_num(out(q), 'el_deg', NaN);
        pos = field_vec(out(q), 'position_enu', 3);
        vel = field_vec(out(q), 'velocity_enu', 3);
        az_err = angle_delta(az, ref_angle(1));
        el_err = el - ref_angle(2);
        los_err = los_separation_deg(az, el, ref_angle(1), ref_angle(2));
        pos_err = pos - ref_position;
        if field_num(out(q), 'output_dim', 0) ~= 3, pos_err(:) = NaN; end
        err3 = norm_if_finite(pos_err);
        fprintf(fid, ['%d\t%.9f\t%d\t%s\t%d\t%.9f\t%.9f\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.15g\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.15g\t%.9f\t%.9f\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\n'], ...
            k, field_num(out(q), 't_sec', events(k).t_sec), ...
            field_num(out(q), 'output_dim', 0), field_text(out(q), 'mode', ''), ...
            field_num(out(q), 'status_code', 0), az, el, ...
            field_num(out(q), 'range_m', NaN), pos, vel, angle_truth, ...
            ref_angle, az_err, el_err, los_err, position_truth, ...
            ref_position, pos_err, err3);
    end
end
fprintf(fid, '\n');
end

function write_association_history(fid, id, detail, est, events, truth_labels)
fprintf(fid, '[逐关联与新息]\n');
fprintf(fid, ['event\ttime_s\ttype\tfilter_dim\tmeas_index\traw_truth\tmapped_truth' ...
    '\tmatched_truth\tis_consistent\tinnovation_kind\tinnovation_unit\tnu1\tnu2' ...
    '\tnu3\tnis\tnormalized_nis\tgroup_size\tassociation_cost\tmeas_az_deg' ...
    '\tmeas_el_deg\tmeas_E_m\tmeas_N_m\tmeas_U_m\n']);
available = 0; total = 0;
for k = 1:min(numel(events), numel_field(est, 'assoc'))
    a = est.assoc{k};
    if isempty(a) || ~isfield(a, 'id'), continue; end
    for q = find(reshape(a.id, 1, []) == id)
        total = total + 1;
        type = indexed_text(a, 'type', q, '');
        mi = indexed_num(a, 'meas_index', q, 0);
        [mapped_truth, meas_t, meas_angle, meas_pos] = association_measurement( ...
            events(k), truth_labels, k, type, mi);
        matched_truth = detail.overall.matched_truth_id;
        is_consistent = isfinite(mapped_truth) && isfinite(matched_truth) && ...
            mapped_truth == matched_truth;
        innovation = indexed_innovation(a, q);
        nis = association_nis(a, q);
        group_size = indexed_num(a, 'group_size', q, 1);
        innovation_kind = indexed_num(a, 'innovation_kind', q, 0);
        if innovation_kind == 0
            measurement_dim = indexed_num(a, 'measurement_dim', q, 0);
            if measurement_dim == 3, innovation_kind = 2; end
            if measurement_dim == 2, innovation_kind = 1; end
        end
        if innovation_kind == 2
            unit = 'm'; innovation_dim = 3;
        elseif innovation_kind == 1
            unit = 'deg'; innovation_dim = 2;
        else
            unit = 'N/A'; innovation_dim = NaN;
        end
        normalized_nis = nis / innovation_dim;
        if any(isfinite(innovation)) && isfinite(nis), available = available + 1; end
        fprintf(fid, ['%d\t%.9f\t%s\t%d\t%d\t%.15g\t%.15g\t%.15g\t%d' ...
            '\t%d\t%s\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%d\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n'], ...
            k, meas_t, type, association_filter_dim(est, k, a, q, id), mi, ...
            indexed_num(a, 'tid', q, NaN), mapped_truth, matched_truth, ...
            is_consistent, innovation_kind, unit, innovation, nis, ...
            normalized_nis, round(group_size), indexed_num(a, 'cost', q, NaN), ...
            meas_angle, meas_pos);
    end
end
fprintf(fid, '\n[新息完整性]\n');
fprintf(fid, '关联记录数\t%d\n', total);
fprintf(fid, '含新息和NIS记录数\t%d\n', available);
fprintf(fid, '缺少新息记录数\t%d\n', total - available);
if available < total
    fprintf(fid, '说明\t出生量测或旧版本滤波结果没有历史新息，字段保持NaN。\n');
end
end

function print_track_console(d, path)
if d.n_output_3d > 0 && d.n_output_2d > 0
    mode = '2D/3D';
elseif d.n_output_3d > 0
    mode = '3D';
else
    mode = '2D';
end
fprintf(['[单轨报告] Track %.15g: %s, 输出=%d点/%d帧, 总体真值=%.15g, ' ...
    '关联覆盖=%.2f%%, 正式输出覆盖=%.2f%%, 一致率=%.2f%%, 航迹级=%s\n'], ...
    d.track_id, mode, d.n_output_points, d.n_output_events, ...
    d.overall.matched_truth_id, 100 * d.overall.coverage, ...
    100 * d.overall.formal_output_coverage, ...
    100 * d.overall.association_consistency, pass_text(d.overall.is_correct));
if d.position_rmse.n > 0
    fprintf('             量测参考3D RMSE=%.2fm (%d点)', ...
        d.position_rmse.three_d_m, d.position_rmse.n);
end
if d.angle_rmse.n > 0
    fprintf(', 量测参考LOS RMSE=%.4fdeg (%d点)', d.angle_rmse.los_deg, d.angle_rmse.n);
end
fprintf('\n             TXT: %s\n', path);
end

function d = find_track_detail(metrics, id)
details = metrics.track_details;
i = find([details.track_id] == id, 1);
if isempty(i), error('export_track_diagnostics:MissingDetail', '评价中不存在航迹 %.15g。', id); end
d = details(i);
end

function ids = output_track_ids(est)
ids = zeros(1, 0);
if isfield(est, 'output')
    for k = 1:numel(est.output)
        out = est.output{k};
        if ~isempty(out) && isfield(out, 'id'), ids = [ids, [out.id]]; end %#ok<AGROW>
    end
end
ids = unique(ids(isfinite(ids)));
end

function [mapped_truth, t, angle, position] = association_measurement(e, labels, k, type, mi)
mapped_truth = NaN; t = e.t_sec; angle = nan(2, 1); position = nan(3, 1);
if mi < 1 || ~ischar(type), return; end
if strncmp(type, 'active', 6)
    if k <= numel(labels.active) && mi <= numel(labels.active{k})
        mapped_truth = labels.active{k}(mi);
    end
    if mi <= e.active.n_meas
        t = indexed_vector(e.active.t_sec, mi, e.t_sec);
        if size(e.active.rae, 1) >= 3 && size(e.active.rae, 2) >= mi
            angle = e.active.rae(2:3, mi);
        end
        if size(e.active.xyz, 1) >= 3 && size(e.active.xyz, 2) >= mi
            position = e.active.xyz(1:3, mi);
        end
    end
elseif strncmp(type, 'passive', 7)
    if k <= numel(labels.passive) && mi <= numel(labels.passive{k})
        mapped_truth = labels.passive{k}(mi);
    end
    if mi <= e.passive.n_meas
        t = indexed_vector(e.passive.t_sec, mi, e.t_sec);
        if size(e.passive.ang, 1) >= 2 && size(e.passive.ang, 2) >= mi
            angle = e.passive.ang(1:2, mi);
        end
    end
end
end

function [angle, position] = event_reference(e, labels, k, truth_id)
angle = nan(2, 1); position = nan(3, 1);
if ~isfinite(truth_id), return; end
active_label = labels.active{k}; passive_label = labels.passive{k};
angles = zeros(2, 0);
ia = find(active_label == truth_id);
ia = ia(ia <= size(e.active.rae, 2));
if ~isempty(ia) && size(e.active.rae, 1) >= 3
    angles = [angles, e.active.rae(2:3, ia)]; %#ok<AGROW>
end
ip = find(passive_label == truth_id);
ip = ip(ip <= size(e.passive.ang, 2));
if ~isempty(ip) && size(e.passive.ang, 1) >= 2
    angles = [angles, e.passive.ang(1:2, ip)]; %#ok<AGROW>
end
angles = angles(:, all(isfinite(angles), 1));
if ~isempty(angles)
    angle = [atan2d(mean(sind(angles(1, :))), mean(cosd(angles(1, :)))); ...
        mean(angles(2, :))];
end
has_range = false(1, e.active.n_meas);
n = min(numel(e.active.has_range), e.active.n_meas);
if n > 0, has_range(1:n) = logical(e.active.has_range(1:n)); end
iz = find(active_label == truth_id & has_range);
iz = iz(iz <= size(e.active.xyz, 2));
if ~isempty(iz) && size(e.active.xyz, 1) >= 3
    Z = e.active.xyz(1:3, iz); Z = Z(:, all(isfinite(Z), 1));
    if ~isempty(Z), position = mean(Z, 2); end
end
end

function out = empty_event()
out = struct('t_sec', NaN, 'active', struct('n_meas', 0, 't_sec', [], ...
    'rae', zeros(3, 0), 'xyz', zeros(3, 0), 'has_range', false(1, 0)), ...
    'passive', struct('n_meas', 0, 't_sec', [], 'ang', zeros(2, 0)));
end

function n = numel_field(s, name)
if isfield(s, name), n = numel(s.(name)); else, n = 0; end
end

function v = field_num(s, name, fallback)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = fallback; end
if ~isscalar(v), v = fallback; end
end

function v = field_vec(s, name, n)
v = nan(n, 1);
if ~isfield(s, name) || isempty(s.(name)), return; end
x = s.(name)(:); q = min(n, numel(x)); v(1:q) = x(1:q);
end

function s = field_text(x, name, fallback)
if isfield(x, name) && ischar(x.(name)), s = x.(name); else, s = fallback; end
s = strrep(s, sprintf('\t'), ' '); s = strrep(s, sprintf('\n'), ' ');
end

function v = indexed_num(s, name, i, fallback)
v = fallback;
if isfield(s, name) && numel(s.(name)) >= i
    x = s.(name)(i); if isfinite(x), v = x; elseif isnan(x), v = NaN; end
end
end

function s = indexed_text(x, name, i, fallback)
s = fallback;
if isfield(x, name) && iscell(x.(name)) && numel(x.(name)) >= i && ...
        ischar(x.(name){i})
    s = x.(name){i};
end
end

function v = indexed_innovation(a, q)
v = nan(3, 1);
if isfield(a, 'innovation') && size(a.innovation, 2) >= q
    x = a.innovation(:, q); n = min(3, numel(x)); v(1:n) = x(1:n);
end
end

function nis = association_nis(a, q)
nis = indexed_num(a, 'nis', q, NaN);
if ~isfinite(nis)
    nis = indexed_num(a, 'space_nis', q, NaN);
end
end

function dim = association_filter_dim(est, k, a, q, id)
dim = indexed_num(a, 'filter_dim', q, NaN);
if isfinite(dim) && any(dim == [2, 3]), return; end
dim = 0;
if ~isfield(est, 'logical_tracks') || k > numel(est.logical_tracks) || ...
        isempty(est.logical_tracks{k})
    return;
end
tracks = est.logical_tracks{k};
j = find([tracks.id] == id, 1);
if isempty(j), return; end
mode = field_text(tracks(j), 'mode', '');
if strncmp(mode, '2d', 2)
    dim = 2;
elseif strncmp(mode, '3d', 2)
    dim = 3;
elseif isfield(tracks, 'state3d') && all(isfinite(tracks(j).state3d))
    dim = 3;
elseif isfield(tracks, 'angle_state') && all(isfinite(tracks(j).angle_state))
    dim = 2;
end
end

function v = indexed_vector(x, i, fallback)
if isempty(x), v = fallback; elseif isscalar(x), v = x; ...
elseif numel(x) >= i, v = x(i); else, v = fallback; end
end

function d = angle_delta(a, b)
if ~isfinite(a) || ~isfinite(b), d = NaN; else, d = mod(a - b + 180, 360) - 180; end
end

function n = norm_if_finite(x)
if all(isfinite(x)), n = norm(x); else, n = NaN; end
end

function s = safe_id_text(id)
s = sprintf('%.15g', id);
s = regexprep(s, '[^0-9A-Za-z_-]', '_');
end

function s = pass_text(tf)
if tf, s = '正确'; else, s = '不正确'; end
end

function v = get_opt(opts, name, fallback)
if isstruct(opts) && isfield(opts, name) && ~isempty(opts.(name))
    v = opts.(name);
else
    v = fallback;
end
end
