function plot_joint_tracking_results(est, events, cfg)
%PLOT_JOINT_TRACKING_RESULTS Plot unified angle and spatial track outputs.

[ids, hist] = collect_output_history(est);
min_life = get_cfg(cfg, 'joint_plot_min_life', 3);
colors = lines(max(numel(ids), 1));
has2 = arrayfun(@(h) count_mode_points(h, 2) >= min_life, hist);
has3 = arrayfun(@(h) count_mode_points(h, 3) >= min_life, hist);
[act_ang, pas_ang, act_xyz] = collect_raw_measurements(events);

% Figure 1: formal 2-D output only. Formal 3-D samples are masked out.
if any(has2)
    figure('Name', '二维纯角度滤波航迹总体图', 'Position', [80, 100, 920, 720]);
    hold on; grid on; box on;
    handles = gobjects(1, 0); labels = cell(1, 0);
    idx2 = find(has2);
    for q = 1:numel(idx2)
        i = idx2(q);
        [az, el] = angle_mode_series(hist(i), 2);
        h = plot(az, el, '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        handles(end + 1) = h; %#ok<AGROW>
        labels{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
        first = find(isfinite(az) & isfinite(el), 1);
        if ~isempty(first)
            text(az(first), el(first), sprintf('  %d', ids(i)), ...
                'Color', colors(i, :), 'FontWeight', 'bold');
        end
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    title(sprintf('二维纯角度滤波航迹总体图（至少%d个二维输出点，共%d条）', ...
        min_life, numel(idx2)));
    if ~isempty(handles), legend(handles, labels, 'Location', 'bestoutside'); end
    hold off;
end

% Figure 2: raw AE observations, kept separate from filtered trajectories.
figure('Name', '二维角度量测', 'Position', [120, 130, 920, 720]);
hold on; grid on; box on;
if ~isempty(act_ang)
    plot(act_ang(1, :), act_ang(2, :), '.', 'Color', [0.15 0.45 0.80], ...
        'MarkerSize', 7, 'DisplayName', '主动AE');
end
if ~isempty(pas_ang)
    plot(pas_ang(1, :), pas_ang(2, :), '.', 'Color', [0.85 0.35 0.15], ...
        'MarkerSize', 7, 'DisplayName', '被动AE');
end
xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
title(sprintf('二维角度量测（主动%d点，被动%d点）', size(act_ang, 2), size(pas_ang, 2)));
if ~isempty(act_ang) || ~isempty(pas_ang), legend('Location', 'best'); end
hold off;

% Figure 3: raw range measurements and formal 3-D output. 2-D intervals
% remain as line breaks, so missing formal 3-D segments stay visible.
if any(has3) || ~isempty(act_xyz)
    figure('Name', '三维主动空间滤波航迹', 'Position', [160, 160, 980, 760]);
    hold on; grid on; box on;
    h3 = gobjects(1, 0); l3 = cell(1, 0);
    if ~isempty(act_xyz)
        h = plot3(act_xyz(1, :), act_xyz(2, :), act_xyz(3, :), '.', ...
            'Color', [0.65 0.65 0.65], 'MarkerSize', 5);
        h3(end + 1) = h; l3{end + 1} = '主动距离量测'; %#ok<AGROW>
    end
    idx3 = find(has3);
    for q = 1:numel(idx3)
        i = idx3(q);
        pos = position_mode_series(hist(i), 3);
        h = plot3(pos(1, :), pos(2, :), pos(3, :), ...
            '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        h3(end + 1) = h; %#ok<AGROW>
        l3{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title(sprintf('三维主动量测与正式输出（量测%d点，至少%d个输出点的航迹%d条）', ...
        size(act_xyz, 2), min_life, numel(idx3)));
    view(45, 30); axis equal;
    if ~isempty(h3), legend(h3, l3, 'Location', 'bestoutside'); end
    hold off;
end

figure('Name', '逻辑航迹模式统计', 'Position', [200, 190, 900, 420]);
plot(est.filter_times, est.N2, 'LineWidth', 1.3, 'DisplayName', '二维输出'); hold on;
plot(est.filter_times, est.N, 'LineWidth', 1.3, 'DisplayName', '三维输出');
plot(est.filter_times, est.N_total, 'k:', 'LineWidth', 1.1, 'DisplayName', '逻辑航迹总数');
grid on; box on; xlabel('Time (s)'); ylabel('航迹数'); title('逻辑航迹输出维度');
legend('Location', 'best'); hold off;

if isfield(cfg, 'plot_save_dir') && ~isempty(cfg.plot_save_dir)
    save_all_open_figures(cfg.plot_save_dir);
end
end

function n = count_mode_points(h, mode)
if mode == 2
    valid = h.dim == 2 & isfinite(h.az) & isfinite(h.el);
else
    valid = h.dim == 3 & all(isfinite(h.pos), 1);
end
n = sum(valid);
end

function [az, el] = angle_mode_series(h, mode)
valid = h.dim == mode & isfinite(h.az) & isfinite(h.el);
az = h.az; el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_az_wrap(az, el);
end

function pos = position_mode_series(h, mode)
valid = h.dim == mode & all(isfinite(h.pos), 1);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, hist] = collect_output_history(est)
n_total = sum(cellfun(@numel, est.output));
track_id = nan(1, n_total); time = nan(1, n_total);
az = nan(1, n_total); el = nan(1, n_total); dim = zeros(1, n_total);
pos = nan(3, n_total); p = 0;
for k = 1:numel(est.output)
    out = est.output{k};
    n = numel(out); ii = p + (1:n);
    if n == 0, continue; end
    track_id(ii) = reshape([out.id], 1, []);
    time(ii) = reshape([out.t_sec], 1, []);
    az(ii) = reshape([out.az_deg], 1, []);
    el(ii) = reshape([out.el_deg], 1, []);
    dim(ii) = reshape([out.output_dim], 1, []);
    pos(:, ii) = cat(2, out.position_enu);
    p = p + n;
end
track_id = track_id(1:p); time = time(1:p); az = az(1:p); el = el(1:p);
dim = dim(1:p); pos = pos(:, 1:p);
ids = unique(track_id(isfinite(track_id)));
hist = repmat(struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0)), numel(ids), 1);
if isempty(ids), return; end
[found, group] = ismember(track_id, ids);
[group, order] = sort(group(found));
source = find(found); source = source(order);
edge = [1, find(diff(group) ~= 0) + 1, numel(group) + 1];
for q = 1:numel(edge) - 1
    block = edge(q):edge(q + 1) - 1; i = group(block(1)); ii = source(block);
    hist(i).t = time(ii); hist(i).az = az(ii); hist(i).el = el(ii);
    hist(i).dim = dim(ii); hist(i).pos = pos(:, ii);
end
end

function [active, passive, xyz] = collect_raw_measurements(events)
n_active = sum(arrayfun(@(e) e.active.n_meas, events));
n_passive = sum(arrayfun(@(e) e.passive.n_meas, events));
n_xyz = 0;
for k = 1:numel(events)
    n = min(numel(events(k).active.has_range), size(events(k).active.xyz, 2));
    if n > 0
        block = events(k).active.xyz(:, 1:n);
        has_range = logical(reshape(events(k).active.has_range(1:n), 1, []));
        n_xyz = n_xyz + nnz(has_range & ...
            all(isfinite(block), 1));
    end
end
active = nan(2, n_active); passive = nan(2, n_passive); xyz = nan(3, n_xyz);
pa = 0; pp = 0; px = 0;
for k = 1:numel(events)
    if events(k).active.n_meas > 0
        n = events(k).active.n_meas; ii = pa + (1:n);
        active(:, ii) = events(k).active.rae(2:3, :); pa = pa + n;
        has_range = logical(events(k).active.has_range(:).');
        has_range = has_range(1:min(numel(has_range), size(events(k).active.xyz, 2)));
        if any(has_range)
            block = events(k).active.xyz(:, 1:numel(has_range));
            use = has_range & all(isfinite(block), 1); n = nnz(use); ii = px + (1:n);
            xyz(:, ii) = block(:, use); px = px + n;
        end
    end
    if events(k).passive.n_meas > 0
        n = events(k).passive.n_meas; ii = pp + (1:n);
        passive(:, ii) = events(k).passive.ang; pp = pp + n;
    end
end
end

function [az, el] = break_az_wrap(az0, el0)
n = numel(az0); insert_break = false(1, n);
if n > 1
    insert_break(2:end) = isfinite(az0(1:end - 1)) & isfinite(az0(2:end)) & ...
        abs(az0(2:end) - az0(1:end - 1)) > 180;
end
target = (1:n) + cumsum(insert_break);
az = nan(1, n + nnz(insert_break)); el = nan(size(az));
az(target) = az0; el(target) = el0;
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end
