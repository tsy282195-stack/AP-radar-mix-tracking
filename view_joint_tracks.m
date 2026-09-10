function info = view_joint_tracks(est, events, opts)
%VIEW_JOINT_TRACKS Inspect selected logical tracks in angle or ENU space.

if nargin < 3, opts = struct(); end
[ids, H] = collect_history(est);
cfg = get_opt(opts, 'cfg', struct());
truth_labels = build_joint_truth_labels(events, cfg);
requested = get_opt(opts, 'track_ids', []);
if isempty(requested), selected = ids; else, selected = intersect(requested(:).', ids, 'stable'); end
if isempty(selected)
    error('view_joint_tracks:NoTrack', '所选航迹不存在。可用ID: %s', mat2str(ids));
end
show_ref = get_opt(opts, 'show_reference', true);
show_angle = get_opt(opts, 'show_angle', true);
show_enu = get_opt(opts, 'show_enu', true);
colors = lines(numel(selected));
matched_truth_2d = nan(1, numel(selected));
matched_truth_3d = nan(1, numel(selected));
selected_has2 = false(1, numel(selected));
selected_has3 = false(1, numel(selected));
for s = 1:numel(selected)
    h = H(ids == selected(s));
    matched_truth_2d(s) = truth_label_for_track( ...
        est, events, truth_labels, selected(s), 2);
    matched_truth_3d(s) = truth_label_for_track( ...
        est, events, truth_labels, selected(s), 3);
    selected_has2(s) = any(angle_mode_mask(h));
    selected_has3(s) = any(position_mode_mask(h));
end

fprintf('联合逻辑航迹可用ID: %s\n', mat2str(ids));
fprintf('当前查看ID: %s\n', mat2str(selected));

if show_angle && any(selected_has2)
    figure('Name', '选中二维纯角度航迹 AE', 'Position', [80, 90, 900, 700]);
    hold on; grid on; box on;
    for s = 1:numel(selected)
        if ~selected_has2(s), continue; end
        h = H(ids == selected(s));
        [az, el] = angle_mode_series(h);
        plot(az, el, '.-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)'); title('选中二维纯角度航迹');
    legend('Location', 'bestoutside'); hold off;

    figure('Name', '二维角度参考对比', 'Position', [120, 120, 980, 720]);
    for ax = 1:2
        subplot(2, 1, ax); hold on; grid on; box on;
        for s = 1:numel(selected)
            if ~selected_has2(s), continue; end
            h = H(ids == selected(s));
            valid = angle_mode_mask(h);
            val = h.az; if ax == 2, val = h.el; end
            val(~valid) = NaN;
            plot(h.t, unwrap_for_plot(val), '-', 'Color', colors(s, :), 'LineWidth', 1.3, ...
                'DisplayName', sprintf('Track %d 滤波', selected(s)));
            if show_ref
                ref = nan(2, numel(h.t));
                ref(:, valid) = angle_reference_for_track(est, events, truth_labels, ...
                    h.event_index(valid), selected(s), matched_truth_2d(s));
                rv = ref(1, :); if ax == 2, rv = ref(2, :); end
                plot(h.t, unwrap_for_plot(rv), '--', 'Color', colors(s, :), 'LineWidth', 1.0, ...
                    'DisplayName', sprintf('Track %d 参考', selected(s)));
            end
        end
        ylabel(axis_name(ax)); if ax == 1, title('二维纯角度滤波与参考量测'); end
        if ax == 2, xlabel('Time (s)'); end
        legend('Location', 'bestoutside'); hold off;
    end

    if show_ref
        figure('Name', '二维角度误差', 'Position', [150, 145, 980, 720]);
        for ax = 1:2
            subplot(2, 1, ax); hold on; grid on; box on;
            for s = 1:numel(selected)
                if ~selected_has2(s), continue; end
                h = H(ids == selected(s));
                valid = angle_mode_mask(h);
                ref = nan(2, numel(h.t));
                ref(:, valid) = angle_reference_for_track(est, events, truth_labels, ...
                    h.event_index(valid), selected(s), matched_truth_2d(s));
                err = nan(1, numel(h.t));
                if ax == 1
                    err(valid) = angle_diff(h.az(valid), ref(1, valid));
                else
                    err(valid) = h.el(valid) - ref(2, valid);
                end
                plot(h.t, err, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
                    'DisplayName', sprintf('Track %d', selected(s)));
            end
            yline(0, 'k:'); ylabel([axis_name(ax), '误差']);
            if ax == 1, title('二维角度误差'); end
            if ax == 2, xlabel('Time (s)'); end
            legend('Location', 'bestoutside'); hold off;
        end
    end
end

if show_enu && any(selected_has3)
    figure('Name', '选中三维主动航迹 ENU', 'Position', [180, 170, 940, 720]);
    hold on; grid on; box on;
    for s = 1:numel(selected)
        if ~selected_has3(s), continue; end
        h = H(ids == selected(s));
        meas = associated_range_positions(est, selected(s));
        if ~isempty(meas)
            plot3(meas(1, :), meas(2, :), meas(3, :), 'o', ...
                'Color', colors(s, :), 'MarkerSize', 3, 'LineStyle', 'none', ...
                'DisplayName', sprintf('Track %d 关联距离量测', selected(s)));
        end
        pos = position_mode_series(h);
        plot3(pos(1, :), pos(2, :), pos(3, :), '.-', ...
            'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title('选中三维主动空间航迹'); view(45, 30); axis equal;
    legend('Location', 'bestoutside'); hold off;

    plot_enu_comparison(est, events, truth_labels, ids, H, selected, ...
        matched_truth_3d, colors, show_ref);
end

info = struct('available_ids', ids, 'selected_ids', selected, 'history', H, ...
    'angle_ids', selected(selected_has2), 'enu_ids', selected(selected_has3), ...
    'matched_truth_2d', matched_truth_2d, 'matched_truth_3d', matched_truth_3d);
end

function plot_enu_comparison(est, events, truth_labels, ids, H, selected, ...
        matched_truth, colors, show_ref)
names = {'East (m)', 'North (m)', 'Up (m)'};
figure('Name', 'ENU参考对比', 'Position', [210, 190, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        val = h.pos(ax, :); val(~valid) = NaN;
        plot(h.t, val, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d 滤波', selected(s)));
        if show_ref
            ref = nan(3, numel(h.t));
            ref(:, valid) = position_reference_for_track(est, events, truth_labels, ...
                h.event_index(valid), selected(s), matched_truth(s));
            plot(h.t, ref(ax, :), '--', 'Color', colors(s, :), 'LineWidth', 1.0, ...
                'DisplayName', sprintf('Track %d 参考', selected(s)));
        end
    end
    ylabel(names{ax}); if ax == 1, title('三维ENU滤波与参考量测'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
if ~show_ref, return; end

figure('Name', 'ENU误差', 'Position', [240, 215, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        ref = nan(3, numel(h.t));
        ref(:, valid) = position_reference_for_track(est, events, truth_labels, ...
            h.event_index(valid), selected(s), matched_truth(s));
        err = nan(1, numel(h.t));
        err(valid) = h.pos(ax, valid) - ref(ax, valid);
        plot(h.t, err, '-', 'Color', colors(s, :), ...
            'LineWidth', 1.2, 'DisplayName', sprintf('Track %d', selected(s)));
    end
    yline(0, 'k:'); ylabel([names{ax}, '误差']);
    if ax == 1, title('三维ENU误差'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
end

function xyz = associated_range_positions(est, id)
xyz = zeros(3, 0);
if ~isfield(est, 'assoc'), return; end
for k = 1:numel(est.assoc)
    a = est.assoc{k};
    if isempty(a) || ~isfield(a, 'id') || ~isfield(a, 'xyz'), continue; end
    j = find(a.id == id & all(isfinite(a.xyz), 1));
    if ~isempty(j), xyz = [xyz, a.xyz(:, j)]; end %#ok<AGROW>
end
end

function valid = angle_mode_mask(h)
valid = h.dim == 2 & isfinite(h.az) & isfinite(h.el);
end

function [az, el] = angle_mode_series(h)
valid = angle_mode_mask(h);
az = h.az; el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_wrap(az, el);
end

function valid = position_mode_mask(h)
valid = h.dim == 3 & all(isfinite(h.pos), 1);
end

function pos = position_mode_series(h)
valid = position_mode_mask(h);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, H] = collect_history(est)
ids = zeros(1, 0);
H = repmat(struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0), 'event_index', [], 'truth_id', []), 0, 1);
for k = 1:numel(est.output)
    out = est.output{k};
    for q = 1:numel(out)
        i = find(ids == out(q).id, 1);
        if isempty(i)
            ids(end + 1) = out(q).id; %#ok<AGROW>
            H(end + 1, 1) = struct('t', [], 'az', [], 'el', [], 'dim', [], ...
                'pos', zeros(3, 0), 'event_index', [], 'truth_id', []); %#ok<AGROW>
            i = numel(ids);
        end
        H(i).t(end + 1) = out(q).t_sec; H(i).az(end + 1) = out(q).az_deg;
        H(i).el(end + 1) = out(q).el_deg; H(i).dim(end + 1) = out(q).output_dim;
        H(i).pos(:, end + 1) = out(q).position_enu; H(i).event_index(end + 1) = k;
        H(i).truth_id(end + 1) = out(q).truth_id;
    end
end
end

function ref = angle_reference_for_track(est, events, labels, event_idx, id, truth_id)
ref = nan(2, numel(event_idx));
for q = 1:numel(event_idx)
    k = event_idx(q); Z = zeros(2, 0);
    if k <= numel(events) && isfinite(truth_id)
        ia = find(labels.active{k} == truth_id);
        ip = find(labels.passive{k} == truth_id);
        if ~isempty(ia), Z = [Z, events(k).active.rae(2:3, ia)]; end %#ok<AGROW>
        if ~isempty(ip), Z = [Z, events(k).passive.ang(:, ip)]; end %#ok<AGROW>
    end
    if isempty(Z) && isfield(est, 'assoc') && k <= numel(est.assoc) && ...
            ~isempty(est.assoc{k})
        a = est.assoc{k}; j = find(a.id == id);
        if ~isempty(j), Z = a.ang(:, j); end
    end
    if ~isempty(Z), ref(:, q) = [circular_mean(Z(1, :)); mean(Z(2, :))]; end
end
end

function ref = position_reference_for_track(est, events, labels, event_idx, id, truth_id)
ref = nan(3, numel(event_idx));
for q = 1:numel(event_idx)
    k = event_idx(q); Z = zeros(3, 0);
    if k <= numel(events) && isfinite(truth_id)
        j = find(events(k).active.has_range & labels.active{k} == truth_id);
        if ~isempty(j), Z = events(k).active.xyz(:, j); end
    end
    if isempty(Z) && isfield(est, 'assoc') && k <= numel(est.assoc) && ...
            ~isempty(est.assoc{k})
        a = est.assoc{k}; j = find(a.id == id & all(isfinite(a.xyz), 1));
        if ~isempty(j), Z = a.xyz(:, j); end
    end
    if ~isempty(Z), ref(:, q) = mean(Z, 2); end
end
end

function truth = truth_label_for_track(est, events, labels, id, dimension)
votes = zeros(1, 0);
if ~isfield(est, 'assoc'), truth = NaN; return; end
K = min([numel(est.assoc), numel(events), numel(labels.active)]);
for k = 1:K
    a = est.assoc{k};
    if isempty(a) || ~isfield(a, 'id'), continue; end
    for q = find(a.id == id)
        type = indexed_text(a, 'type', q, '');
        mi = round(indexed_number(a, 'meas_index', q, 0));
        if mi < 1, continue; end
        if strncmp(type, 'active', 6) && mi <= numel(labels.active{k})
            label = labels.active{k}(mi);
            is_range = mi <= numel(events(k).active.has_range) && ...
                events(k).active.has_range(mi);
            measurement_dim = 2 + double(is_range);
        elseif strncmp(type, 'passive', 7) && mi <= numel(labels.passive{k})
            label = labels.passive{k}(mi);
            measurement_dim = 2;
        else
            label = NaN;
            measurement_dim = 0;
        end
        if measurement_dim == dimension && isfinite(label)
            votes(end + 1) = label; %#ok<AGROW>
        end
    end
end
if isempty(votes), truth = NaN; else, truth = mode(votes); end
end

function value = indexed_number(s, name, index, fallback)
value = fallback;
if isfield(s, name) && index >= 1 && index <= numel(s.(name)) && ...
        isfinite(s.(name)(index))
    value = s.(name)(index);
end
end

function value = indexed_text(s, name, index, fallback)
value = fallback;
if isfield(s, name) && iscell(s.(name)) && index >= 1 && ...
        index <= numel(s.(name)) && ischar(s.(name){index})
    value = s.(name){index};
end
end

function y = unwrap_for_plot(x)
x = x(:).';
y = nan(size(x));
good = isfinite(x);
edges = diff([false, good, false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
for q = 1:numel(starts)
    j = starts(q):stops(q);
    y(j) = rad2deg(unwrap(deg2rad(x(j))));
end
end

function [az, el] = break_wrap(az0, el0)
az = []; el = [];
for k = 1:numel(az0)
    if k > 1 && abs(az0(k) - az0(k - 1)) > 180
        az(end + 1) = NaN; el(end + 1) = NaN; %#ok<AGROW>
    end
    az(end + 1) = az0(k); el(end + 1) = el0(k); %#ok<AGROW>
end
end

function y = circular_mean(x)
y = atan2d(mean(sind(x)), mean(cosd(x)));
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function s = axis_name(ax)
if ax == 1, s = '方位角 (deg)'; else, s = '俯仰角 (deg)'; end
end

function v = get_opt(opts, name, fallback)
if isfield(opts, name) && ~isempty(opts.(name)), v = opts.(name); else, v = fallback; end
end
