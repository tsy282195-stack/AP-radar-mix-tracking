%% run_fusion_main.m
% 主被动雷达融合跟踪系统 - 主入口脚本
%
% 流程:
%   1. 加载配置
%   2. 解析主动/被动/平台数据
%   3. 空时凝聚
%   4. 构造量测驱动型主被动异步事件
%   5. 自适应CKF滤波
%   6. 定量评价与可视化

clear; close all;
fprintf('╔════════════════════════════════════════════════════╗\n');
fprintf('║   主被动异构融合跟踪系统  - IMM-KF/CKF滤波        ║\n');
fprintf('╚════════════════════════════════════════════════════╝\n\n');

%% ── 第1步：加载配置 ───────────────────────────────────────────────────
cfg = config_fusion();
cfg = validate_config_fusion(cfg);

fprintf('[配置] 请确认 config_fusion.m 中的文件路径和列号参数正确\n');
fprintf('  - 主动雷达文件: %d 个\n', numel(cfg.active_files));
fprintf('  - 被动雷达文件: %d 个\n', numel(cfg.passive_files));
fprintf('  - 平台文件: %s\n', cfg.platform_file);
fprintf('  - 读取百分比范围: %.1f%% ~ %.1f%%\n', cfg.read_start_percent, cfg.read_end_percent);
fprintf('  - 处理框架: %s\n', cfg.processing_framework);

%% ── 第2步：解析数据 ───────────────────────────────────────────────────
fprintf('\n========== 数据解析 ==========\n');

[active_list, passive_list, load_info] = load_radar_measurement_files(cfg);
fprintf('  文件解析模式: %s, 耗时 %.2f s\n', load_info.mode, load_info.elapsed_s);

% Platform interpolation must cover the selected radar interval. Percentage
% windows are meaningful for radar rows, not for an independently sampled
% navigation file, so always load the complete platform timeline here.
platform_cfg = cfg;
platform_cfg.read_start_percent = 0;
platform_cfg.read_end_percent = 100;
platform_cfg.read_percent = 100;
platform_cfg.max_rows = inf;
platform_cfg.time_range_s = [];
platform = load_platform_txt( ...
    resolve_data_path(cfg.data_dir, cfg.platform_file), platform_cfg);
assert_platform_covers_measurements(platform, active_list, passive_list);

%% ── 第3步：空时凝聚 ───────────────────────────────────────────────────
fprintf('\n========== 空时凝聚 ==========\n');
[frames, coherence_info] = cohere_measurements(active_list, passive_list, platform, cfg);
fprintf('共 %d 帧\n', numel(frames));
if isempty(frames)
    fprintf('无有效量测帧，流程结束。请检查文件路径、列号、有效标识和读取比例。\n');
    return;
end

%% ── 第4步：滤波输入构造 ───────────────────────────────────────────────
fprintf('\n========== 滤波输入构造 ==========\n');
fprintf('\n========== 异步异构量测事件输入 ==========\n');
joint_events = [];
if strcmp(cfg.processing_framework, 'joint_2d3d')
    [joint_events, fusion_info, legacy_input] = build_joint_measurement_events(frames, cfg);
    fused_xyz = legacy_input.xyz;
    fused_R = legacy_input.R;
    fused_ids = legacy_input.ids;
    passive_bearing = legacy_input.passive_bearing;
    frame_times = legacy_input.times;
    event_meta = legacy_input.event_meta;
else
    if isempty(active_list)
        error('legacy_active3d 框架不支持主动量测为空，请改用 joint_2d3d。');
    end
    [fused_xyz, fused_R, fusion_info, passive_bearing, fused_ids, frame_times, event_meta] = ...
        build_async_measurement_events(frames, cfg);
end
fusion_info.preprocessing = coherence_info;

%% ── 第5步：统一IMM-KF/CKF滤波 ─────────────────────────────────────────
fprintf('\n========== 统一IMM-KF/CKF滤波 ==========\n');
tic;
if strcmp(cfg.processing_framework, 'joint_2d3d')
    est = run_filter_joint_2d3d(joint_events, platform, cfg);
    n_active_accounted = coherence_info.n_active_condensed + ...
        fusion_info.n_active_duplicates + est.stats.active_measurements;
    n_passive_accounted = fusion_info.n_passive_duplicates + est.stats.passive_measurements;
    if n_active_accounted ~= coherence_info.n_active_input || ...
            n_passive_accounted ~= coherence_info.n_passive_input
        error('run_fusion_main:InputAccountingMismatch', ...
            '原始量测至滤波输入的凝聚/去重计数不守恒。');
    end
    fprintf(['  [主动输入去向] 原始=%d = 凝聚归并%d + 跨文件去重%d + ' ...
        '关联%d + 新生%d + 明确抑制%d\n'], ...
        coherence_info.n_active_input, coherence_info.n_active_condensed, ...
        fusion_info.n_active_duplicates, est.stats.active_assigned, ...
        est.stats.active_births, est.stats.active_birth_suppressed);
    fprintf(['  [被动输入去向] 原始=%d = 跨文件去重%d + ' ...
        '关联%d + 新生%d + 明确抑制%d\n'], ...
        coherence_info.n_passive_input, fusion_info.n_passive_duplicates, ...
        est.stats.passive_assigned, est.stats.passive_births, ...
        est.stats.passive_birth_suppressed);
    fprintf('  逐量测去向: est.measurement_disposition，索引与joint_events内量测一一对应。\n');
else
    est = run_filter_adapt_ckf(fused_xyz, fused_R, frame_times, cfg, ...
        passive_bearing, platform, fused_ids, event_meta);
end
elapsed = toc;
fprintf('总耗时: %.2f s\n', elapsed);

%% ── 第6步：定量评价指 标 ───────────────────────────────────────────────
metrics_info = [];
if ~isfield(cfg, 'metrics_enabled') || cfg.metrics_enabled
    fprintf('\n========== 定量评价 ==========\n');
    metrics_start = tic;
    try
        if strcmp(cfg.processing_framework, 'joint_2d3d')
            metrics_info = evaluate_joint_tracking_metrics(est, joint_events, cfg, platform);
        else
            metrics_info = evaluate_fusion_quant_metrics(frames, fused_xyz, fused_ids, ...
                est, frame_times, cfg);
        end
    catch ME
        error('run_fusion_main:MetricsFailed', ...
            '定量评价指标计算失败，已停止保存正式结果: %s', ME.message);
    end
    fprintf('定量评价耗时: %.2f s\n', toc(metrics_start));
end

if cfg.do_plot
    fprintf('\n========== 可视化 ==========\n');

    if strcmp(cfg.processing_framework, 'joint_2d3d')
        plot_joint_tracking_results(est, joint_events, cfg);
    else

    % ── 收集全部主动量测点 ──
    all_meas_xyz = [];
    for k = 1:numel(frames)
        if ~isempty(frames(k).active_xyz)
            all_meas_xyz = [all_meas_xyz, frames(k).active_xyz]; %#ok<AGROW>
        end
    end

    % ── 收集全部确认航迹（直接来自 est，不使用缝合） ──
    track_labels = [];
    track_pos = {};
    track_t   = {};
    for k = 1:numel(est.X)
        if est.N(k) > 0
            for i = 1:size(est.L{k}, 1)
                tid = est.L{k}(i, 2);
                pos = est.X{k}([1,4,7], i);
                idx = find(track_labels == tid, 1);
                if isempty(idx)
                    track_labels(end+1) = tid;          %#ok<SAGROW>
                    track_pos{end+1} = pos;             %#ok<SAGROW>
                    track_t{end+1}   = frame_times(k);  %#ok<SAGROW>
                else
                    track_pos{idx} = [track_pos{idx}, pos]; %#ok<SAGROW>
                    track_t{idx}   = [track_t{idx}, frame_times(k)]; %#ok<SAGROW>
                end
            end
        end
    end

    suffix = '';

    min_life = 3;
    long_idx = find(cellfun(@(p) size(p,2) >= min_life, track_pos));
    n_tracks = numel(long_idx);
    track_colors = lines(max(n_tracks, 1));

    %% 图1: 纯主动量测图
    figure('Name', '主动量测点', 'Position', [50, 100, 900, 750]);
    if ~isempty(all_meas_xyz)
        plot3(all_meas_xyz(1,:), all_meas_xyz(2,:), all_meas_xyz(3,:), ...
              '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4);
    end
    title(sprintf('主动量测点 (共%d点)', size(all_meas_xyz,2)));
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    grid on; axis equal; view(45, 30);

    %% 图2: 3D航迹 + 量测背景
    figure('Name', '跟踪航迹 3D', 'Position', [100, 100, 1000, 800]);

    if ~isempty(all_meas_xyz)
        plot3(all_meas_xyz(1,:), all_meas_xyz(2,:), all_meas_xyz(3,:), ...
              '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 3);
    end
    hold on;

    h_leg = zeros(1, n_tracks);
    leg_str = cell(1, n_tracks);
    for t = 1:n_tracks
        idx = long_idx(t);
        pos_seq = track_pos{idx};
        col = track_colors(t, :);
        h_leg(t) = plot3(pos_seq(1, :), pos_seq(2, :), pos_seq(3, :), ...
                         '.-', 'Color', col, 'LineWidth', 0.6, 'MarkerSize', 3);
        plot3(pos_seq(1, 1), pos_seq(2, 1), pos_seq(3, 1), ...
              'o', 'Color', col, 'MarkerFaceColor', col, 'MarkerSize', 4);
        % 在轨迹起点附近标上目标编号
        text(pos_seq(1, 1), pos_seq(2, 1), pos_seq(3, 1), ...
             sprintf(' est%d', track_labels(idx)), ...
             'Color', col, 'FontSize', 8, 'FontWeight', 'bold', ...
             'VerticalAlignment', 'bottom');
        leg_str{t} = sprintf('est%d', track_labels(idx));
    end
    if n_tracks > 0
        legend(h_leg, leg_str, 'Location', 'bestoutside');
    end
    title(sprintf('3D跟踪航迹 (>=%d帧, 共%d条%s)', min_life, n_tracks, suffix));
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    grid on; axis equal; view(45, 30);
    hold off;

    %% 图3: 2D俯视图 + 量测背景
    figure('Name', '跟踪航迹 俯视图', 'Position', [150, 150, 900, 700]);

    if ~isempty(all_meas_xyz)
        plot(all_meas_xyz(1,:), all_meas_xyz(2,:), ...
             '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 3);
    end
    hold on;

    h_leg2 = zeros(1, n_tracks);
    leg_str2 = cell(1, n_tracks);
    for t = 1:n_tracks
        idx = long_idx(t);
        pos_seq = track_pos{idx};
        col = track_colors(t, :);
        h_leg2(t) = plot(pos_seq(1, :), pos_seq(2, :), ...
                         '.-', 'Color', col, 'LineWidth', 0.6, 'MarkerSize', 3);
        plot(pos_seq(1, 1), pos_seq(2, 1), ...
             'o', 'Color', col, 'MarkerFaceColor', col, 'MarkerSize', 4);
        % 在轨迹起点附近标上目标编号
        text(pos_seq(1, 1), pos_seq(2, 1), ...
             sprintf(' est%d', track_labels(idx)), ...
             'Color', col, 'FontSize', 8, 'FontWeight', 'bold', ...
             'VerticalAlignment', 'bottom');
        leg_str2{t} = sprintf('est%d', track_labels(idx));
    end
    if n_tracks > 0
        legend(h_leg2, leg_str2, 'Location', 'bestoutside');
    end
    title(sprintf('俯视图 (>=%d帧, 共%d条%s)', min_life, n_tracks, suffix));
    xlabel('East (m)'); ylabel('North (m)');
    grid on; axis equal;
    hold off;

    %% 图4: 航迹统计
    figure('Name', '航迹统计', 'Position', [200, 200, 800, 400]);
    subplot(1, 2, 1);
    plot(frame_times, est.N, 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('确认航迹数');
    title('每帧确认航迹数量');
    grid on;

    subplot(1, 2, 2);
    meas_per_frame = cellfun(@(x) size(x, 2), fused_xyz);
    plot(frame_times, meas_per_frame, 'r-', 'LineWidth', 1);
    xlabel('Time (s)'); ylabel('量测数');
    title('每帧融合量测数量');
    grid on;

    %% 图5: 估计航迹长度（帧）条形图
    figure('Name', '航迹长度(帧)', 'Position', [250, 250, 950, 470]);
    if isempty(track_labels)
        axis off;
        text(0.5, 0.5, '无确认航迹', 'HorizontalAlignment', 'center', 'FontSize', 12);
    else
        track_len = cellfun(@(p) size(p, 2), track_pos);   % 每条航迹存在的帧数
        [len_sorted, ord] = sort(track_len, 'descend');    % 按长度降序
        lab_sorted = track_labels(ord);
        ntot = numel(len_sorted);
        xpos = 1:ntot;
        is_long = len_sorted >= min_life;                  % 是否达到轨迹绘制阈值

        hold on;
        hL = []; lstr = {};
        if any(is_long)
            h = bar(xpos(is_long), len_sorted(is_long), 0.7, ...
                    'FaceColor', [0.20 0.45 0.75], 'EdgeColor', 'none');
            hL(end+1) = h; lstr{end+1} = sprintf('\\geq%d帧', min_life);
        end
        if any(~is_long)
            h = bar(xpos(~is_long), len_sorted(~is_long), 0.7, ...
                    'FaceColor', [0.78 0.78 0.78], 'EdgeColor', 'none');
            hL(end+1) = h; lstr{end+1} = '偏短';
        end

        % min_life 参考线（即3D/俯视图里绘制航迹的帧数阈值）
        plot([0.5, ntot + 0.5], [min_life, min_life], 'r--', 'LineWidth', 1);

        % 航迹不多时在柱顶标注帧数
        if ntot <= 40
            for b = 1:ntot
                text(b, len_sorted(b), sprintf('%d', len_sorted(b)), ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                     'FontSize', 7);
            end
        end

        set(gca, 'XTick', 1:ntot, 'XTickLabel', ...
            arrayfun(@(g) sprintf('est%d', g), lab_sorted, 'UniformOutput', false));
        try
            xtickangle(45);
        catch
        end
        xlim([0.5, ntot + 0.5]);
        xlabel('航迹编号'); ylabel('航迹长度 (帧)');
        title(sprintf('估计航迹长度 (共%d条, \\geq%d帧: %d条%s)', ...
              ntot, min_life, sum(is_long), suffix));
        grid on;
        if ~isempty(hL), legend(hL, lstr, 'Location', 'northeast'); end
        hold off;
    end

    fprintf('可视化完成。\n');
    if ~isempty(cfg.plot_save_dir)
        save_all_open_figures(cfg.plot_save_dir);
    end
    end
end

%% ── 第8步：二次平滑（在线/因果，可选；不改滤波器）─────────────────────
smt = [];
if isfield(cfg, 'do_smoothing') && cfg.do_smoothing && ...
        strcmp(cfg.processing_framework, 'legacy_active3d')
    fprintf('\n========== 二次平滑 ==========\n');
    so = struct();
    if isfield(cfg, 'smooth_method'),   so.method   = cfg.smooth_method;   end
    if isfield(cfg, 'smooth_lag'),      so.lag      = cfg.smooth_lag;      end
    if isfield(cfg, 'smooth_min_life'), so.min_life = cfg.smooth_min_life; end
    if isfield(cfg, 'smooth_meas_std'), so.meas_std = cfg.smooth_meas_std; end
    if isfield(cfg, 'smooth_proc_std'), so.proc_std = cfg.smooth_proc_std; end
    try
        smt = smooth_filter_tracks(est, frame_times, so);
        if cfg.do_plot
            po = struct();
            if isfield(cfg, 'smooth_show_axes'), po.show_axes = cfg.smooth_show_axes; end
            if exist('all_meas_xyz', 'var') && ~isempty(all_meas_xyz)
                po.bg_xyz = all_meas_xyz;
            end
            if isfield(cfg, 'plot_save_dir') && ~isempty(cfg.plot_save_dir)
                po.save_dir = cfg.plot_save_dir;
            end
            plot_track_smoothing(smt, po);
        end
    catch ME
        warning('run_fusion_main:SmoothingFailed', ...
            '二次平滑失败: %s', ME.message);
    end
end

%% ── 结果汇总 ──────────────────────────────────────────────────────────
fprintf('\n╔════════════════════════════════════════════════════╗\n');
fprintf('║                    结果汇总                        ║\n');
fprintf('╠════════════════════════════════════════════════════╣\n');
fprintf('║  总帧数:          %6d                          ║\n', numel(frames));
fprintf('║  主动量测总数:    %6d                          ║\n', fusion_info.n_active);
fprintf('║  被动量测总数:    %6d                          ║\n', fusion_info.n_passive);
fprintf('║  异步事件总数:    %6d                          ║\n', fusion_info.n_events);
fprintf('║  被动bearing量测: %6d                          ║\n', fusion_info.n_passive_bearing);
if isfield(est, 'N_total')
    fprintf('║  逻辑航迹输出数:  %6d (跨所有事件)            ║\n', sum(est.N_total));
    fprintf('║  二维/三维输出:   %6d / %-6d                  ║\n', sum(est.N2), sum(est.N));
else
    fprintf('║  确认航迹总数:    %6d (跨所有帧)              ║\n', sum(est.N));
end
fprintf('║  滤波耗时:        %6.1f s                       ║\n', elapsed);
fprintf('╚════════════════════════════════════════════════════╝\n');

if exist('metrics_info', 'var') && ~isempty(metrics_info) && isstruct(metrics_info)
    fprintf('\n定量评价指标汇总:\n');
    if isfield(metrics_info, 'real_truth')
        R = metrics_info.real_truth;
        if strcmp(get_struct_field(R, 'status', ''), 'ok')
            fprintf(['  [真实精度] 目标=%d, 输出时间匹配=%d/%d, ', ...
                '目标覆盖=%d/%d\n'], R.n_truth_targets, ...
                R.n_time_matched_outputs, R.n_identity_mapped_outputs, ...
                R.n_output_truth_targets, R.n_truth_targets);
            if R.three_d.position.n > 0
                fprintf('             三维位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
                    R.three_d.position.rmse_e_m, R.three_d.position.rmse_n_m, ...
                    R.three_d.position.rmse_u_m, R.three_d.position.rmse_3d_m, ...
                    R.three_d.position.n);
            end
            if R.two_d.angle.n > 0
                fprintf('             二维角度RMSE: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
                    R.two_d.angle.rmse_az_deg, R.two_d.angle.rmse_el_deg, ...
                    R.two_d.angle.rmse_los_deg, R.two_d.angle.n);
            end
        else
            fprintf('  [真实精度] 不可用: %s (%s)\n', ...
                get_struct_field(R, 'status', 'unavailable'), ...
                get_struct_field(R, 'reason', 'truth_not_available'));
        end
    end
    if isfield(metrics_info, 'truth_targets')
        truth_count = metrics_info.truth_targets;
    elseif isfield(metrics_info, 'id_split')
        truth_count = metrics_info.id_split;
    else
        truth_count = [];
    end
    if ~isempty(truth_count) && isfield(truth_count, 'raw_id_count') && ...
            isfield(truth_count, 'instance_count')
        fprintf('  [量测伪真值一致性] 原始标签=%d, 拆分后=%d, 被拆分原始编号=%d\n', ...
            truth_count.raw_id_count, truth_count.instance_count, ...
            get_struct_field(truth_count, 'n_split_raw_ids', 0));
    end
    if isfield(metrics_info, 'two_d') && isfield(metrics_info, 'three_d') && ...
            isfield(metrics_info, 'overall')
        D2 = metrics_info.two_d;
        D3 = metrics_info.three_d;
        OA = metrics_info.overall;
        fprintf(['  [二维] 输出=%d点/%d轨, 量测关联率=%.2f%%, 关联点一致率=%.2f%%, ', ...
            '航迹正确率=%.2f%%\n'], D2.output.n_outputs, D2.output.n_unique_tracks, ...
            100 * D2.association.rate_all_tracks, 100 * D2.accuracy.accuracy, ...
            100 * D2.track_accuracy.accuracy_vs_output);
        if D2.angle.n > 0
            fprintf('         角度RMSE: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
                D2.angle.rmse_az_deg, D2.angle.rmse_el_deg, ...
                D2.angle.rmse_los_deg, D2.angle.n);
        end
        fprintf('         起始延迟: 最早=%.3fs, 主航迹=%.3fs (%d/%d目标)\n', ...
            D2.start_time.mean_track_start_delay_s, ...
            D2.start_time.mean_main_track_start_delay_s, ...
            D2.start_time.n_started, D2.start_time.n_truth);
        fprintf('         正式输出同步覆盖率=%.2f%% (%d/%d带标签物理量测点)\n', ...
            100 * D2.output_coverage.rate, D2.output_coverage.n_covered_measurements, ...
            D2.output_coverage.n_reference_measurements);

        fprintf(['  [三维] 输出=%d点/%d轨, 量测关联率=%.2f%%, 关联点一致率=%.2f%%, ', ...
            '航迹正确率=%.2f%%\n'], D3.output.n_outputs, D3.output.n_unique_tracks, ...
            100 * D3.association.rate_all_tracks, 100 * D3.accuracy.accuracy, ...
            100 * D3.track_accuracy.accuracy_vs_output);
        if D3.position.n > 0
            fprintf('         位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
                D3.position.rmse_e_m, D3.position.rmse_n_m, D3.position.rmse_u_m, ...
                D3.position.rmse_3d_m, D3.position.n);
        end
        fprintf('         起始延迟: 最早=%.3fs, 主航迹=%.3fs (%d/%d目标)\n', ...
            D3.start_time.mean_track_start_delay_s, ...
            D3.start_time.mean_main_track_start_delay_s, ...
            D3.start_time.n_started, D3.start_time.n_truth);
        fprintf('         正式输出同步覆盖率=%.2f%% (%d/%d带标签物理量测点)\n', ...
            100 * D3.output_coverage.rate, D3.output_coverage.n_covered_measurements, ...
            D3.output_coverage.n_reference_measurements);

        fprintf(['  [总体] 输出=%d点/%d轨, 关联率(全部/曾确认)=%.2f%%/%.2f%%, ', ...
            '关联点一致率=%.2f%%, 航迹正确率=%.2f%%\n'], ...
            OA.output.n_outputs, OA.output.n_unique_tracks, ...
            100 * OA.association.rate_all_tracks, ...
            100 * OA.association.rate_confirmed_tracks, ...
            100 * OA.accuracy.accuracy, ...
            100 * OA.track_accuracy.accuracy_vs_output);
        fprintf('         正式输出同步覆盖率=%.2f%% (%d/%d带标签物理量测点)\n', ...
            100 * OA.output_coverage.rate, OA.output_coverage.n_covered_measurements, ...
            OA.output_coverage.n_reference_measurements);
        if isfield(metrics_info, 'measurement_accounting') && ...
                isfield(metrics_info.measurement_accounting, 'passive')
            P = metrics_info.measurement_accounting.passive;
            fprintf(['  [物理被动量测] 输入=%d, 去2D=%d, 去3D=%d, 关联后未保留=%d, ' ...
                '明确抑制=%d, 未解释=%d\n'], P.n_input, P.n_to_2d, P.n_to_3d, ...
                P.n_associated_not_retained, P.n_explicitly_suppressed, P.unaccounted);
        end
    else
        if isfield(metrics_info, 'association')
            fprintf('  滤波关联率(全部/确认): %.2f%% / %.2f%%\n', ...
                100 * metrics_info.association.rate_all_tracks, ...
                100 * metrics_info.association.rate_confirmed_tracks);
        end
        if isfield(metrics_info, 'angle') && metrics_info.angle.n > 0
            fprintf('  二维角度RMSE: az=%.4f deg, el=%.4f deg, LOS=%.4f deg (%d点)\n', ...
                metrics_info.angle.rmse_az_deg, metrics_info.angle.rmse_el_deg, ...
                metrics_info.angle.rmse_los_deg, metrics_info.angle.n);
        end
        if isfield(metrics_info, 'accuracy_raw') && isfield(metrics_info, 'accuracy_split')
            fprintf('  关联点一致率(原始编号/拆分实例): %.2f%% / %.2f%%\n', ...
                100 * metrics_info.accuracy_raw.accuracy, ...
                100 * metrics_info.accuracy_split.accuracy);
        end
        if isfield(metrics_info, 'track_accuracy_split')
            TS = metrics_info.track_accuracy_split;
            fprintf(['  航迹级正确率_比滤波输出航迹数: %.2f%%  正确航迹数=%d, 滤波输出总航迹数=%d, ', ...
                '可评价航迹数=%d, 多余/错误航迹数=%d, 拆分实例数=%d\n'], ...
                100 * TS.accuracy_vs_output, TS.n_correct_tracks, TS.n_output_tracks, ...
                TS.n_evaluable_tracks, TS.n_error_or_extra_tracks, TS.n_truth_reference);
            fprintf(['  航迹级正确率_比参考伪真值: %.2f%%  正确航迹数=%d, 滤波输出总航迹数=%d, ', ...
                '可评价航迹数=%d, 多余/错误航迹数=%d, 拆分实例数=%d\n'], ...
                100 * TS.accuracy_vs_truth, TS.n_correct_tracks, TS.n_output_tracks, ...
                TS.n_evaluable_tracks, TS.n_error_or_extra_tracks, TS.n_truth_reference);
        end
        if isfield(metrics_info, 'start_time')
            T = metrics_info.start_time;
            fprintf(['  航迹起始: 最早确认碎片=%d/%d, 平均航迹起始延迟=%.3fs; ', ...
                '一对一主航迹确认=%d/%d, 平均主航迹确认起始延迟=%.3fs\n'], ...
                T.n_started, T.n_truth, T.mean_track_start_delay_s, ...
                T.n_main_confirmed, T.n_truth, T.mean_main_track_start_delay_s);
            if isfield(T, 'n_paired') && T.n_paired > 0
                fprintf(['  航迹起始同集对比(%d条): 最早碎片=%.3fs, 主航迹=%.3fs, ', ...
                    '主航迹相对晚=%.3fs, 违例=%d\n'], ...
                    T.n_paired, T.mean_track_start_delay_paired_s, ...
                    T.mean_main_track_start_delay_paired_s, ...
                    T.mean_main_minus_first_paired_s, T.n_start_order_violations);
            end
        end
        if isfield(metrics_info, 'rms')
            fprintf('  伪真值RTS平滑三维RMSE: %.2f m\n', metrics_info.rms.rmse_3d);
        end
    end
end

if isfield(cfg, 'result_save_file') && ~isempty(cfg.result_save_file)
    [result_dir, ~, ~] = fileparts(cfg.result_save_file);
    if ~isempty(result_dir) && exist(result_dir, 'dir') ~= 7
        mkdir(result_dir);
    end
    save(cfg.result_save_file, 'cfg', 'frames', 'fused_xyz', 'fused_R', 'fused_ids', ...
        'passive_bearing', 'fusion_info', 'frame_times', 'est', ...
        'metrics_info', 'smt', 'event_meta', 'joint_events', '-v7.3');
    fprintf('结果已保存: %s\n', cfg.result_save_file);
end

fprintf('\n完成！\n');

function value = get_struct_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end
