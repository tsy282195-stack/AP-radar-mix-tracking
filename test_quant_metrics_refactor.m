function test_quant_metrics_refactor()
%TEST_QUANT_METRICS_REFACTOR Synthetic checks for the revised metrics.

K = 5;
frame_times = (0:K-1).';
fused_xyz = cell(K, 1);
fused_ids = cell(K, 1);
for k = 1:K
    fused_xyz{k} = [100 * (k - 1); 0; 0];
    fused_ids{k} = 1;
end

est = struct();
est.X = cell(K, 1);
est.L = cell(K, 1);
est.assoc = cell(K, 1);
for k = 1:K
    est.assoc{k} = struct('id', 10, 'xyz', fused_xyz{k}, 'tid', 1);
    if k >= 3
        x = zeros(9, 1);
        x([1, 4, 7]) = fused_xyz{k};
        est.X{k} = x;
        est.L{k} = [1, 10];
    else
        est.X{k} = [];
        est.L{k} = [];
    end
end

% Track 20 is a confirmed output with no labeled association. It must be
% included in the output-track denominator and counted as error/extra.
x20 = zeros(9, 1);
x20([1, 4, 7]) = [5000; 5000; 0];
est.X{K} = [est.X{K}, x20];
est.L{K} = [est.L{K}; 5, 20];

cfg = struct();
cfg.metrics_enabled = true;
cfg.metrics_max_print = 3;
cfg.frame_time_window_s = 1;
cfg.track_accuracy_purity_th = 0.8;
cfg.track_accuracy_min_assoc = 3;
cfg.truth_id_split_enabled = true;
cfg.truth_id_split_dist_m = 10000;
cfg.truth_id_split_max_gap_s = 30;
cfg.truth_assoc_map_max_dist_m = inf;
cfg.truth_rts_meas_std_m = 10;
cfg.truth_rts_proc_std_mps2 = 1;
cfg.truth_rms_time_tolerance_s = 1;

metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
ta = metrics.track_accuracy_split;
st = metrics.start_time;

% 确认航迹关联率应回溯包含该ID在确认前的出生/试探关联点；确认时刻仍为首次输出帧。
assert(metrics.association.n_assoc_all_tracks == K);
assert(metrics.association.n_assoc_confirmed_tracks == K);
assert(abs(metrics.association.rate_confirmed_tracks - 1) < 1e-12);
assert(metrics.track_timeout.n_total == 0);

assert(ta.n_correct_tracks == 1);
assert(ta.n_output_tracks == 2);
assert(ta.n_evaluable_tracks == 1);
assert(ta.n_error_or_extra_tracks == 1);
assert(ta.n_truth_reference == 1);
assert(abs(ta.accuracy_vs_output - 0.5) < 1e-12);
assert(abs(ta.accuracy_vs_truth - 1.0) < 1e-12);

assert(st.n_started == 1 && st.n_main_confirmed == 1);
assert(abs(st.mean_track_start_delay_s - 2) < 1e-12);
assert(abs(st.mean_main_track_start_delay_s - 2) < 1e-12);
assert(abs(st.records(1).first_confirmed_time - 2) < 1e-12);
assert(abs(st.records(1).main_track_confirmed_time - 2) < 1e-12);

assert(~isfield(metrics, 'duplicate_raw'));
assert(~isfield(metrics, 'duplicate_split'));
assert(~isfield(st, 'median_confirm_delay_s'));
assert(~isfield(st.records, 'first_capture_time'));

% 航迹纯度分母必须是匹配伪真值的全部量测数，而不是该航迹自身的关联点数。
% 航迹10只关联前3/5个真值量测，虽然这3点全部来自同一目标，纯度仍应为0.6。
est_fragment = est;
for k = 4:K
    est_fragment.assoc{k} = struct('id', zeros(1, 0), ...
        'xyz', zeros(3, 0), 'tid', zeros(1, 0));
end
metrics_fragment = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est_fragment, frame_times, cfg);
ta_fragment = metrics_fragment.track_accuracy_split;
rec10 = ta_fragment.records([ta_fragment.records.track_id] == 10);
assert(numel(rec10) == 1);
assert(abs(rec10.purity - 3/5) < 1e-12);
assert(ta_fragment.n_correct_tracks == 0);

% 起始延迟使用最早有效确认碎片，不使用“整条伪真值覆盖率”门槛。
% 航迹10虽然只覆盖3/5、达不到0.8正确航迹阈值，仍是有效确认碎片。
st_fragment = metrics_fragment.start_time;
assert(st_fragment.n_started == 1);
assert(st_fragment.n_main_confirmed == 1);
assert(st_fragment.n_fragment_candidates == 1);
assert(st_fragment.records(1).first_confirmed_track_id == 10);
assert(st_fragment.records(1).main_track_id == 10);
assert(abs(st_fragment.mean_track_start_delay_s - 2) < 1e-12);
assert(abs(st_fragment.mean_main_track_start_delay_s - 2) < 1e-12);
assert(st_fragment.n_paired == 1);
assert(abs(st_fragment.mean_track_start_delay_paired_s - 2) < 1e-12);
assert(abs(st_fragment.mean_main_track_start_delay_paired_s - 2) < 1e-12);
assert(metrics_fragment.rms.n_tracks == 1);

% 起始碎片不受航迹级正确率的最少3点门槛限制；即使只有2个实际归属点，
% 只要该滤波航迹已经确认输出，仍应参与最早起始计算。
est_two_point_fragment = est_fragment;
est_two_point_fragment.assoc{3} = struct('id', zeros(1, 0), ...
    'xyz', zeros(3, 0), 'tid', zeros(1, 0));
metrics_two_point = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est_two_point_fragment, frame_times, cfg);
assert(metrics_two_point.start_time.n_started == 1);
assert(metrics_two_point.start_time.n_main_confirmed == 1);
assert(metrics_two_point.start_time.n_fragment_candidates == 1);
assert(metrics_two_point.start_time.records(1).main_track_id == 10);
assert(abs(metrics_two_point.start_time.mean_track_start_delay_s - 2) < 1e-12);
assert(abs(metrics_two_point.start_time.mean_main_track_start_delay_s - 2) < 1e-12);
assert(metrics_two_point.track_accuracy_split.n_correct_tracks == 0);

test_earliest_fragment_differs_from_main_track();
test_preconfirmed_fragment_is_skipped();
test_confirmed_fragment_needs_no_purity_or_minimum_hits();
test_main_track_ignores_purity_and_minimum_hits();
test_main_track_assignment_is_global_not_greedy();
fprintf('Quantitative metric refactor synthetic test passed.\n');
end

function test_earliest_fragment_differs_from_main_track()
% 同一伪真值由两条确认航迹组成：短而早的碎片负责起始，覆盖更多量测的
% 后续航迹负责一对一主航迹评价。
K = 10;
frame_times = (0:K-1).';
fused_xyz = cell(K, 1);
fused_ids = cell(K, 1);
est = empty_estimate(K);
for k = 1:K
    p = [100 * (k - 1); 0; 0];
    fused_xyz{k} = p;
    fused_ids{k} = 1;
    if k <= 3
        track_id = 10;
    else
        track_id = 20;
    end
    est.assoc{k} = struct('id', track_id, 'xyz', p, 'tid', 1);

    if k == 3
        [est.X{k}, est.L{k}] = confirmed_output(p, 10);
    elseif k >= 6
        [est.X{k}, est.L{k}] = confirmed_output(p, 20);
    end
end

cfg = metric_cfg(0.7, 3);
metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
st = metrics.start_time;

assert(st.n_started == 1 && st.n_main_confirmed == 1);
assert(st.n_fragment_candidates == 2);
assert(st.n_multi_fragment_truth == 1);
assert(st.records(1).first_confirmed_track_id == 10);
assert(st.records(1).main_track_id == 20);
assert(abs(st.records(1).track_start_delay_s - 2) < 1e-12);
assert(abs(st.records(1).main_track_start_delay_s - 5) < 1e-12);
assert(abs(st.mean_track_start_delay_s - 2) < 1e-12);
assert(abs(st.mean_main_track_start_delay_s - 5) < 1e-12);
assert(st.n_paired == 1);
assert(abs(st.mean_track_start_delay_paired_s - 2) < 1e-12);
assert(abs(st.mean_main_track_start_delay_paired_s - 5) < 1e-12);
assert(abs(st.mean_main_minus_first_paired_s - 3) < 1e-12);
assert(st.n_first_earlier_than_main == 1);
assert(st.n_first_equal_main == 0);
assert(st.n_start_order_violations == 0);
end

function test_preconfirmed_fragment_is_skipped()
% 旧航迹在伪真值出现前已经确认，随后错误吸收该目标量测。该负延迟候选
% 必须被记录并跳过，继续选择起点之后首次确认的有效碎片。
K = 7;
frame_times = (0:K-1).';
fused_xyz = cell(K, 1);
fused_ids = cell(K, 1);
est = empty_estimate(K);
for k = 1:K
    if k < 3
        fused_xyz{k} = zeros(3, 0);
        fused_ids{k} = zeros(1, 0);
        est.assoc{k} = struct('id', zeros(1, 0), ...
            'xyz', zeros(3, 0), 'tid', zeros(1, 0));
    else
        p = [100 * (k - 3); 0; 0];
        fused_xyz{k} = p;
        fused_ids{k} = 1;
        if k <= 4, track_id = 10; else, track_id = 20; end
        est.assoc{k} = struct('id', track_id, 'xyz', p, 'tid', 1);
    end
end
[est.X{2}, est.L{2}] = confirmed_output([-100; 0; 0], 10); % t=1 < truth start t=2
for k = 6:K
    [est.X{k}, est.L{k}] = confirmed_output(fused_xyz{k}, 20);
end

cfg = metric_cfg(0.6, 2);
metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
st = metrics.start_time;

assert(st.n_started == 1 && st.n_main_confirmed == 1);
assert(st.n_fragment_candidates == 2);
assert(st.n_preconfirmed_candidates == 1);
assert(st.records(1).first_confirmed_track_id == 20);
assert(st.records(1).main_track_id == 20);
assert(abs(st.records(1).track_start_delay_s - 3) < 1e-12);
assert(abs(st.records(1).main_track_start_delay_s - 3) < 1e-12);
assert(st.n_paired == 1);
assert(st.n_first_earlier_than_main == 0);
assert(st.n_first_equal_main == 1);
assert(st.n_start_order_violations == 0);

% 如果所有确认碎片都早于伪真值起点，则该伪真值完全不进入起始均值。
est_all_pre = est;
for k = 6:K
    est_all_pre.X{k} = [];
    est_all_pre.L{k} = [];
end
metrics_all_pre = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est_all_pre, frame_times, cfg);
st_all_pre = metrics_all_pre.start_time;
assert(st_all_pre.n_fragment_candidates == 1);
assert(st_all_pre.n_preconfirmed_candidates == 1);
assert(st_all_pre.n_started == 0 && st_all_pre.n_paired == 0);
assert(st_all_pre.n_main_confirmed == 0);
assert(st_all_pre.n_main_preconfirmed == 1);
assert(isnan(st_all_pre.mean_track_start_delay_s));
assert(isnan(st_all_pre.mean_main_track_start_delay_s));
end

function test_confirmed_fragment_needs_no_purity_or_minimum_hits()
% 航迹10对目标1有2个归属点、对目标2只有1个归属点。它已经确认输出，
% 因而是两条伪真值各自的组成碎片；起始评价不使用纯度或最少3点门槛。
K = 3;
frame_times = (0:K-1).';
fused_xyz = {[0; 0; 0], [100; 0; 0], [10000; 0; 0]};
fused_ids = {1, 1, 2};
est = empty_estimate(K);
for k = 1:K
    est.assoc{k} = struct('id', 10, 'xyz', fused_xyz{k}, 'tid', fused_ids{k});
end
[est.X{3}, est.L{3}] = confirmed_output(fused_xyz{3}, 10);

cfg = metric_cfg(0.5, 3);
metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
st = metrics.start_time;

assert(st.n_fragment_candidates == 2);
assert(st.n_started == 2);
assert(st.n_main_confirmed == 1);
assert(abs(st.mean_track_start_delay_s - 1) < 1e-12);
assert(all([st.records.first_confirmed_track_id] == 10));
assert(all([st.records.track_start_delay_s] >= 0));
assert(st.n_paired == 1);
assert(abs(st.mean_main_track_start_delay_s - 2) < 1e-12);
assert(metrics.track_accuracy_split.n_correct_tracks == 0);
end

function test_main_track_ignores_purity_and_minimum_hits()
% 主航迹只由全局一对一最大关联量测数决定，不受纯度和最少点数门槛影响。
% 目标2的主航迹21只覆盖3/10、达不到0.7纯度，但仍必须参与主航迹起始评价。
K = 10;
frame_times = (0:K-1).';
fused_xyz = cell(K, 1);
fused_ids = cell(K, 1);
est = empty_estimate(K);
for k = 1:K
    p1 = [100 * (k - 1); 0; 0];
    p2 = [100 * (k - 1); 10000; 0];
    fused_xyz{k} = [p1, p2];
    fused_ids{k} = [1, 2];
    if k <= 3, id1 = 11; else, id1 = 12; end
    if k >= 8, id2 = 21; else, id2 = 100 + k; end
    est.assoc{k} = struct('id', [id1, id2], 'xyz', [p1, p2], 'tid', [1, 2]);

    if k == 3
        [est.X{k}, est.L{k}] = confirmed_output(p1, 11);
    elseif k >= 6
        [est.X{k}, est.L{k}] = confirmed_output(p1, 12);
    end
end
[x21, L21] = confirmed_output(fused_xyz{10}(:, 2), 21);
est.X{10} = [est.X{10}, x21];
est.L{10} = [est.L{10}; L21];

cfg = metric_cfg(0.7, 3);
metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
st = metrics.start_time;

assert(st.n_started == 2 && st.n_main_confirmed == 2);
assert(abs(st.mean_track_start_delay_s - 5.5) < 1e-12);
assert(abs(st.mean_main_track_start_delay_s - 7) < 1e-12);
assert(st.mean_track_start_delay_s < st.mean_main_track_start_delay_s);
assert(st.n_paired == 2);
assert(abs(st.mean_track_start_delay_paired_s - 5.5) < 1e-12);
assert(abs(st.mean_main_track_start_delay_paired_s - 7) < 1e-12);
assert(abs(st.mean_main_minus_first_paired_s - 1.5) < 1e-12);
assert(st.n_first_earlier_than_main == 1);
assert(st.n_first_equal_main == 1);
assert(st.n_start_order_violations == 0);
assert(metrics.track_accuracy_split.n_correct_tracks == 1);
assert(metrics.rms.n_tracks == 2);
end

function test_main_track_assignment_is_global_not_greedy()
% 关联计数矩阵（行=目标1/2，列=航迹10/20）为 [9 8; 8 0]。
% 逐边贪心会先取9、总计仅9；正确的全局一对一结果应取8+8=16。
K = 17;
frame_times = (0:K-1).';
fused_xyz = cell(K, 1);
fused_ids = cell(K, 1);
est = empty_estimate(K);
for k = 1:K
    p1 = [100 * (k - 1); 0; 0];
    p2 = [100 * (k - 1); 10000; 0];
    fused_xyz{k} = [p1, p2];
    fused_ids{k} = [1, 2];
    if k <= 9
        id1 = 10;
    else
        id1 = 20;
    end
    if k >= 10
        id2 = 10;
    else
        id2 = 100 + k;
    end
    est.assoc{k} = struct('id', [id1, id2], 'xyz', [p1, p2], 'tid', [1, 2]);
end
[x10, L10] = confirmed_output(fused_xyz{2}(:, 1), 10);
est.X{2} = x10;
est.L{2} = L10;
[x20, L20] = confirmed_output(fused_xyz{11}(:, 1), 20);
est.X{11} = x20;
est.L{11} = L20;

cfg = metric_cfg(1.0, 20); % 故意让两条航迹都不满足正确率门槛。
metrics = evaluate_fusion_quant_metrics([], fused_xyz, fused_ids, ...
    est, frame_times, cfg);
ta = metrics.track_accuracy_split;
r10 = ta.records([ta.records.track_id] == 10);
r20 = ta.records([ta.records.track_id] == 20);
assert(r10.is_primary_match && r10.matched_truth_key == 2);
assert(r20.is_primary_match && r20.matched_truth_key == 1);
assert(r10.matched_n_correct + r20.matched_n_correct == 16);
assert(ta.n_correct_tracks == 0);
assert(metrics.rms.n_tracks == 2);

st = metrics.start_time;
assert(st.n_main_confirmed == 2);
assert(st.records([st.records.truth_key] == 1).main_track_id == 20);
assert(st.records([st.records.truth_key] == 2).main_track_id == 10);
assert(st.n_start_order_violations == 0);
end

function est = empty_estimate(K)
est = struct();
est.X = cell(K, 1);
est.L = cell(K, 1);
est.assoc = cell(K, 1);
end

function [x, L] = confirmed_output(pos, id)
x = zeros(9, 1);
x([1, 4, 7]) = pos(:);
L = [1, id];
end

function cfg = metric_cfg(purity_threshold, min_assoc)
cfg = struct();
cfg.metrics_enabled = true;
cfg.metrics_max_print = 3;
cfg.frame_time_window_s = 1;
cfg.track_accuracy_purity_th = purity_threshold;
cfg.track_accuracy_min_assoc = min_assoc;
cfg.truth_id_split_enabled = true;
cfg.truth_id_split_dist_m = 10000;
cfg.truth_id_split_max_gap_s = 30;
cfg.truth_assoc_map_max_dist_m = inf;
cfg.truth_rts_meas_std_m = 10;
cfg.truth_rts_proc_std_mps2 = 1;
cfg.truth_rms_time_tolerance_s = 1;
end
