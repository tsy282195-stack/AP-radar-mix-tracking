function test_active_assoc_cascade()
%TEST_ACTIVE_ASSOC_CASCADE  Confirmed-first active association regressions.
test_persistent_independent_conflict_can_confirm();
test_isolated_conflict_cannot_replace_confirmed_track();
test_assignment_gate_is_stricter_than_unmatched_penalty();
test_validation_rejects_hidden_assignment_gate();
fprintf('test_active_assoc_cascade: PASS\n');
end

function test_persistent_independent_conflict_can_confirm()
cfg = cascade_cfg();
cfg.M_confirm = 3;
cfg.N_confirm = 5;
xyz = {[0;0;0], [0;0;0], [0;0;0], ...
    [0,800;0,0;0,0], [0,800;0,0;0,0], [0,800;0,0;0,0]};
R = {cov3(1), cov3(1), cov3(1), cov3(2), cov3(2), cov3(2)};
times = (0:5)';

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(est.tracks{3}.conf(1) == 1);
assert(size(est.tracks{4}.L, 2) == 2);
idx2 = find(est.tracks{4}.L(2, :) == 2, 1);
assert(~isempty(idx2));
assert(est.tracks{4}.birth_kind(idx2) == 2);
assert(est.tracks{4}.parent_id(idx2) == 1);
assert(est.N(6) == 2);
assert(all(sort(est.L{6}(:, 2)).' == [1, 2]));
end

function test_isolated_conflict_cannot_replace_confirmed_track()
cfg = cascade_cfg();
cfg.M_confirm = 3;
cfg.N_confirm = 5;
cfg.tentative_max_miss = 2;
xyz = {[0;0;0], [0;0;0], [0;0;0], [0,800;0,0;0,0], ...
    [0;0;0], [0;0;0], [0;0;0], [0;0;0]};
R = {cov3(1), cov3(1), cov3(1), cov3(2), cov3(1), cov3(1), cov3(1), cov3(1)};
times = (0:7)';

est = run_filter_adapt_ckf(xyz, R, times, cfg);
assert(est.N(8) == 1);
assert(est.L{8}(1, 2) == 1);
assert(est.track_delete_stats.n_tentative_miss >= 1);
assert(est.birth_stats.n_secondary_conflict >= 1);
assert(est.birth_stats.n_secondary_conflict_confirmed == 0);
end

function test_assignment_gate_is_stricter_than_unmatched_penalty()
cfg = cascade_cfg();
cfg.M_confirm = 1;
cfg.N_confirm = 1;
cfg.cost_unmatched = 150;
cfg.assoc_accept_nis = 16;
xyz = {[0;0;0], [5000;0;0]};
R = {cov3(1), cov3(1)};

est = run_filter_adapt_ckf(xyz, R, [0;1], cfg);
old_idx = find(est.tracks{2}.L(2, :) == 1, 1);
new_idx = find(est.tracks{2}.L(2, :) == 2, 1);
assert(~isempty(old_idx) && ~isempty(new_idx));
assert(est.tracks{2}.miss(old_idx) == 1);
assert(est.tracks{2}.birth_kind(new_idx) == 1);
assert(est.active_assoc_stats.n_edges_over_accept_nis >= 1);
assert(isequal(est.assoc{2}.id, 2));
end

function test_validation_rejects_hidden_assignment_gate()
cfg = cascade_cfg();
cfg.cost_unmatched = 8;
cfg.assoc_accept_nis = 16;
did_error = false;
try
    validate_config_fusion(cfg);
catch ME
    did_error = contains(ME.message, 'cost_unmatched');
end
assert(did_error);
end

function cfg = cascade_cfg()
cfg = config_fusion();
cfg.use_imm = false;
cfg.vel_trend_enabled = false;
cfg.use_target_id_prior = false;
cfg.adapt_R_enabled = false;
cfg.meas_robust_enabled = false;
cfg.active_pre_gate_enabled = false;
cfg.birth_suppress_gated = false;
cfg.birth_guard_m = 10;
cfg.merge_pos_dist_m = 1;
cfg.assoc_pos_gate_m = 5000;
cfg.cost_unmatched = 150;
cfg.nis_gate = 16;
cfg.assoc_accept_nis = 16;
cfg.meas_fuse_enabled = true;
cfg.meas_fuse_weight = 'likelihood';
cfg.meas_fuse_R_scale = 1.5;
cfg.meas_fuse_cluster_gamma = 16;
cfg.meas_fuse_cluster_dist_m = 200;
cfg.vinit_baseline_s = 10;
cfg.track_timeout_enabled = true;
cfg.confirmed_max_silence_s = 20;
cfg.tentative_max_silence_s = 10;
% This suite verifies miss-based conflict lifecycle, not weight pruning.
cfg.missed_weight_decay = 1;
cfg.do_plot = false;
end

function R = cov3(n)
R = repmat(diag([100^2, 100^2, 100^2]), 1, 1, n);
end
