function cfg = validate_config_fusion(cfg)
%VALIDATE_CONFIG_FUSION  主被动融合配置一致性检查

if ~isfield(cfg, 'active_files') || isempty(cfg.active_files)
    cfg.active_files = {};
elseif ~iscell(cfg.active_files)
    error('cfg.active_files 必须是cell数组');
end
if ~isfield(cfg, 'passive_files') || isempty(cfg.passive_files)
    cfg.passive_files = {};
elseif ~iscell(cfg.passive_files)
    error('cfg.passive_files 必须是cell数组');
end
if isempty(cfg.active_files) && isempty(cfg.passive_files)
    error('cfg.active_files 与 cfg.passive_files 不能同时为空');
elseif isempty(cfg.active_files)
    warning('未配置主动雷达文件，将运行纯二维角度跟踪。');
elseif isempty(cfg.passive_files)
    warning('未配置被动雷达文件，将运行纯主动三维/伴随二维跟踪。');
end

if ~isfield(cfg, 'read_percent') || isempty(cfg.read_percent)
    cfg.read_percent = 100;
end
if ~isscalar(cfg.read_percent) || ~isfinite(cfg.read_percent)
    error('cfg.read_percent 必须是0到100之间的有限数值');
end
cfg.read_percent = min(max(cfg.read_percent, 0), 100);
if ~isfield(cfg, 'read_start_percent') || isempty(cfg.read_start_percent)
    cfg.read_start_percent = 0;
end
if ~isfield(cfg, 'read_end_percent') || isempty(cfg.read_end_percent)
    cfg.read_end_percent = cfg.read_percent;
end
if ~isscalar(cfg.read_start_percent) || ~isscalar(cfg.read_end_percent) || ...
        ~isfinite(cfg.read_start_percent) || ~isfinite(cfg.read_end_percent)
    error('cfg.read_start_percent 和 cfg.read_end_percent 必须是0到100之间的有限数值');
end
cfg.read_start_percent = min(max(cfg.read_start_percent, 0), 100);
cfg.read_end_percent = min(max(cfg.read_end_percent, 0), 100);
if cfg.read_end_percent < cfg.read_start_percent
    error('读取百分比范围要求 cfg.read_end_percent >= cfg.read_start_percent');
end
if ~isfield(cfg, 'max_targets_per_row') || isempty(cfg.max_targets_per_row)
    cfg.max_targets_per_row = 64;
end
if ~isfield(cfg, 'max_rows') || isempty(cfg.max_rows) || ~isscalar(cfg.max_rows) || ...
        isnan(cfg.max_rows) || cfg.max_rows < 1 || ...
        (isfinite(cfg.max_rows) && floor(cfg.max_rows) ~= cfg.max_rows)
    error('cfg.max_rows 必须为正整数或inf');
end
if isfield(cfg, 'time_range_s') && ~isempty(cfg.time_range_s)
    if numel(cfg.time_range_s) ~= 2 || any(~isfinite(cfg.time_range_s)) || cfg.time_range_s(2) < cfg.time_range_s(1)
        error('cfg.time_range_s 必须为空或 [t_start, t_end]，且 t_end >= t_start');
    end
end
if ~isscalar(cfg.max_targets_per_row) || ~isfinite(cfg.max_targets_per_row) || ...
        cfg.max_targets_per_row < 1 || floor(cfg.max_targets_per_row) ~= cfg.max_targets_per_row
    error('cfg.max_targets_per_row 必须是正整数');
end

validate_layout(cfg.active, {'time_col','count_col','target_id_col','az_col','el_col','range_col','stride'}, 'cfg.active');
validate_vector_cols(cfg.active.valid_cols, 3, 'cfg.active.valid_cols');
validate_layout(cfg.passive, {'time_col','count_col','target_id_col','az_col','el_col','stride'}, 'cfg.passive');
validate_vector_cols(cfg.passive.valid_cols, 2, 'cfg.passive.valid_cols');
validate_layout(cfg.platform, {'time_col','lat_col','lon_col','alt_col'}, 'cfg.platform');
cfg.active.angle_unit = validate_choice(cfg.active.angle_unit, ...
    {'deg','rad','mrad'}, 'cfg.active.angle_unit');
cfg.passive.angle_unit = validate_choice(cfg.passive.angle_unit, ...
    {'deg','rad','mrad'}, 'cfg.passive.angle_unit');
cfg.platform.angle_unit = validate_choice(cfg.platform.angle_unit, ...
    {'deg','rad','mrad'}, 'cfg.platform.angle_unit');
cfg.active.time_format = validate_choice(cfg.active.time_format, ...
    {'hms','seconds','second','sec','s','numeric'}, 'cfg.active.time_format');
cfg.passive.time_format = validate_choice(cfg.passive.time_format, ...
    {'hms','seconds','second','sec','s','numeric'}, 'cfg.passive.time_format');
cfg.platform.time_format = validate_choice(cfg.platform.time_format, ...
    {'hms','seconds','second','sec','s','numeric'}, 'cfg.platform.time_format');
cfg = set_default(cfg, 'platform_max_extrapolation_s', 0.1);
if ~isnumeric(cfg.platform_max_extrapolation_s) || ~isreal(cfg.platform_max_extrapolation_s)
    error('cfg.platform_max_extrapolation_s 必须为有限非负实数');
end
must_be_nonnegative(cfg.platform_max_extrapolation_s, 'cfg.platform_max_extrapolation_s');

must_be_positive(cfg.frame_time_window_s, 'cfg.frame_time_window_s');
must_be_positive(cfg.sigma_range_m, 'cfg.sigma_range_m');
must_be_positive(cfg.sigma_az_deg, 'cfg.sigma_az_deg');
must_be_positive(cfg.sigma_el_deg, 'cfg.sigma_el_deg');
must_be_positive(cfg.sigma_passive_az_deg, 'cfg.sigma_passive_az_deg');
must_be_positive(cfg.sigma_passive_el_deg, 'cfg.sigma_passive_el_deg');

cfg = set_default(cfg, 'condense_enable', false);
cfg = set_default(cfg, 'condense_method', 'spatiotemporal');
cfg = set_default(cfg, 'condense_radius_m', 100);
cfg = set_default(cfg, 'condense_res_range_m', 100);
cfg = set_default(cfg, 'condense_res_az_deg', 0.2);
cfg = set_default(cfg, 'condense_res_el_deg', 0.2);
cfg = set_default(cfg, 'condense_gate_gamma', 9);
cfg = set_default(cfg, 'condense_birth_gamma', 6);
cfg = set_default(cfg, 'condense_birth_merge_enabled', false);
cfg = set_default(cfg, 'condense_amax', 10);
cfg = set_default(cfg, 'condense_vel_beta', 0.15);
cfg = set_default(cfg, 'condense_coast', 3);
cfg.condense_enable = validate_flag(cfg.condense_enable, 'cfg.condense_enable');
cfg.condense_birth_merge_enabled = validate_flag( ...
    cfg.condense_birth_merge_enabled, 'cfg.condense_birth_merge_enabled');
cfg.condense_method = validate_choice(cfg.condense_method, ...
    {'spatiotemporal','resolution','radius'}, 'cfg.condense_method');
must_be_positive(cfg.condense_radius_m, 'cfg.condense_radius_m');
must_be_positive(cfg.condense_res_range_m, 'cfg.condense_res_range_m');
must_be_positive(cfg.condense_res_az_deg, 'cfg.condense_res_az_deg');
must_be_positive(cfg.condense_res_el_deg, 'cfg.condense_res_el_deg');
must_be_positive(cfg.condense_gate_gamma, 'cfg.condense_gate_gamma');
must_be_positive(cfg.condense_birth_gamma, 'cfg.condense_birth_gamma');
must_be_nonnegative(cfg.condense_amax, 'cfg.condense_amax');
must_be_probability(cfg.condense_vel_beta, 'cfg.condense_vel_beta');
if ~isscalar(cfg.condense_coast) || ~isfinite(cfg.condense_coast) || ...
        cfg.condense_coast < 0 || floor(cfg.condense_coast) ~= cfg.condense_coast
    error('cfg.condense_coast 必须是非负整数帧数');
end

cfg = set_default(cfg, 'async_microbatch_dt_s', 0.005);
cfg = set_default(cfg, 'async_active_time_mode', 'frame');
cfg = set_default(cfg, 'async_passive_count_miss', false);
cfg = set_default(cfg, 'passive_bearing_enabled', true);
cfg = set_default(cfg, 'passive_bearing_gate', 4);
cfg = set_default(cfg, 'passive_bearing_nis_gate', 4);
cfg = set_default(cfg, 'passive_bearing_weight_gain', 0.6);
cfg = set_default(cfg, 'passive_bearing_confirm_hit', false);
cfg = set_default(cfg, 'passive_bearing_update_on_active', true);
cfg = set_default(cfg, 'passive_bearing_update_on_pure', true);
cfg = set_default(cfg, 'passive_bearing_update_active_hit_tracks', false);
cfg = set_default(cfg, 'passive_bearing_min_dt_s', 0.10);
cfg = set_default(cfg, 'passive_bearing_fast_gate_deg', 2.0);
cfg = set_default(cfg, 'passive_meas_fuse_enabled', true);
cfg = set_default(cfg, 'passive_meas_fuse_weight', 'likelihood');
cfg = set_default(cfg, 'passive_meas_fuse_R_scale', 1.5);
cfg = set_default(cfg, 'passive_meas_fuse_max_count', 4);
cfg = set_default(cfg, 'passive_meas_fuse_max_spread_deg', 0.35);
cfg = set_default(cfg, 'passive_meas_fuse_amb_ratio', 1.5);
cfg = set_default(cfg, 'passive_meas_fuse_amb_abs_nis', 0.5);
cfg = set_default(cfg, 'active_pre_gate_enabled', false);
cfg = set_default(cfg, 'nis_gate', 16);
cfg = set_default(cfg, 'nis_max_bad', 5);
cfg = set_default(cfg, 'assoc_accept_nis', cfg.nis_gate);
cfg = set_default(cfg, 'track_timeout_enabled', true);
cfg = set_default(cfg, 'tentative_max_silence_s', 2);
cfg = set_default(cfg, 'confirmed_max_silence_s', 6);
cfg = set_default(cfg, 'meas_fuse_enabled', true);
cfg = set_default(cfg, 'meas_fuse_weight', 'likelihood');
cfg = set_default(cfg, 'meas_fuse_R_scale', 1.5);
cfg = set_default(cfg, 'meas_fuse_cluster_gamma', 16);
cfg = set_default(cfg, 'meas_fuse_cluster_dist_m', 200);
cfg = set_default(cfg, 'birth_guard_m', 1000);
cfg = set_default(cfg, 'birth_suppress_gated', true);
cfg = set_default(cfg, 'merge_pos_dist_m', 400);
cfg = set_default(cfg, 'dedup_vel_angle_deg', 35);
cfg = set_default(cfg, 'dedup_min_speed', 30);
cfg = set_default(cfg, 'processing_framework', 'joint_2d3d');
cfg = set_default(cfg, 'parallel_file_loading', false);
cfg = set_default(cfg, 'parallel_file_workers', 0);
cfg = set_default(cfg, 'joint_scan_tolerance_s', cfg.frame_time_window_s);
cfg = set_default(cfg, 'joint_sync_tolerance_s', min(cfg.frame_time_window_s, 0.010));
cfg = set_default(cfg, 'joint_shard_dedup_enabled', true);
cfg = set_default(cfg, 'joint_shard_duplicate_angle_deg', 0.02);
cfg = set_default(cfg, 'joint_shard_duplicate_range_m', 30);
cfg = set_default(cfg, 'joint_confirm_M', 3);
cfg = set_default(cfg, 'joint_confirm_N', 5);
cfg = set_default(cfg, 'joint_gate_2d', 9.2103);
cfg = set_default(cfg, 'joint_gate_3d', 11.3449);
cfg = set_default(cfg, 'joint_unmatched_cost', 50);
cfg = set_default(cfg, 'joint_birth_explain_nis', 1.0);
cfg = set_default(cfg, 'joint_tentative_timeout_s', 3);
cfg = set_default(cfg, 'joint_confirmed_timeout_s', 12);
cfg = set_default(cfg, 'joint_angle_q_cv', 0.08);
cfg = set_default(cfg, 'joint_angle_q_ca', 0.50);
cfg = set_default(cfg, 'joint_angle_rate_birth_std_dps', 2.0);
cfg = set_default(cfg, 'joint_angle_acc_birth_std_dps2', 3.0);
cfg = set_default(cfg, 'joint_space_sigma_a_cv', 12);
cfg = set_default(cfg, 'joint_space_sigma_j_ca', 15);
cfg = set_default(cfg, 'joint_space_vel_birth_std_mps', 500);
cfg = set_default(cfg, 'joint_space_acc_birth_std_mps2', 80);
cfg = set_default(cfg, 'joint_imm_cv_stay', 0.96);
cfg = set_default(cfg, 'joint_imm_ca_stay', 0.94);
cfg = set_default(cfg, 'joint_imm_cv_probability', 0.65);
cfg = set_default(cfg, 'joint_angle95_max_deg', 1.0);
cfg = set_default(cfg, 'joint_3d_birth_M', 3);
cfg = set_default(cfg, 'joint_3d_birth_N', 5);
cfg = set_default(cfg, 'joint_3d_upgrade_M', 2);
cfg = set_default(cfg, 'joint_3d_upgrade_N', 3);
cfg = set_default(cfg, 'joint_up_consecutive', 1);
cfg = set_default(cfg, 'joint_down_consecutive', 3);
cfg = set_default(cfg, 'joint_radial_sigma_warn_m', 5000);
cfg = set_default(cfg, 'joint_radial_sigma_drop_m', 10000);
cfg = set_default(cfg, 'joint_range_age_warn_s', 3);
cfg = set_default(cfg, 'joint_range_age_drop_s', 10);
cfg = set_default(cfg, 'joint_pos95_warn_m', 15000);
cfg = set_default(cfg, 'joint_pos95_down_m', 30000);
cfg = set_default(cfg, 'joint_pos95_recover_m', 10000);
cfg = set_default(cfg, 'joint_radial95_warn_m', 10000);
cfg = set_default(cfg, 'joint_radial95_down_m', 20000);
cfg = set_default(cfg, 'joint_radial95_recover_m', 7000);
cfg = set_default(cfg, 'joint_switch_gate_2d', 9.2103);
cfg = set_default(cfg, 'joint_switch_cov_inflate', 2.0);
cfg = set_default(cfg, 'joint_relative_range_down', 0.60);
cfg = set_default(cfg, 'joint_space_nis_window', 5);
cfg = set_default(cfg, 'joint_space_nis_recover', 1.5);
cfg = set_default(cfg, 'joint_space_nis_warn', 2.5);
cfg = set_default(cfg, 'joint_space_nis_down', 4.0);
cfg = set_default(cfg, 'joint_max_predict_dt_s', 5);
cfg = set_default(cfg, 'joint_shadow_max_s', 20);
cfg = set_default(cfg, 'joint_mode_3d_prior_cost', 0.5);
cfg = set_default(cfg, 'joint_merge_angle_deg', 0.08);
cfg = set_default(cfg, 'joint_merge_rate_dps', 1.0);
cfg = set_default(cfg, 'joint_merge_nis', 13.2767);
cfg = set_default(cfg, 'joint_history_level', 'output');

cfg.async_active_time_mode = validate_choice(cfg.async_active_time_mode, ...
    {'frame','plot'}, 'cfg.async_active_time_mode');
cfg.passive_meas_fuse_weight = validate_choice(cfg.passive_meas_fuse_weight, ...
    {'likelihood','equal'}, 'cfg.passive_meas_fuse_weight');
cfg.meas_fuse_weight = validate_choice(cfg.meas_fuse_weight, ...
    {'likelihood','equal'}, 'cfg.meas_fuse_weight');
cfg.processing_framework = validate_choice(cfg.processing_framework, ...
    {'joint_2d3d','legacy_active3d'}, 'cfg.processing_framework');
cfg.joint_history_level = validate_choice(cfg.joint_history_level, ...
    {'output','diagnostic','full'}, 'cfg.joint_history_level');
cfg.async_passive_count_miss = validate_flag(cfg.async_passive_count_miss, 'cfg.async_passive_count_miss');
cfg.passive_bearing_enabled = validate_flag(cfg.passive_bearing_enabled, 'cfg.passive_bearing_enabled');
cfg.passive_bearing_confirm_hit = validate_flag(cfg.passive_bearing_confirm_hit, 'cfg.passive_bearing_confirm_hit');
cfg.passive_bearing_update_on_active = validate_flag(cfg.passive_bearing_update_on_active, 'cfg.passive_bearing_update_on_active');
cfg.passive_bearing_update_on_pure = validate_flag(cfg.passive_bearing_update_on_pure, 'cfg.passive_bearing_update_on_pure');
cfg.passive_bearing_update_active_hit_tracks = validate_flag(cfg.passive_bearing_update_active_hit_tracks, 'cfg.passive_bearing_update_active_hit_tracks');
cfg.passive_meas_fuse_enabled = validate_flag(cfg.passive_meas_fuse_enabled, 'cfg.passive_meas_fuse_enabled');
cfg.active_pre_gate_enabled = validate_flag(cfg.active_pre_gate_enabled, 'cfg.active_pre_gate_enabled');
cfg.track_timeout_enabled = validate_flag(cfg.track_timeout_enabled, 'cfg.track_timeout_enabled');
cfg.meas_fuse_enabled = validate_flag(cfg.meas_fuse_enabled, 'cfg.meas_fuse_enabled');
cfg.birth_suppress_gated = validate_flag(cfg.birth_suppress_gated, ...
    'cfg.birth_suppress_gated');
cfg.joint_shard_dedup_enabled = validate_flag(cfg.joint_shard_dedup_enabled, ...
    'cfg.joint_shard_dedup_enabled');
cfg.parallel_file_loading = validate_flag(cfg.parallel_file_loading, ...
    'cfg.parallel_file_loading');

must_be_nonnegative(cfg.async_microbatch_dt_s, 'cfg.async_microbatch_dt_s');
must_be_positive(cfg.passive_bearing_gate, 'cfg.passive_bearing_gate');
must_be_positive(cfg.passive_bearing_nis_gate, 'cfg.passive_bearing_nis_gate');
must_be_nonnegative(cfg.passive_bearing_weight_gain, 'cfg.passive_bearing_weight_gain');
must_be_nonnegative(cfg.passive_bearing_min_dt_s, 'cfg.passive_bearing_min_dt_s');
must_be_nonnegative(cfg.passive_bearing_fast_gate_deg, 'cfg.passive_bearing_fast_gate_deg');
must_be_positive(cfg.passive_meas_fuse_R_scale, 'cfg.passive_meas_fuse_R_scale');
must_be_nonnegative(cfg.passive_meas_fuse_max_spread_deg, 'cfg.passive_meas_fuse_max_spread_deg');
must_be_nonnegative(cfg.passive_meas_fuse_amb_abs_nis, 'cfg.passive_meas_fuse_amb_abs_nis');
must_be_positive(cfg.assoc_accept_nis, 'cfg.assoc_accept_nis');
must_be_positive_or_inf(cfg.tentative_max_silence_s, 'cfg.tentative_max_silence_s');
must_be_positive_or_inf(cfg.confirmed_max_silence_s, 'cfg.confirmed_max_silence_s');
must_be_positive(cfg.meas_fuse_R_scale, 'cfg.meas_fuse_R_scale');
must_be_positive(cfg.meas_fuse_cluster_gamma, 'cfg.meas_fuse_cluster_gamma');
must_be_positive(cfg.meas_fuse_cluster_dist_m, 'cfg.meas_fuse_cluster_dist_m');
must_be_nonnegative(cfg.birth_guard_m, 'cfg.birth_guard_m');
must_be_nonnegative(cfg.merge_pos_dist_m, 'cfg.merge_pos_dist_m');
if ~isscalar(cfg.dedup_vel_angle_deg) || ~isfinite(cfg.dedup_vel_angle_deg) || ...
        cfg.dedup_vel_angle_deg < 0 || cfg.dedup_vel_angle_deg > 180
    error('cfg.dedup_vel_angle_deg 必须位于 [0, 180] 度');
end
must_be_nonnegative(cfg.dedup_min_speed, 'cfg.dedup_min_speed');
must_be_nonnegative(cfg.joint_scan_tolerance_s, 'cfg.joint_scan_tolerance_s');
if ~isscalar(cfg.parallel_file_workers) || ~isfinite(cfg.parallel_file_workers) || ...
        cfg.parallel_file_workers < 0 || floor(cfg.parallel_file_workers) ~= cfg.parallel_file_workers
    error('cfg.parallel_file_workers 必须是非负整数，0表示自动');
end
must_be_nonnegative(cfg.joint_sync_tolerance_s, 'cfg.joint_sync_tolerance_s');
if cfg.joint_sync_tolerance_s > cfg.joint_scan_tolerance_s
    error('cfg.joint_sync_tolerance_s 不得大于 cfg.joint_scan_tolerance_s');
end
must_be_nonnegative(cfg.joint_shard_duplicate_angle_deg, ...
    'cfg.joint_shard_duplicate_angle_deg');
must_be_nonnegative(cfg.joint_shard_duplicate_range_m, ...
    'cfg.joint_shard_duplicate_range_m');
must_be_positive(cfg.joint_gate_2d, 'cfg.joint_gate_2d');
must_be_positive(cfg.joint_gate_3d, 'cfg.joint_gate_3d');
must_be_positive(cfg.joint_unmatched_cost, 'cfg.joint_unmatched_cost');
must_be_nonnegative(cfg.joint_birth_explain_nis, 'cfg.joint_birth_explain_nis');
must_be_positive_or_inf(cfg.joint_tentative_timeout_s, 'cfg.joint_tentative_timeout_s');
must_be_positive_or_inf(cfg.joint_confirmed_timeout_s, 'cfg.joint_confirmed_timeout_s');
must_be_nonnegative(cfg.joint_angle_q_cv, 'cfg.joint_angle_q_cv');
must_be_nonnegative(cfg.joint_angle_q_ca, 'cfg.joint_angle_q_ca');
must_be_positive(cfg.joint_angle_rate_birth_std_dps, 'cfg.joint_angle_rate_birth_std_dps');
must_be_positive(cfg.joint_angle_acc_birth_std_dps2, 'cfg.joint_angle_acc_birth_std_dps2');
must_be_nonnegative(cfg.joint_space_sigma_a_cv, 'cfg.joint_space_sigma_a_cv');
must_be_nonnegative(cfg.joint_space_sigma_j_ca, 'cfg.joint_space_sigma_j_ca');
must_be_positive(cfg.joint_space_vel_birth_std_mps, 'cfg.joint_space_vel_birth_std_mps');
must_be_positive(cfg.joint_space_acc_birth_std_mps2, 'cfg.joint_space_acc_birth_std_mps2');
must_be_probability(cfg.joint_imm_cv_stay, 'cfg.joint_imm_cv_stay');
must_be_probability(cfg.joint_imm_ca_stay, 'cfg.joint_imm_ca_stay');
must_be_probability(cfg.joint_imm_cv_probability, 'cfg.joint_imm_cv_probability');
must_be_positive(cfg.joint_angle95_max_deg, 'cfg.joint_angle95_max_deg');
validate_m_of_n(cfg.joint_confirm_M, cfg.joint_confirm_N, 'joint_confirm');
validate_m_of_n(cfg.joint_3d_birth_M, cfg.joint_3d_birth_N, 'joint_3d_birth');
validate_m_of_n(cfg.joint_3d_upgrade_M, cfg.joint_3d_upgrade_N, 'joint_3d_upgrade');
must_be_positive_integer(cfg.joint_up_consecutive, 'cfg.joint_up_consecutive');
must_be_positive_integer(cfg.joint_down_consecutive, 'cfg.joint_down_consecutive');
must_be_positive(cfg.joint_radial_sigma_warn_m, 'cfg.joint_radial_sigma_warn_m');
must_be_positive(cfg.joint_radial_sigma_drop_m, 'cfg.joint_radial_sigma_drop_m');
must_be_positive(cfg.joint_range_age_warn_s, 'cfg.joint_range_age_warn_s');
must_be_positive(cfg.joint_range_age_drop_s, 'cfg.joint_range_age_drop_s');
must_be_positive_integer(cfg.joint_space_nis_window, 'cfg.joint_space_nis_window');
must_be_positive(cfg.joint_max_predict_dt_s, 'cfg.joint_max_predict_dt_s');
must_be_positive(cfg.joint_switch_gate_2d, 'cfg.joint_switch_gate_2d');
if ~isscalar(cfg.joint_switch_cov_inflate) || ~isfinite(cfg.joint_switch_cov_inflate) || ...
        cfg.joint_switch_cov_inflate < 1
    error('cfg.joint_switch_cov_inflate 必须是大于等于1的有限数');
end
must_be_positive(cfg.joint_relative_range_down, 'cfg.joint_relative_range_down');
must_be_nonnegative(cfg.joint_shadow_max_s, 'cfg.joint_shadow_max_s');
must_be_nonnegative(cfg.joint_mode_3d_prior_cost, 'cfg.joint_mode_3d_prior_cost');
must_be_nonnegative(cfg.joint_merge_angle_deg, 'cfg.joint_merge_angle_deg');
must_be_nonnegative(cfg.joint_merge_rate_dps, 'cfg.joint_merge_rate_dps');
must_be_positive(cfg.joint_merge_nis, 'cfg.joint_merge_nis');
must_be_nonnegative(cfg.joint_pos95_recover_m, 'cfg.joint_pos95_recover_m');
must_be_positive(cfg.joint_pos95_warn_m, 'cfg.joint_pos95_warn_m');
must_be_positive(cfg.joint_pos95_down_m, 'cfg.joint_pos95_down_m');
must_be_nonnegative(cfg.joint_radial95_recover_m, 'cfg.joint_radial95_recover_m');
must_be_positive(cfg.joint_radial95_warn_m, 'cfg.joint_radial95_warn_m');
must_be_positive(cfg.joint_radial95_down_m, 'cfg.joint_radial95_down_m');
must_be_nonnegative(cfg.joint_space_nis_recover, 'cfg.joint_space_nis_recover');
must_be_positive(cfg.joint_space_nis_warn, 'cfg.joint_space_nis_warn');
must_be_positive(cfg.joint_space_nis_down, 'cfg.joint_space_nis_down');
if ~(cfg.joint_radial_sigma_warn_m < cfg.joint_radial_sigma_drop_m)
    error('径向距离不确定度阈值要求 warn < drop');
end
if ~(cfg.joint_range_age_warn_s < cfg.joint_range_age_drop_s)
    error('有效距离信息龄期阈值要求 warn < drop');
end
if ~(cfg.joint_pos95_recover_m < cfg.joint_pos95_warn_m && ...
        cfg.joint_pos95_warn_m < cfg.joint_pos95_down_m)
    error('三维位置门限要求 recover < warn < down');
end
if ~(cfg.joint_radial95_recover_m < cfg.joint_radial95_warn_m && ...
        cfg.joint_radial95_warn_m < cfg.joint_radial95_down_m)
    error('径向误差门限要求 recover < warn < down');
end
if ~(cfg.joint_space_nis_recover < cfg.joint_space_nis_warn && ...
        cfg.joint_space_nis_warn < cfg.joint_space_nis_down)
    error('空间归一化NIS门限要求 recover < warn < down');
end
if cfg.joint_unmatched_cost < max(cfg.joint_gate_2d, cfg.joint_gate_3d)
    error('cfg.joint_unmatched_cost 不得小于二维/三维关联门，否则会形成隐藏的更严关联门');
end
if cfg.joint_birth_explain_nis > min(cfg.joint_gate_2d, cfg.joint_gate_3d)
    error('cfg.joint_birth_explain_nis 不得大于二维/三维关联门的较小值');
end
if cfg.joint_confirmed_timeout_s < cfg.joint_tentative_timeout_s
    error('cfg.joint_confirmed_timeout_s 应不小于 cfg.joint_tentative_timeout_s');
end
if ~isscalar(cfg.passive_meas_fuse_max_count) || ~isfinite(cfg.passive_meas_fuse_max_count) || ...
        cfg.passive_meas_fuse_max_count < 1 || floor(cfg.passive_meas_fuse_max_count) ~= cfg.passive_meas_fuse_max_count
    error('cfg.passive_meas_fuse_max_count 必须是正整数');
end
if ~isscalar(cfg.passive_meas_fuse_amb_ratio) || ~isfinite(cfg.passive_meas_fuse_amb_ratio) || ...
        cfg.passive_meas_fuse_amb_ratio < 1
    error('cfg.passive_meas_fuse_amb_ratio 必须 >= 1');
end
if ~(ischar(cfg.local_origin) && strcmp(cfg.local_origin, 'first_platform')) && ...
        ~(isnumeric(cfg.local_origin) && numel(cfg.local_origin) == 3 && all(isfinite(cfg.local_origin)))
    error('cfg.local_origin 必须为 ''first_platform'' 或 [lat, lon, alt]');
end
must_be_positive(cfg.gating_gamma, 'cfg.gating_gamma');
must_be_positive(cfg.cost_unmatched, 'cfg.cost_unmatched');
must_be_positive(cfg.nis_gate, 'cfg.nis_gate');
if ~isscalar(cfg.nis_max_bad) || ~isfinite(cfg.nis_max_bad) || ...
        cfg.nis_max_bad < 1 || floor(cfg.nis_max_bad) ~= cfg.nis_max_bad
    error('cfg.nis_max_bad 必须是正整数');
end
if cfg.assoc_accept_nis > cfg.nis_gate
    error('cfg.assoc_accept_nis 不得大于 cfg.nis_gate，否则异常量测仍可能被计作合规命中');
end
if cfg.cost_unmatched < cfg.assoc_accept_nis
    error(['cfg.cost_unmatched 必须不小于 cfg.assoc_accept_nis；否则未关联代价会' ...
        '重新成为比主动NIS门更严格的隐含验收门']);
end
if cfg.confirmed_max_silence_s < cfg.tentative_max_silence_s
    error('cfg.confirmed_max_silence_s 应不小于 cfg.tentative_max_silence_s');
end
if cfg.P_S < 0 || cfg.P_S > 1 || cfg.P_D < 0 || cfg.P_D > 1
    error('cfg.P_S 和 cfg.P_D 必须位于 [0, 1]');
end
if isfield(cfg, 'missed_weight_decay') && ~isempty(cfg.missed_weight_decay) && ...
        (cfg.missed_weight_decay < 0 || cfg.missed_weight_decay > 1)
    error('cfg.missed_weight_decay 必须位于 [0, 1]，或设为空数组 []');
end
if cfg.M_confirm > cfg.N_confirm
    error('航迹确认参数要求 cfg.M_confirm <= cfg.N_confirm');
end
if cfg.M_confirm < 1 || cfg.N_confirm < 1 || ...
        floor(cfg.M_confirm) ~= cfg.M_confirm || floor(cfg.N_confirm) ~= cfg.N_confirm
    error('cfg.M_confirm 和 cfg.N_confirm 必须为正整数');
end
if cfg.max_tracks < 1 || floor(cfg.max_tracks) ~= cfg.max_tracks
    error('cfg.max_tracks 必须为正整数');
end

% ── 改进A：滑窗速度趋势参数校验/规整（缺省由滤波器内置默认补齐）──
if isfield(cfg, 'vel_trend_blend') && ~isempty(cfg.vel_trend_blend)
    cfg.vel_trend_blend = min(max(cfg.vel_trend_blend, 0), 1);
end
if isfield(cfg, 'vel_trend_window') && ~isempty(cfg.vel_trend_window) && cfg.vel_trend_window < 2
    error('cfg.vel_trend_window 必须 >= 2');
end
if isfield(cfg, 'vel_trend_min_n') && ~isempty(cfg.vel_trend_min_n) && cfg.vel_trend_min_n < 2
    error('cfg.vel_trend_min_n 必须 >= 2');
end
if isfield(cfg, 'vel_trend_span_s') && ~isempty(cfg.vel_trend_span_s) && cfg.vel_trend_span_s < 0
    error('cfg.vel_trend_span_s 必须 >= 0');
end

% ── 改进B：编号先验参数校验 ──
if isfield(cfg, 'id_dup_dist_m') && ~isempty(cfg.id_dup_dist_m) && cfg.id_dup_dist_m <= 0
    error('cfg.id_dup_dist_m 必须为正数(重复编号去重簇间距门)');
end
if isfield(cfg, 'id_prior_cost_bonus') && ~isempty(cfg.id_prior_cost_bonus) && cfg.id_prior_cost_bonus < 0
    error('cfg.id_prior_cost_bonus 必须 >= 0');
end
if isfield(cfg, 'id_prior_min_conf') && ~isempty(cfg.id_prior_min_conf) && cfg.id_prior_min_conf < 1
    error('cfg.id_prior_min_conf 必须 >= 1');
end

% ── 改进C：IMM 参数校验/规整 ──
for fld = {'imm_p_cv_stay', 'imm_p_ca_stay', 'imm_mu_init_cv'}
    nm = fld{1};
    if isfield(cfg, nm) && ~isempty(cfg.(nm))
        if cfg.(nm) < 0 || cfg.(nm) > 1
            error('cfg.%s 必须位于 [0, 1]', nm);
        end
    end
end
if isfield(cfg, 'imm_cv_sigma_a') && ~isempty(cfg.imm_cv_sigma_a) && cfg.imm_cv_sigma_a < 0
    error('cfg.imm_cv_sigma_a 必须 >= 0');
end

% 定量评价默认值和一致性检查。
cfg = set_default(cfg, 'joint_plot_min_life', 3);
cfg = set_default(cfg, 'metrics_enabled', true);
cfg = set_default(cfg, 'metrics_max_print', 12);
cfg = set_default(cfg, 'track_accuracy_purity_th', 0.80);
cfg = set_default(cfg, 'track_accuracy_min_assoc', 3);
cfg = set_default(cfg, 'truth_id_split_enabled', true);
cfg = set_default(cfg, 'truth_cross_sensor_id_consistent', false);
cfg = set_default(cfg, 'truth_cross_sensor_match_angle_deg', 1.0);
cfg = set_default(cfg, 'truth_cross_sensor_match_max_dt_s', 0.5);
cfg = set_default(cfg, 'truth_cross_sensor_match_min_points', 3);
cfg = set_default(cfg, 'truth_cross_sensor_match_min_ratio', 0.20);
cfg = set_default(cfg, 'truth_cross_sensor_match_ambiguity_deg', 0.10);
cfg = set_default(cfg, 'truth_cross_sensor_match_max_samples', 200);
cfg = set_default(cfg, 'truth_id_split_dist_m', 10000);
cfg = set_default(cfg, 'truth_id_split_max_gap_s', 30);
cfg = set_default(cfg, 'truth_assoc_map_max_dist_m', inf);
cfg = set_default(cfg, 'truth_rts_meas_std_m', 1000);
cfg = set_default(cfg, 'truth_rts_proc_std_mps2', 10);
cfg = set_default(cfg, 'truth_rms_time_tolerance_s', cfg.frame_time_window_s);
cfg = set_default(cfg, 'truth_file', '');
cfg = set_default(cfg, 'truth_auto_discover', true);
cfg = set_default(cfg, 'truth_altitude_is_absolute', true);
cfg = set_default(cfg, 'truth_real_time_tolerance_s', max(cfg.frame_time_window_s, 0.03));
cfg.truth_auto_discover = validate_flag(cfg.truth_auto_discover, ...
    'cfg.truth_auto_discover');
cfg.truth_altitude_is_absolute = validate_flag(cfg.truth_altitude_is_absolute, ...
    'cfg.truth_altitude_is_absolute');
cfg.truth_id_split_enabled = validate_flag(cfg.truth_id_split_enabled, ...
    'cfg.truth_id_split_enabled');
cfg.truth_cross_sensor_id_consistent = validate_flag( ...
    cfg.truth_cross_sensor_id_consistent, 'cfg.truth_cross_sensor_id_consistent');
if ~cfg.truth_id_split_enabled && ~cfg.truth_cross_sensor_id_consistent
    error(['关闭 cfg.truth_id_split_enabled 时必须确认 ' ...
        'cfg.truth_cross_sensor_id_consistent=true；否则主被动相同数字编号会被错误视为同一目标。']);
end
if isa(cfg.truth_file, 'string') && isscalar(cfg.truth_file)
    cfg.truth_file = char(cfg.truth_file);
end
if ~ischar(cfg.truth_file)
    error('cfg.truth_file 必须是字符路径或空字符');
end

if ~isscalar(cfg.joint_plot_min_life) || ~isfinite(cfg.joint_plot_min_life) || ...
        cfg.joint_plot_min_life < 1 || ...
        floor(cfg.joint_plot_min_life) ~= cfg.joint_plot_min_life
    error('cfg.joint_plot_min_life 必须是正整数');
end
if ~isscalar(cfg.metrics_max_print) || ~isfinite(cfg.metrics_max_print) || cfg.metrics_max_print < 0 || ...
        floor(cfg.metrics_max_print) ~= cfg.metrics_max_print
    error('cfg.metrics_max_print 必须是非负整数，0表示不打印');
end
if ~isfinite(cfg.track_accuracy_purity_th) || cfg.track_accuracy_purity_th < 0 || ...
        cfg.track_accuracy_purity_th > 1
    error('cfg.track_accuracy_purity_th 必须位于 [0, 1]');
end
if ~isfinite(cfg.track_accuracy_min_assoc) || cfg.track_accuracy_min_assoc < 1 || ...
        floor(cfg.track_accuracy_min_assoc) ~= cfg.track_accuracy_min_assoc
    error('cfg.track_accuracy_min_assoc 必须是正整数');
end
if ~isfinite(cfg.truth_id_split_dist_m) || cfg.truth_id_split_dist_m <= 0
    error('伪真值编号分割的距离门限必须为正数');
end
if isnan(cfg.truth_id_split_max_gap_s) || isnan(cfg.truth_assoc_map_max_dist_m) || ...
        cfg.truth_id_split_max_gap_s < 0 || cfg.truth_assoc_map_max_dist_m < 0
    error('伪真值编号分割的最大间隔和映射距离必须为非负数');
end
if ~isfinite(cfg.truth_rts_meas_std_m) || ~isfinite(cfg.truth_rts_proc_std_mps2) || ...
        ~isfinite(cfg.truth_rms_time_tolerance_s) || ...
        cfg.truth_rts_meas_std_m <= 0 || cfg.truth_rts_proc_std_mps2 <= 0 || ...
        cfg.truth_rms_time_tolerance_s < 0
    error('伪真值RTS噪声参数或RMS时间容差无效');
end
must_be_nonnegative(cfg.truth_real_time_tolerance_s, ...
    'cfg.truth_real_time_tolerance_s');
must_be_positive(cfg.truth_cross_sensor_match_angle_deg, ...
    'cfg.truth_cross_sensor_match_angle_deg');
must_be_nonnegative(cfg.truth_cross_sensor_match_max_dt_s, ...
    'cfg.truth_cross_sensor_match_max_dt_s');
must_be_positive_integer(cfg.truth_cross_sensor_match_min_points, ...
    'cfg.truth_cross_sensor_match_min_points');
must_be_probability(cfg.truth_cross_sensor_match_min_ratio, ...
    'cfg.truth_cross_sensor_match_min_ratio');
must_be_nonnegative(cfg.truth_cross_sensor_match_ambiguity_deg, ...
    'cfg.truth_cross_sensor_match_ambiguity_deg');
must_be_positive_integer(cfg.truth_cross_sensor_match_max_samples, ...
    'cfg.truth_cross_sensor_match_max_samples');

for i = 1:numel(cfg.active_files)
    assert_file_exists(resolve_data_path(cfg.data_dir, cfg.active_files{i}), '主动雷达');
end
for i = 1:numel(cfg.passive_files)
    assert_file_exists(resolve_data_path(cfg.data_dir, cfg.passive_files{i}), '被动雷达');
end
assert_file_exists(resolve_data_path(cfg.data_dir, cfg.platform_file), '平台');
end

%% ═══════════════════════════════════════════════════════════════════════════
function validate_layout(s, names, label)
for i = 1:numel(names)
    name = names{i};
    if ~isfield(s, name) || isempty(s.(name)) || ~isscalar(s.(name)) || ...
            ~isfinite(s.(name)) || s.(name) < 1 || floor(s.(name)) ~= s.(name)
        error('%s.%s 必须是正整数列号/步长', label, name);
    end
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function validate_vector_cols(cols, expected_len, label)
if numel(cols) ~= expected_len || any(~isfinite(cols)) || any(cols < 1) || any(floor(cols) ~= cols)
    error('%s 必须是长度为%d的正整数列号数组', label, expected_len);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function cfg = set_default(cfg, name, value)
if ~isfield(cfg, name) || isempty(cfg.(name))
    cfg.(name) = value;
end
end

function value = validate_choice(value, choices, label)
if isa(value, 'string'), value = char(value); end
if ~ischar(value) || ~any(strcmpi(value, choices))
    error('%s 必须是: %s', label, strjoin(choices, ', '));
end
value = lower(value);
end

function must_be_positive(value, label)
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('%s 必须为正数', label);
end
end

function must_be_positive_integer(value, label)
if ~isscalar(value) || ~isfinite(value) || value < 1 || floor(value) ~= value
    error('%s 必须是正整数', label);
end
end

function validate_m_of_n(m, n, label)
must_be_positive_integer(m, [label, '_M']);
must_be_positive_integer(n, [label, '_N']);
if m > n
    error('%s 要求 M <= N', label);
end
end

function must_be_nonnegative(value, label)
if ~isscalar(value) || ~isfinite(value) || value < 0
    error('%s 必须为非负数', label);
end
end

function must_be_positive_or_inf(value, label)
if ~isscalar(value) || isnan(value) || value <= 0
    error('%s 必须为正数或inf', label);
end
end

function must_be_probability(value, label)
if ~isscalar(value) || ~isfinite(value) || value < 0 || value > 1
    error('%s 必须位于 [0, 1]', label);
end
end

function value = validate_flag(value, label)
if ~isscalar(value) || ~(islogical(value) || isnumeric(value)) || ~isfinite(double(value)) || ...
        ~(double(value) == 0 || double(value) == 1)
    error('%s 必须为 true/false 或 0/1', label);
end
value = logical(value);
end

function assert_file_exists(path, label)
if exist(path, 'file') ~= 2
    error('%s文件不存在: %s', label, path);
end
end
