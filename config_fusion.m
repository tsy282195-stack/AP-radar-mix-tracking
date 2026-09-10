  function cfg = config_fusion()
%CONFIG_FUSION 主被动雷达融合跟踪系统统一参数配置
% 参数按数据输入、预处理、联合滤波、旧三维兼容和结果输出分组。

cfg = struct();

%% 1. 数据文件与运行模式
cfg.active_files = {'meas_sensor1.txt', 'meas_sensor2.txt', 'meas_sensor3.txt'}; % 独立主动观测源文件
cfg.passive_files = {'passive1.txt', 'passive2.txt', 'passive3.txt', ...       % 独立被动观测源文件
                     'passive4.txt', 'passive5.txt', 'passive6.txt'};
cfg.platform_file = 'platform.csv';                                          % 观测平台轨迹文件
cfg.data_dir = 'C:\Users\topsy\Desktop\数据仿真\sim_output_air_air_crossroads_smooth_100t_500k\fusion_wide_input'; % 数据目录
cfg.delimiter = 'auto';                                                       % 文本分隔符，auto为自动识别
cfg.processing_framework = 'joint_2d3d';                                     % joint_2d3d或legacy_active3d

%% 2. 数据读取范围与并行加载
cfg.read_percent = 100;                 % 读取数据百分比
cfg.read_start_percent = 0;             % 起始位置百分比
cfg.read_end_percent = [];              % 结束位置百分比，空值沿用read_percent
cfg.max_rows = inf;                     % 每个文件最大读取行数
cfg.max_targets_per_row = 192;          % 单行最多解析目标数
cfg.time_range_s = [];                  % 时间范围[s]，空值表示不限制
cfg.parallel_file_loading = false;      % 是否并行读取多个文件
cfg.parallel_file_workers = 0;          % 并行工作进程数，0为自动

%% 3. 输入文件列定义
% 3.1 主动雷达量测
cfg.active.time_col = 1;                % 时间列
cfg.active.count_col = 5;               % 目标数量列
cfg.active.target_id_col = 6;           % 首个目标编号列
cfg.active.valid_cols = [12, 13, 14];   % 首目标有效标志列
cfg.active.az_col = 19;                 % 首目标方位角列
cfg.active.el_col = 20;                 % 首目标俯仰角列
cfg.active.range_col = 23;              % 首目标距离列
cfg.active.stride = 56;                 % 相邻目标字段跨度
cfg.active.angle_unit = 'mrad';         % 角度单位
cfg.active.time_format = 'hms';         % 时间格式

% 3.2 被动雷达量测
cfg.passive.time_col = 1;               % 时间列
cfg.passive.count_col = 5;              % 目标数量列
cfg.passive.target_id_col = 6;          % 首个目标编号列
cfg.passive.valid_cols = [33, 34];      % 首目标有效标志列
cfg.passive.az_col = 35;                % 首目标方位角列
cfg.passive.el_col = 36;                % 首目标俯仰角列
cfg.passive.stride = 76;                % 相邻目标字段跨度
cfg.passive.angle_unit = 'mrad';        % 角度单位
cfg.passive.time_format = 'hms';        % 时间格式

% 3.3 观测平台轨迹
cfg.platform.time_col = 1;              % 时间列
cfg.platform.lat_col = 2;               % 纬度列
cfg.platform.lon_col = 3;               % 经度列
cfg.platform.alt_col = 4;               % 高度列
cfg.platform.angle_unit = 'mrad';       % 经纬度角度单位
cfg.platform.time_format = 'hms';       % 时间格式
cfg.platform_max_extrapolation_s = 0.1; % 平台端点线性外推上限[s]，另限一个采样周期；0禁用

%% 4. 时间配准、坐标原点与量测凝聚
cfg.frame_time_window_s = 0.1;        % 同帧时间容差[s]
cfg.local_origin = 'first_platform';    % 本地坐标原点选取方式
cfg.condense_enable = true;            % 是否启用主动量测凝聚
cfg.condense_method = 'spatiotemporal'; % 凝聚方法
cfg.condense_radius_m = 100;            % 空间凝聚半径[m]
cfg.condense_res_range_m = 100;         % 距离分辨率[m]
cfg.condense_res_az_deg = 0.2;          % 方位角分辨率[deg]
cfg.condense_res_el_deg = 0.2;          % 俯仰角分辨率[deg]
cfg.condense_gate_gamma = 9;            % 已有凝聚中心门限
cfg.condense_birth_gamma = 6;           % 新生凝聚中心门限
cfg.condense_birth_merge_enabled = false; % 是否合并无历史支撑的新生凝聚点（默认保护目标数）
cfg.condense_amax = 10;                 % 凝聚运动最大加速度[m/s^2]
cfg.condense_vel_beta = 0.15;           % 凝聚速度更新系数
cfg.condense_coast = 3;                 % 凝聚中心最大滑行帧数
cfg.condense_protect_diff_id = false;   % 已弃用兼容字段；目标编号不参与凝聚

%% 5. 统一二维/三维事件组织
cfg.joint_scan_tolerance_s = 0.015;                % 同一扫描归并容差[s]
cfg.joint_sync_tolerance_s = 0.010;                % 主被动同步容差[s]
cfg.joint_shard_dedup_enabled = true;              % 仅去除跨文件近完全相同的重传记录
cfg.joint_shard_duplicate_angle_deg = 0.02;        % 重传检测角度上限[deg]，实际另限1e-3deg
cfg.joint_shard_duplicate_range_m = 30;            % 主动重传距离门限[m]

%% 6. 统一逻辑航迹关联与生命周期
cfg.joint_confirm_M = 5;                 % 航迹确认所需命中数M
cfg.joint_confirm_N = 8;                 % 航迹确认统计窗口N
cfg.joint_gate_2d = 13.8155;             % 二维角度关联卡方门限
cfg.joint_gate_3d = 16.2662;             % 三维空间关联卡方门限
cfg.joint_unmatched_cost = 50;           % 未匹配虚拟分配代价
cfg.joint_birth_explain_nis = 1.0;       % 新生量测解释NIS门限
cfg.joint_tentative_timeout_s = 3;       % 暂态航迹静默超时[s]
cfg.joint_confirmed_timeout_s = 12;      % 确认航迹静默超时[s]
cfg.max_tracks = 400;                    % 全系统最大航迹数

%% 7. 统一滤波运动模型
% 7.1 二维角度IMM模型
cfg.joint_angle_q_cv = 0.08;                    % 角度CV模型过程噪声强度
cfg.joint_angle_q_ca = 0.50;                    % 角度CA模型过程噪声强度
cfg.joint_angle_rate_birth_std_dps = 2.0;       % 新生角速度标准差[deg/s]
cfg.joint_angle_acc_birth_std_dps2 = 3.0;       % 新生角加速度标准差[deg/s^2]

% 7.2 三维空间IMM模型
cfg.joint_space_sigma_a_cv = 12;                % 空间CV模型加速度噪声[m/s^2]
cfg.joint_space_sigma_j_ca = 15;                % 空间CA模型加加速度噪声[m/s^3]
cfg.joint_space_vel_birth_std_mps = 500;        % 新生速度标准差[m/s]
cfg.joint_space_acc_birth_std_mps2 = 80;        % 新生加速度标准差[m/s^2]

% 7.3 IMM转移概率
cfg.joint_imm_cv_stay = 0.96;                   % CV模型保持概率
cfg.joint_imm_ca_stay = 0.94;                   % CA模型保持概率
cfg.joint_imm_cv_probability = 0.65;            % 新生航迹CV初始概率

%% 8. 二维/三维模式切换与质量控制
cfg.joint_angle95_max_deg = 1.0;                % 二维角度95%误差上限[deg]
cfg.joint_3d_birth_M = 3;                       % 三维航迹新生所需命中数M
cfg.joint_3d_birth_N = 5;                       % 三维航迹新生统计窗口N
cfg.joint_3d_upgrade_M = 2;                     % 二维升级三维所需命中数M
cfg.joint_3d_upgrade_N = 3;                     % 二维升级三维统计窗口N
cfg.joint_up_consecutive = 1;                   % 升级所需连续满足次数
cfg.joint_down_consecutive = 3;                 % 降级所需连续异常次数
cfg.joint_switch_gate_2d = 9.2103;              % 模式切换角度一致性门限
cfg.joint_switch_cov_inflate = 2.0;             % 二维/三维一致性检验协方差尺度
% 三维正式输出仅由以下四个核心阈值控制。径向量采用1-sigma标准差，
% 距离龄期从最近一次真正完成三维距离约束更新的量测实际时刻起算。
cfg.joint_radial_sigma_warn_m = 5000;            % 径向距离不确定度预警阈值[m]
cfg.joint_radial_sigma_drop_m = 10000;           % 径向距离不确定度正式降级阈值[m]
cfg.joint_range_age_warn_s = 3;                  % 有效距离信息龄期预警阈值[s]
cfg.joint_range_age_drop_s = 10;                 % 有效距离信息龄期正式降级阈值[s]

% 以下旧质量阈值仅为配置兼容和诊断标尺保留，不再参与3D->2D决策。
cfg.joint_pos95_warn_m = 15000;                  % 三维位置95%诊断阈值[m]
cfg.joint_pos95_down_m = 30000;                  % 三维位置95%诊断阈值[m]
cfg.joint_pos95_recover_m = 10000;               % 三维位置95%诊断阈值[m]
cfg.joint_radial95_warn_m = 10000;               % 径向95%诊断阈值[m]
cfg.joint_radial95_down_m = 20000;               % 径向95%诊断阈值[m]
cfg.joint_radial95_recover_m = 7000;             % 径向95%诊断阈值[m]
cfg.joint_relative_range_down = 0.60;            % 相对距离不确定度诊断阈值
cfg.joint_space_nis_window = 5;                  % 空间NIS诊断统计窗口长度
cfg.joint_space_nis_recover = 1.5;               % 空间NIS诊断阈值
cfg.joint_space_nis_warn = 2.5;                  % 空间NIS诊断阈值
cfg.joint_space_nis_down = 4.0;                  % 空间NIS诊断阈值

cfg.joint_shadow_max_s = 20;                     % 影子状态最大保留时间[s]
cfg.joint_mode_3d_prior_cost = 0.5;              % 三维模式关联优先代价
% 以下三项只用于 joint_2d3d 的暂态-暂态航迹去重；已确认航迹不自动合并。
cfg.joint_merge_angle_deg = 0.08;                % 联合暂态航迹合并角度门限[deg]
cfg.joint_merge_rate_dps = 1.0;                  % 联合暂态航迹合并角速度门限[deg/s]
cfg.joint_merge_nis = 13.2767;                   % 联合暂态航迹合并NIS门限
cfg.joint_max_predict_dt_s = 5;                  % 单次预测最大时间间隔[s]
cfg.joint_history_level = 'output';              % output/diagnostic/full，控制历史内存

%% 9. 传感器量测噪声
cfg.sigma_range_m = 150;                         % 主动距离标准差[m]
cfg.sigma_az_deg = 0.08;                         % 主动方位角标准差[deg]
cfg.sigma_el_deg = 0.06;                         % 主动俯仰角标准差[deg]
cfg.sigma_passive_az_deg = 0.05;                 % 被动方位角标准差[deg]
cfg.sigma_passive_el_deg = 0.04;                 % 被动俯仰角标准差[deg]

%% 10. 旧三维模式的被动角度更新
cfg.passive_bearing_enabled = true;                    % 是否启用被动角度更新
cfg.passive_bearing_gate = 16;                         % 被动角度关联门限
cfg.passive_bearing_nis_gate = 16;                     % 被动角度更新NIS门限
cfg.passive_bearing_weight_gain = 0.5;                 % 被动更新权重增益
cfg.passive_bearing_confirm_hit = false;               % 被动命中是否参与确认
cfg.passive_bearing_update_on_active = true;           % 主动事件中是否融合被动量测
cfg.passive_bearing_update_on_pure = true;             % 纯被动事件中是否更新航迹
cfg.passive_bearing_update_active_hit_tracks = false;  % 是否更新本帧主动命中航迹
cfg.passive_bearing_min_dt_s = 0.15;                   % 同航迹被动更新最小间隔[s]
cfg.passive_bearing_fast_gate_deg = 2.5;               % 被动角度快速门限[deg]
cfg.passive_meas_fuse_enabled = true;                  % 是否融合多条被动量测
cfg.passive_meas_fuse_weight = 'likelihood';           % 被动量测融合权重方式
cfg.passive_meas_fuse_R_scale = 1.5;                   % 被动融合协方差缩放倍数
cfg.passive_meas_fuse_max_count = 4;                   % 单次最多融合量测数
cfg.passive_meas_fuse_max_spread_deg = 0.35;           % 融合量测最大角度离散[deg]
cfg.passive_meas_fuse_amb_ratio = 1.5;                 % 被动关联模糊度比值门限
cfg.passive_meas_fuse_amb_abs_nis = 0.5;               % 被动关联最小NIS差值
cfg.async_microbatch_dt_s = 0.005;                     % 异步事件微批宽度[s]
cfg.async_active_time_mode = 'frame';                  % 主动事件时间组织方式
cfg.async_passive_count_miss = false;                  % 被动事件是否计为主动漏检

%% 11. 旧三维CKF模型与自适应量测噪声
cfg.x_dim = 9;                                  % 三维CA状态维数
cfg.z_dim = 3;                                  % 笛卡尔量测维数
cfg.sigma_a_cs = 25;                            % 当前统计模型加速度噪声
cfg.alpha_cs = 0.4;                             % 当前统计模型机动频率
cfg.a_max_cs = 120;                             % 当前统计模型最大加速度[m/s^2]
cfg.adapt_R_enabled = true;                     % 是否自适应估计量测协方差
cfg.adapt_R_alpha = 0.15;                       % 自适应协方差更新系数
cfg.adapt_R_window = 200;                       % 自适应协方差统计窗口
cfg.adapt_R_min_samples = 100;                  % 启用自适应估计的最少样本数
cfg.R_default_diag = [2500^2, 2500^2, 2500^2]; % 默认笛卡尔量测方差[m^2]
cfg.R_min_diag = [200^2, 200^2, 200^2];         % 笛卡尔量测方差下限[m^2]
cfg.P_S = 0.99;                                 % 航迹生存概率
cfg.P_D = 0.70;                                 % 主动雷达检测概率
cfg.missed_weight_decay = 1e-28;                % 漏检时航迹权重衰减系数

%% 12. 旧三维关联与航迹管理
cfg.gating_gamma = 50;                   % 统计关联门限
cfg.assoc_pos_gate_m = 4500;             % 位置预关联门限[m]
cfg.active_pre_gate_enabled = true;     % 是否启用主动位置预门控
cfg.cost_unmatched = 150;                % 未匹配虚拟分配代价
cfg.M_confirm = 3;                       % 航迹确认所需命中数M
cfg.N_confirm = 5;                       % 航迹确认统计窗口N
cfg.prune_threshold = 1e-12;             % 航迹权重删除阈值
cfg.P_pos_birth = 1e6;                   % 新生位置方差[m^2]
cfg.P_vel_birth = 500^2;                 % 新生速度方差[(m/s)^2]
cfg.P_acc_birth = 80^2;                  % 新生加速度方差[(m/s^2)^2]

% 以下三项仅用于 legacy_active3d，不控制默认 joint_2d3d 框架。
cfg.birth_guard_m = 1;                   % 旧三维新生航迹空间抑制半径[m]
cfg.birth_suppress_gated = true;         % 旧三维是否抑制已被门控解释的量测新生
cfg.merge_pos_dist_m = 100;              % 旧三维暂态-暂态航迹合并位置门限[m]，已确认航迹不自动合并

cfg.max_coast_frames = 10;               % 最大连续滑行帧数
cfg.tentative_max_miss = 8;              % 暂态航迹最大连续漏检数
cfg.track_timeout_enabled = true;        % 是否启用静默超时删除
cfg.tentative_max_silence_s = 3.0;       % 暂态航迹静默超时[s]
cfg.confirmed_max_silence_s = 10;        % 确认航迹静默超时[s]
cfg.weight_floor_confirmed = 0.55;       % 确认航迹权重下限
cfg.hit_weight_gain = 0.08;              % 命中时航迹权重增量
cfg.coast_acc_decay = 0.30;              % 滑行时加速度衰减系数
cfg.nis_gate = 16;                       % 滤波更新NIS门限
cfg.nis_max_bad = 4;                     % 连续异常NIS最大次数
cfg.assoc_accept_nis = 16;               % 关联结果接受NIS门限
cfg.dedup_vel_angle_deg = 30;            % 航迹去重速度夹角门限[deg]
cfg.dedup_min_speed = 20;                % 航迹去重最低速度[m/s]
cfg.vinit_baseline_s = 0.2;              % 速度初始化最小时间基线[s]
cfg.vinit_min_disp_m = 80;               % 速度初始化最小位移[m]

%% 13. 旧三维量测融合与失捕重获
cfg.meas_fuse_enabled = true;            % 是否融合多部主动雷达量测
cfg.meas_fuse_weight = 'likelihood';     % 主动量测融合权重方式
cfg.meas_fuse_R_scale = 2.0;             % 主动融合协方差缩放倍数
cfg.meas_fuse_cluster_gamma = 35;        % 主动量测聚类统计门限
cfg.meas_fuse_cluster_dist_m = 3000;     % 主动量测聚类距离门限[m]

cfg.meas_robust_enabled = true;          % 是否启用鲁棒量测更新
cfg.robust_k = 2.0;                      % 鲁棒权重转折系数
cfg.robust_R_max_infl = 4;               % 鲁棒协方差最大膨胀倍数
cfg.reacq_P_inflate = 15;                % 重获时状态协方差膨胀倍数
cfg.reacq_vel_inflate = 5;               % 重获时速度协方差额外膨胀倍数

%% 14. 旧三维速度趋势辅助
cfg.vel_trend_enabled = true;            % 是否启用速度趋势估计
cfg.vel_trend_window = 50;               % 趋势估计最大历史点数
cfg.vel_trend_span_s = 1.0;              % 趋势估计时间跨度[s]
cfg.vel_trend_min_n = 10;                % 趋势估计最少样本数
cfg.vel_trend_blend = 0.55;              % 趋势速度融合比例
cfg.vel_trend_min_speed = 20;            % 启用趋势估计最低速度[m/s]
cfg.vel_trend_conf_only = true;          % 是否仅用于确认航迹

%% 15. 旧三维目标编号先验与IMM
cfg.use_target_id_prior = false;          % 是否使用目标编号关联先验
cfg.id_prior_cost_bonus = 120;           % 编号匹配关联代价奖励
cfg.id_dup_dist_m = 5000;                % 同编号航迹去重距离[m]
cfg.id_prior_min_conf = 1;               % 编号先验最小置信度
cfg.id_prior_conf_max = 100;             % 编号先验最大置信度

cfg.use_imm = false;                     % 是否启用旧三维IMM
cfg.imm_p_cv_stay = 0.95;                % 旧IMM的CV保持概率
cfg.imm_p_ca_stay = 0.95;                % 旧IMM的CA保持概率
cfg.imm_mu_init_cv = 0.5;                % 旧IMM的CV初始概率
cfg.imm_cv_sigma_a = 5;                  % 旧IMM的CV加速度噪声[m/s^2]
cfg.imm_cv_acc_floor = 1e-3;             % 旧IMM的CV加速度状态下限

%% 16. 绘图与结果保存
cfg.do_plot = true;                      % 是否绘制处理结果
cfg.joint_plot_min_life = 3;             % 分维度绘图最少输出点数
cfg.plot_save_dir = '';                  % 图片保存目录，空值表示不自动保存
cfg.result_save_file = '';               % 结果MAT文件，空值表示不自动保存

%% 17. 评价指标与参考真值
cfg.metrics_enabled = true;                    % 是否计算评价指标
cfg.metrics_max_print = 12;                    % 最多打印目标指标数
cfg.track_accuracy_purity_th = 0.99;            % 正确关联数/匹配真值对应范围全部量测数；3D为RAE，2D为进入二维的被动AE
cfg.track_accuracy_min_assoc = 3;              % 精度统计最少关联点数

cfg.truth_file = '..\truth_100_targets_500k.csv'; % 真实真值CSV；空值时可自动发现
cfg.truth_auto_discover = true;                % truth_file不可用时在数据目录及父目录查找truth*.csv
cfg.truth_altitude_is_absolute = true;         % alt_m为绝对海拔，评价时减本地原点高度
cfg.truth_real_time_tolerance_s = 0.03;        % 正式输出与真实真值最近时刻容差[s]
cfg.truth_time_origin_s = [];                 % time_s零点的事件时钟秒数；空时由真值time/time_s配对推算，否则取完整平台首时刻

cfg.truth_id_split_enabled = true;             % 是否按空间间断拆分同编号真值
cfg.truth_cross_sensor_id_consistent = false;  % 主被动原始编号是否确认属于同一编号空间
cfg.truth_id_split_dist_m = 15000;             % 真值编号拆分距离门限[m]
cfg.truth_id_split_max_gap_s = 60;             % 真值编号拆分时间门限[s]
cfg.truth_assoc_map_max_dist_m = inf;           % 真值关联最大空间距离[m]
cfg.truth_cross_sensor_match_angle_deg = 1.0;  % 主被动片段几何匹配角度门限[deg]
cfg.truth_cross_sensor_match_max_dt_s = 0.5;   % 主被动片段几何匹配最大时差[s]
cfg.truth_cross_sensor_match_min_points = 3;   % 几何匹配最少有效样本数
cfg.truth_cross_sensor_match_min_ratio = 0.20; % 几何匹配最少通过比例
cfg.truth_cross_sensor_match_ambiguity_deg = 0.10; % 最优/次优匹配最小角度差[deg]
cfg.truth_cross_sensor_match_max_samples = 200; % 单片段最多抽样点数

cfg.truth_rts_meas_std_m = 1000;               % 真值RTS量测标准差[m]
cfg.truth_rts_proc_std_mps2 = 10;               % 真值RTS过程噪声[m/s^2]
cfg.truth_rms_time_tolerance_s = cfg.frame_time_window_s; % 真值时间匹配容差[s]

%% 18. 航迹平滑
cfg.do_smoothing = false;               % 是否启用航迹后处理平滑
cfg.smooth_method = 'fixedlag';         % 平滑方法
cfg.smooth_lag = 8;                     % 固定滞后步数
cfg.smooth_min_life = 10;               % 执行平滑的最短航迹长度
cfg.smooth_meas_std = 30;                % 平滑器量测标准差[m]
cfg.smooth_proc_std = 3;                 % 平滑器过程噪声标准差[m/s^2]
cfg.smooth_show_axes = false;            % 是否显示分轴平滑对比图
end
