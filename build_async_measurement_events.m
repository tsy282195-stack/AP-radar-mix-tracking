function [async_xyz, async_R, fusion_info, passive_bearing, async_ids, async_times, event_meta] = build_async_measurement_events(frames, cfg)
%BUILD_ASYNC_MEASUREMENT_EVENTS  Build event-driven heterogeneous filter input.
%
% Active measurements keep Cartesian ENU position and per-plot covariance.
% Passive measurements keep bearing-only az/el observations and their own
% timestamps. The function only groups close timestamps into micro-batches;
% it does not fuse active and passive measurements into a single 3-D point.

n_frames = numel(frames);
dt_batch = get_cfg_scalar(cfg, 'async_microbatch_dt_s', 0.005);
passive_count_miss = get_cfg_scalar(cfg, 'async_passive_count_miss', false) ~= 0;
active_time_mode = get_cfg_text(cfg, 'async_active_time_mode', 'frame');
keep_pure_passive = get_cfg_scalar(cfg, 'passive_bearing_update_on_pure', true) ~= 0 || passive_count_miss;

% ---------- collect active plots ----------
n_act = 0;
for k = 1:n_frames
    if isfield(frames, 'active_xyz')
        n_act = n_act + size(frames(k).active_xyz, 2);
    end
end
act_t = zeros(1, n_act);
act_xyz = zeros(3, n_act);
act_R = zeros(3, 3, n_act);
act_id = nan(1, n_act);
act_src = nan(1, n_act);

p = 0;
for k = 1:n_frames
    if ~isfield(frames, 'active_xyz'), continue; end
    Z = frames(k).active_xyz;
    M = size(Z, 2);
    if M == 0, continue; end
    idx = p + (1:M);
    act_xyz(:, idx) = Z;
    if strcmp(active_time_mode, 'plot')
        act_t(idx) = normalize_time_vector(get_frame_field(frames(k), 'active_t', []), M, frames(k).t_sec);
    else
        act_t(idx) = frames(k).t_sec * ones(1, M);
    end
    act_id(idx) = normalize_row(get_frame_field(frames(k), 'target_ids', []), M, NaN);
    act_src(idx) = normalize_row(get_frame_field(frames(k), 'active_src', []), M, NaN);
    Rk = get_active_R(frames(k), M, cfg);
    act_R(:, :, idx) = Rk;
    p = p + M;
end

% ---------- collect passive bearing events ----------
n_pas = 0;
for k = 1:n_frames
    if isfield(frames, 'passive_ang')
        n_pas = n_pas + size(frames(k).passive_ang, 2);
    end
end
pas_t = zeros(1, n_pas);
pas_ang = zeros(2, n_pas);
pas_R = zeros(2, 2, n_pas);
pas_src = nan(1, n_pas);
pas_id = nan(1, n_pas);

p = 0;
for k = 1:n_frames
    if ~isfield(frames, 'passive_ang'), continue; end
    A = frames(k).passive_ang;
    M = size(A, 2);
    if M == 0, continue; end
    idx = p + (1:M);
    pas_ang(:, idx) = A;
    pas_t(idx) = normalize_time_vector(get_frame_field(frames(k), 'passive_t', []), M, frames(k).t_sec);
    pas_src(idx) = normalize_row(get_frame_field(frames(k), 'passive_src', []), M, NaN);
    pas_id(idx) = normalize_row(get_frame_field(frames(k), 'passive_ids', []), M, NaN);
    Rb = default_passive_R(cfg);
    for q = idx
        pas_R(:, :, q) = Rb;
    end
    p = p + M;
end

% Drop invalid timestamps and measurements.
act_good = isfinite(act_t) & all(isfinite(act_xyz), 1);
act_t = act_t(act_good); act_xyz = act_xyz(:, act_good); act_R = act_R(:, :, act_good);
act_id = act_id(act_good); act_src = act_src(act_good);

pas_good = isfinite(pas_t) & all(isfinite(pas_ang), 1);
pas_t = pas_t(pas_good); pas_ang = pas_ang(:, pas_good); pas_R = pas_R(:, :, pas_good);
pas_src = pas_src(pas_good); pas_id = pas_id(pas_good);

n_act = numel(act_t);
n_pas = numel(pas_t);
all_t = [act_t, pas_t];
all_type = [ones(1, n_act), 2 * ones(1, n_pas)];
all_idx = [1:n_act, 1:n_pas];

if isempty(all_t)
    async_xyz = {};
    async_R = {};
    async_ids = {};
    passive_bearing = {};
    async_times = [];
    event_meta = struct('has_active', {}, 'has_passive', {}, 'miss_cycle', {}, ...
        'confirm_cycle', {}, 'n_active', {}, 'n_passive', {}, ...
        't_start', {}, 't_end', {});
    fusion_info = make_info(n_frames, n_act, n_pas, 0, dt_batch);
    fusion_info.async_active_time_mode = active_time_mode;
    fusion_info.keep_pure_passive_events = keep_pure_passive;
    return;
end

[all_t, ord] = sort(all_t);
all_type = all_type(ord);
all_idx = all_idx(ord);

groups = {};
i = 1;
while i <= numel(all_t)
    j = i;
    t0 = all_t(i);
    while j + 1 <= numel(all_t) && (all_t(j + 1) - t0) <= dt_batch
        j = j + 1;
    end
    groups{end + 1} = i:j; %#ok<AGROW>
    i = j + 1;
end

if ~keep_pure_passive
    keep_group = false(1, numel(groups));
    for g = 1:numel(groups)
        keep_group(g) = any(all_type(groups{g}) == 1);
    end
    groups = groups(keep_group);
end

K = numel(groups);
async_xyz = cell(K, 1);
async_R = cell(K, 1);
async_ids = cell(K, 1);
passive_bearing = cell(K, 1);
async_times = zeros(K, 1);
event_meta = repmat(struct('has_active', false, 'has_passive', false, ...
    'miss_cycle', false, 'confirm_cycle', false, 'n_active', 0, 'n_passive', 0, ...
    't_start', NaN, 't_end', NaN), K, 1);

for k = 1:K
    gi = groups{k};
    a_local = all_idx(gi(all_type(gi) == 1));
    p_local = all_idx(gi(all_type(gi) == 2));
    has_active = ~isempty(a_local);
    has_passive = ~isempty(p_local);
    if has_active
        t_rep = median(act_t(a_local));
    else
        t_rep = median(all_t(gi));
    end
    async_times(k) = t_rep;

    if has_active
        async_xyz{k} = act_xyz(:, a_local);
        async_R{k} = act_R(:, :, a_local);
        async_ids{k} = act_id(a_local);
    else
        async_xyz{k} = zeros(3, 0);
        async_R{k} = zeros(3, 3, 0);
        async_ids{k} = zeros(1, 0);
    end

    if has_passive
        passive_bearing{k} = struct( ...
            't_sec', t_rep, ...
            'ang_deg', pas_ang(:, p_local), ...
            'R_deg2', pas_R(:, :, p_local), ...
            'src', pas_src(p_local), ...
            'tracklet_id', pas_id(p_local), ...
            'n_meas', numel(p_local));
    else
        passive_bearing{k} = struct('t_sec', t_rep, 'ang_deg', zeros(2, 0), ...
            'R_deg2', zeros(2, 2, 0), 'src', zeros(1, 0), ...
            'tracklet_id', zeros(1, 0), 'n_meas', 0);
    end

    event_meta(k).has_active = has_active;
    event_meta(k).has_passive = has_passive;
    event_meta(k).miss_cycle = has_active || (has_passive && passive_count_miss);
    event_meta(k).confirm_cycle = has_active;
    event_meta(k).n_active = numel(a_local);
    event_meta(k).n_passive = numel(p_local);
    event_meta(k).t_start = all_t(gi(1));
    event_meta(k).t_end = all_t(gi(end));
end

fusion_info = make_info(n_frames, n_act, n_pas, K, dt_batch);
fusion_info.async_active_time_mode = active_time_mode;
fusion_info.keep_pure_passive_events = keep_pure_passive;
fprintf('  async_active_time_mode=%s\n', active_time_mode);
fprintf('  keep_pure_passive_events=%d\n', keep_pure_passive);
fprintf('\n========== 异步异构量测事件组织 ==========\n');
fprintf('  原始分帧: %d, 异步微批事件: %d, microbatch_dt=%.4fs\n', n_frames, K, dt_batch);
fprintf('  主动事件点: %d, 被动bearing-only事件点: %d\n', n_act, n_pas);
fprintf('  事件类型: 含主动=%d, 纯被动=%d, 主被动同批=%d\n', ...
    sum([event_meta.has_active]), ...
    sum(~[event_meta.has_active] & [event_meta.has_passive]), ...
    sum([event_meta.has_active] & [event_meta.has_passive]));
end

function info = make_info(n_frames, n_active, n_passive, n_events, dt_batch)
info = struct();
info.mode = 'async_hetero';
info.n_frames = n_frames;
info.n_events = n_events;
info.async_microbatch_dt_s = dt_batch;
info.n_active = n_active;
info.n_passive = n_passive;
info.n_passive_bearing = n_passive;
end

function v = get_frame_field(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_value;
end
end

function t = normalize_time_vector(v, n, fallback)
if isempty(v)
    t = fallback * ones(1, n);
else
    t = v(:).';
    if numel(t) < n
        t = [t, fallback * ones(1, n - numel(t))];
    elseif numel(t) > n
        t = t(1:n);
    end
end
end

function v = normalize_row(v, n, fill_value)
if isempty(v)
    v = fill_value * ones(1, n);
else
    v = v(:).';
    if numel(v) < n
        v = [v, fill_value * ones(1, n - numel(v))];
    elseif numel(v) > n
        v = v(1:n);
    end
end
end

function R = get_active_R(frame, n, cfg)
if isfield(frame, 'active_R') && size(frame.active_R, 1) == 3 && ...
        size(frame.active_R, 2) == 3 && size(frame.active_R, 3) >= n
    R = frame.active_R;
else
    R0 = diag(get_cfg_vector(cfg, 'R_default_diag', [4500^2, 4500^2, 4500^2]));
    R = repmat(R0, 1, 1, n);
end
for i = 1:n
    R(:, :, i) = make_spd_local(R(:, :, i));
end
end

function R = default_passive_R(cfg)
sig_az = get_cfg_scalar(cfg, 'sigma_passive_az_deg', 0.1);
sig_el = get_cfg_scalar(cfg, 'sigma_passive_el_deg', 0.1);
R = diag([sig_az^2, sig_el^2]);
R = make_spd_local(R);
end

function v = get_cfg_scalar(cfg, name, default_value)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name)) && isscalar(cfg.(name))
    v = cfg.(name);
else
    v = default_value;
end
end

function v = get_cfg_text(cfg, name, default_value)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
    if isa(v, 'string')
        v = char(v);
    end
    if ischar(v)
        v = lower(strtrim(v));
    else
        v = default_value;
    end
else
    v = default_value;
end
end

function v = get_cfg_vector(cfg, name, default_value)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
else
    v = default_value;
end
v = v(:).';
end

function A = make_spd_local(A)
A = 0.5 * (A + A');
[V, D] = eig(A);
d = max(diag(D), 1e-9);
A = V * diag(d) * V';
A = 0.5 * (A + A');
end
