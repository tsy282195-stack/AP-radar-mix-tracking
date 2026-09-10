function [events, info, legacy] = build_joint_measurement_events(frames, cfg)
%BUILD_JOINT_MEASUREMENT_EVENTS Build ordered active/passive scan events.
%
% Every input file is retained as an independent observation source. Records
% are grouped into modality scans, then active/passive scans inside the
% synchronization tolerance are combined into one logical event. Optional
% cross-file de-duplication only removes near-identical retransmissions and
% never uses target_id. Consumers process each source independently while
% counting lifecycle evidence once per logical event.

scan_tol = get_cfg(cfg, 'joint_scan_tolerance_s', cfg.frame_time_window_s);
sync_tol = get_cfg(cfg, 'joint_sync_tolerance_s', min(cfg.frame_time_window_s, 0.010));

[act, pas] = collect_measurements(frames, cfg);
act_blocks = make_blocks(act.t, scan_tol, 1);
pas_blocks = make_blocks(pas.t, scan_tol, 2);
blocks = [act_blocks, pas_blocks];
if isempty(blocks)
    events = empty_events();
    info = make_info(0, numel(act.t), numel(pas.t), 0, 0, scan_tol, sync_tol);
    legacy = make_legacy(events, info);
    return;
end

[~, ord] = sort([blocks.t]);
blocks = blocks(ord);
groups = pair_sensor_blocks(blocks, sync_tol);

events = repmat(event_template(), numel(groups), 1);
n_act_raw = 0; n_pas_raw = 0; n_act_out = 0; n_pas_out = 0;
for k = 1:numel(groups)
    b = blocks(groups{k});
    ia = zeros(1, 0); ip = zeros(1, 0);
    for q = 1:numel(b)
        if b(q).type == 1
            ia = [ia, b(q).idx]; %#ok<AGROW>
        else
            ip = [ip, b(q).idx]; %#ok<AGROW>
        end
    end
    n_act_raw = n_act_raw + numel(ia);
    n_pas_raw = n_pas_raw + numel(ip);
    if get_cfg(cfg, 'joint_shard_dedup_enabled', true)
        [ia, ga] = dedup_active_indices(act, ia, cfg);
        [ip, gp] = dedup_passive_indices(pas, ip, cfg);
        act = fuse_active_groups(act, ia, ga);
        pas = fuse_passive_groups(pas, ip, gp);
    end
    n_act_out = n_act_out + numel(ia);
    n_pas_out = n_pas_out + numel(ip);

    e = event_template();
    e.cycle_id = k;
    e.has_active = ~isempty(ia);
    e.has_passive = ~isempty(ip);
    e.t_start = min([b.t_start]);
    e.t_end = max([b.t_end]);
    if e.has_active
        e.t_sec = median(act.t(ia));
    else
        e.t_sec = median(pas.t(ip));
    end
    e.active.t_sec = act.t(ia);
    e.active.xyz = act.xyz(:, ia);
    e.active.rae = act.rae(:, ia);
    e.active.R_xyz = act.R_xyz(:, :, ia);
    e.active.R_ae = act.R_ae(:, :, ia);
    e.active.has_range = act.has_range(ia);
    e.active.ids = act.ids(ia);
    e.active.src = act.src(ia);
    e.active.n_meas = numel(ia);
    e.passive.t_sec = pas.t(ip);
    e.passive.ang = pas.ang(:, ip);
    e.passive.R_ae = pas.R_ae(:, :, ip);
    e.passive.ids = pas.ids(ip);
    e.passive.src = pas.src(ip);
    e.passive.n_meas = numel(ip);
    events(k) = e;
end

info = make_info(numel(events), numel(act.t), numel(pas.t), ...
    n_act_raw - n_act_out, n_pas_raw - n_pas_out, scan_tol, sync_tol);
legacy = make_legacy(events, info);

fprintf('\n========== 统一扫描事件组织 ==========\n');
fprintf('  事件=%d, 主动点=%d, 被动点=%d, 扫描容差=%.4fs, 同步容差=%.4fs\n', ...
    info.n_events, info.n_active, info.n_passive, scan_tol, sync_tol);
    fprintf('  跨文件近重复凝聚: 主动=%d, 被动=%d; 组合事件=%d\n', ...
    info.n_active_duplicates, info.n_passive_duplicates, ...
    sum([events.has_active] & [events.has_passive]));
end

function [act, pas] = collect_measurements(frames, cfg)
n_act = 0; n_pas = 0;
for k = 1:numel(frames)
    n_act = n_act + size(frames(k).active_xyz, 2);
    if isfield(frames(k), 'active_ae_only')
        n_act = n_act + size(frames(k).active_ae_only, 2);
    end
    n_pas = n_pas + size(frames(k).passive_ang, 2);
end
act.t = zeros(1, n_act); act.xyz = zeros(3, n_act); act.rae = zeros(3, n_act);
act.R_xyz = zeros(3, 3, n_act); act.R_ae = zeros(2, 2, n_act);
act.ids = nan(1, n_act); act.src = nan(1, n_act); act.has_range = false(1, n_act);
pas.t = zeros(1, n_pas); pas.ang = zeros(2, n_pas); pas.R_ae = zeros(2, 2, n_pas);
pas.ids = nan(1, n_pas); pas.src = nan(1, n_pas);
Ra = diag([cfg.sigma_az_deg^2, cfg.sigma_el_deg^2]);
Rp = diag([cfg.sigma_passive_az_deg^2, cfg.sigma_passive_el_deg^2]);
pa = 0; pp = 0;
for k = 1:numel(frames)
    M = size(frames(k).active_xyz, 2);
    if M > 0
        ii = pa + (1:M);
        act.t(ii) = normalize_row(field_or(frames(k), 'active_t', []), M, frames(k).t_sec);
        act.xyz(:, ii) = frames(k).active_xyz;
        if isfield(frames(k), 'active_rae') && size(frames(k).active_rae, 2) >= M
            act.rae(:, ii) = frames(k).active_rae(:, 1:M);
        else
            act.rae(:, ii) = nan(3, M);
        end
        if isfield(frames(k), 'active_R') && size(frames(k).active_R, 3) >= M
            act.R_xyz(:, :, ii) = frames(k).active_R(:, :, 1:M);
        else
            act.R_xyz(:, :, ii) = repmat(diag(cfg.R_default_diag), 1, 1, M);
        end
        act.R_ae(:, :, ii) = repmat(Ra, 1, 1, M);
        act.has_range(ii) = true;
        act.ids(ii) = normalize_row(field_or(frames(k), 'target_ids', []), M, NaN);
        act.src(ii) = normalize_row(field_or(frames(k), 'active_src', []), M, NaN);
        pa = pa + M;
    end
    if isfield(frames(k), 'active_ae_only')
        M = size(frames(k).active_ae_only, 2);
    else
        M = 0;
    end
    if M > 0
        ii = pa + (1:M);
        act.t(ii) = normalize_row(field_or(frames(k), 'active_ae_only_t', []), M, frames(k).t_sec);
        act.xyz(:, ii) = nan(3, M);
        act.rae(:, ii) = [nan(1, M); frames(k).active_ae_only];
        act.R_xyz(:, :, ii) = nan(3, 3, M);
        act.R_ae(:, :, ii) = repmat(Ra, 1, 1, M);
        act.ids(ii) = normalize_row(field_or(frames(k), 'active_ae_only_ids', []), M, NaN);
        act.src(ii) = normalize_row(field_or(frames(k), 'active_ae_only_src', []), M, NaN);
        pa = pa + M;
    end
    M = size(frames(k).passive_ang, 2);
    if M > 0
        ii = pp + (1:M);
        pas.t(ii) = normalize_row(field_or(frames(k), 'passive_t', []), M, frames(k).t_sec);
        pas.ang(:, ii) = frames(k).passive_ang;
        pas.R_ae(:, :, ii) = repmat(Rp, 1, 1, M);
        pas.ids(ii) = normalize_row(field_or(frames(k), 'passive_ids', []), M, NaN);
        pas.src(ii) = normalize_row(field_or(frames(k), 'passive_src', []), M, NaN);
        pp = pp + M;
    end
end
good = isfinite(act.t) & all(isfinite(act.rae(2:3, :)), 1) & ...
    (~act.has_range | all(isfinite(act.xyz), 1));
act = subset_measurements(act, good);
good = isfinite(pas.t) & all(isfinite(pas.ang), 1);
pas = subset_measurements(pas, good);
end

function s = subset_measurements(s, keep)
names = fieldnames(s);
for i = 1:numel(names)
    name = names{i}; v = s.(name);
    if ndims(v) == 3
        s.(name) = v(:, :, keep);
    elseif size(v, 1) > 1 && size(v, 2) == numel(keep)
        s.(name) = v(:, keep);
    else
        s.(name) = v(keep);
    end
end
end

function blocks = make_blocks(t, tol, type)
blocks = struct('type', {}, 't', {}, 't_start', {}, 't_end', {}, 'idx', {});
if isempty(t), return; end
[ts, ord] = sort(t);
i = 1;
while i <= numel(ts)
    j = i;
    while j < numel(ts) && ts(j + 1) - ts(i) <= tol
        j = j + 1;
    end
    q = numel(blocks) + 1;
    blocks(q).type = type;
    blocks(q).t = median(ts(i:j));
    blocks(q).t_start = ts(i);
    blocks(q).t_end = ts(j);
    blocks(q).idx = ord(i:j);
    i = j + 1;
end
end

function groups = pair_sensor_blocks(blocks, sync_tol)
% Pair opposite-sensor scans globally by timestamp, then retain unpaired scans.
ia = find([blocks.type] == 1);
ip = find([blocks.type] == 2);
pairs = zeros(0, 2);
if ~isempty(ia) && ~isempty(ip)
    dt = [blocks(ip).t] - [blocks(ia).t].';
    C = abs(dt);
    % The joint filter applies active before passive inside a paired event.
    % Pair only causal combinations; an earlier passive scan remains its
    % own event instead of being shifted forward to the active timestamp.
    C(dt < -1e-9) = inf;
    C(C > sync_tol) = inf;
    unmatched = sync_tol + max(eps(max(sync_tol, 1)), 1e-12);
    pairs = solve_global_assignment(C, unmatched);
end

used_a = false(size(ia)); used_p = false(size(ip));
groups = cell(1, size(pairs, 1) + numel(ia) + numel(ip));
n = 0;
for q = 1:size(pairs, 1)
    n = n + 1;
    groups{n} = [ia(pairs(q, 1)), ip(pairs(q, 2))];
    used_a(pairs(q, 1)) = true; used_p(pairs(q, 2)) = true;
end
for q = find(~used_a)
    n = n + 1; groups{n} = ia(q);
end
for q = find(~used_p)
    n = n + 1; groups{n} = ip(q);
end
groups = groups(1:n);

processing_time = zeros(1, n);
for q = 1:n
    b = blocks(groups{q});
    j = find([b.type] == 1, 1);
    if isempty(j), processing_time(q) = b(1).t; else, processing_time(q) = b(j).t; end
end
[~, ord] = sort(processing_time);
groups = groups(ord);
end

function [keep, groups] = dedup_active_indices(a, idx, cfg)
    [keep, groups] = dedup_indices(a.rae(2:3, :), a.src, idx, ...
        get_cfg(cfg, 'joint_shard_duplicate_angle_deg', 0.02), ...
        a.rae(1, :), get_cfg(cfg, 'joint_shard_duplicate_range_m', 30));
end

function [keep, groups] = dedup_passive_indices(p, idx, cfg)
    [keep, groups] = dedup_indices(p.ang, p.src, idx, ...
        get_cfg(cfg, 'joint_shard_duplicate_angle_deg', 0.02), [], inf);
end

function [keep, groups] = dedup_indices(ang, src, idx, gate_deg, range, range_gate)
keep = zeros(1, 0);
groups = cell(1, 0);
for q = reshape(idx, 1, [])
    group_index = 0;
    for g = 1:numel(keep)
        r = keep(g);
        different_shard = isfinite(src(q)) && isfinite(src(r)) && src(q) ~= src(r);
        da = hypot(angle_diff(ang(1, q), ang(1, r)), ang(2, q) - ang(2, r));
        near_exact = da <= min(gate_deg, 1e-3);
        range_ok = isempty(range) || (all(isfinite([range(q), range(r)])) && ...
            abs(range(q) - range(r)) <= range_gate);
        duplicate = different_shard && range_ok && near_exact;
        if duplicate, group_index = g; break; end
    end
    if group_index == 0
        keep(end + 1) = q; %#ok<AGROW>
        groups{end + 1} = q; %#ok<AGROW>
    else
        groups{group_index}(end + 1) = q;
    end
end
end

function a = fuse_active_groups(a, keep, groups)
for g = 1:numel(groups)
    jj = groups{g}; r = keep(g);
    if numel(jj) < 2, continue; end
    [ae, Rae] = fuse_angles(a.rae(2:3, jj), a.R_ae(:, :, jj));
    a.rae(2:3, r) = ae; a.R_ae(:, :, r) = Rae;
    full = jj(a.has_range(jj) & all(isfinite(a.xyz(:, jj)), 1));
    if ~isempty(full)
        [xyz, Rxyz, w] = fuse_vectors(a.xyz(:, full), a.R_xyz(:, :, full));
        a.xyz(:, r) = xyz; a.R_xyz(:, :, r) = Rxyz;
        a.rae(1, r) = sum(w .* a.rae(1, full));
        a.has_range(r) = true;
    else
        a.xyz(:, r) = NaN; a.R_xyz(:, :, r) = NaN;
        a.rae(1, r) = NaN; a.has_range(r) = false;
    end
    a.t(r) = median(a.t(jj)); a.ids(r) = representative_id(a.ids(jj));
end
end

function p = fuse_passive_groups(p, keep, groups)
for g = 1:numel(groups)
    jj = groups{g}; r = keep(g);
    if numel(jj) < 2, continue; end
    [p.ang(:, r), p.R_ae(:, :, r)] = fuse_angles(p.ang(:, jj), p.R_ae(:, :, jj));
    p.t(r) = median(p.t(jj)); p.ids(r) = representative_id(p.ids(jj));
end
end

function [z, Rmix] = fuse_angles(Z, R)
n = size(Z, 2); w = covariance_weights(R);
z = [atan2d(sum(w .* sind(Z(1, :))), sum(w .* cosd(Z(1, :)))); ...
    sum(w .* Z(2, :))];
Rmix = zeros(2);
for i = 1:n
    d = [angle_diff(Z(1, i), z(1)); Z(2, i) - z(2)];
    Rmix = Rmix + w(i) * (R(:, :, i) + d * d');
end
Rmix = make_spd(Rmix);
end

function [z, Rmix, w] = fuse_vectors(Z, R)
n = size(Z, 2); w = covariance_weights(R); z = Z * w(:);
Rmix = zeros(size(R, 1));
for i = 1:n
    d = Z(:, i) - z;
    Rmix = Rmix + w(i) * (R(:, :, i) + d * d');
end
Rmix = make_spd(Rmix);
end

function w = covariance_weights(R)
n = size(R, 3); score = zeros(1, n);
for i = 1:n
    score(i) = 1 / max(trace(R(:, :, i)), realmin);
end
if any(~isfinite(score)) || sum(score) <= 0
    w = ones(1, n) / n;
else
    w = score / sum(score);
end
end

function id = representative_id(ids)
finite_ids = ids(isfinite(ids));
if isempty(finite_ids), id = NaN; else, id = mode(finite_ids); end
end

function A = make_spd(A)
A = 0.5 * (A + A');
[V, D] = eig(A); d = max(real(diag(D)), 1e-12);
A = real(V * diag(d) * V'); A = 0.5 * (A + A');
end

function events = empty_events()
events = repmat(event_template(), 0, 1);
end

function e = event_template()
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'has_active', false, 'has_passive', false, ...
    'active', active_template(), 'passive', passive_template());
end

function a = active_template()
a = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'R_xyz', zeros(3, 3, 0), 'R_ae', zeros(2, 2, 0), ...
    'has_range', false(1, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
end

function p = passive_template()
p = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), ...
    'src', zeros(1, 0), 'n_meas', 0);
end

function info = make_info(n_events, n_active, n_passive, n_adup, n_pdup, scan_tol, sync_tol)
info = struct('mode', 'joint_2d3d', 'n_events', n_events, ...
    'n_active', n_active, 'n_passive', n_passive, ...
    'n_passive_bearing', n_passive, 'n_active_duplicates', n_adup, ...
    'n_passive_duplicates', n_pdup, 'scan_tolerance_s', scan_tol, ...
    'sync_tolerance_s', sync_tol);
end

function legacy = make_legacy(events, info)
K = numel(events);
legacy.xyz = cell(K, 1); legacy.R = cell(K, 1); legacy.ids = cell(K, 1);
legacy.passive_bearing = cell(K, 1); legacy.times = zeros(K, 1);
legacy.event_meta = repmat(struct('has_active', false, 'has_passive', false, ...
    'miss_cycle', true, 'confirm_cycle', true, 'n_active', 0, 'n_passive', 0, ...
    't_start', NaN, 't_end', NaN), K, 1);
for k = 1:K
    e = events(k);
    jj = find(e.active.has_range);
    legacy.xyz{k} = e.active.xyz(:, jj);
    legacy.R{k} = e.active.R_xyz(:, :, jj);
    legacy.ids{k} = e.active.ids(jj);
    legacy.times(k) = e.t_sec;
    legacy.passive_bearing{k} = struct('t_sec', e.t_sec, ...
        'ang_deg', e.passive.ang, 'R_deg2', e.passive.R_ae, ...
        'src', e.passive.src, 'tracklet_id', e.passive.ids, ...
        'n_meas', e.passive.n_meas);
    legacy.event_meta(k) = struct('has_active', e.has_active, ...
        'has_passive', e.has_passive, 'miss_cycle', true, ...
        'confirm_cycle', true, 'n_active', e.active.n_meas, ...
        'n_passive', e.passive.n_meas, 't_start', e.t_start, 't_end', e.t_end);
end
legacy.info = info;
end

function v = field_or(s, name, fallback)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = fallback; end
end

function v = normalize_row(v, n, fallback)
if isempty(v), v = fallback * ones(1, n); else, v = v(:).'; end
if numel(v) < n, v = [v, fallback * ones(1, n - numel(v))]; end
v = v(1:n);
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end
