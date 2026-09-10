function [z_fused, R_fused, info] = fuse_passive_bearing_group(ang_deg, R_deg2, assoc_cost, cfg)
%FUSE_PASSIVE_BEARING_GROUP  Fuse same-source bearings on the unit sphere.
%
% The LOS-vector mean avoids the azimuth wrap discontinuity at +/-180 deg.
% Covariance uses reliability-weighted information accumulation and a
% residual-consistency inflation. The configurable correlation scale is
% applied only when more than one bearing is fused.

if nargin < 4 || isempty(cfg)
    cfg = struct();
end
if isempty(ang_deg)
    z_fused = zeros(2, 0);
    R_fused = zeros(2, 2, 0);
    info = make_info(0);
    return;
end
if size(ang_deg, 1) ~= 2
    error('ang_deg 必须为 2xN 的 [az; el] 角度矩阵');
end

n = size(ang_deg, 2);
info = make_info(n);
assoc_cost = normalize_cost(assoc_cost, n);
R_all = normalize_covariance(R_deg2, n);

valid = all(isfinite(ang_deg), 1) & isfinite(assoc_cost);
for j = 1:n
    Rj = R_all(:, :, j);
    valid(j) = valid(j) && all(isfinite(Rj(:)));
end
if ~all(valid)
    ang_deg = ang_deg(:, valid);
    assoc_cost = assoc_cost(valid);
    R_all = R_all(:, :, valid);
end
n = size(ang_deg, 2);
info.n_valid = n;
if n == 0
    z_fused = zeros(2, 0);
    R_fused = zeros(2, 2, 0);
    return;
end

mode = get_cfg_text(cfg, 'passive_meas_fuse_weight', 'likelihood');
if strcmp(mode, 'equal')
    reliability = ones(1, n);
else
    delta = assoc_cost - min(assoc_cost);
    reliability = exp(-0.5 * min(delta, 700));
    if ~any(isfinite(reliability) & reliability > 0)
        reliability = ones(1, n);
    end
end
reliability(~isfinite(reliability) | reliability < 0) = 0;
if sum(reliability) <= 0
    reliability = ones(1, n);
end
mean_weight = reliability / sum(reliability);

az = ang_deg(1, :);
el = ang_deg(2, :);
los = [cosd(el) .* sind(az); ...
       cosd(el) .* cosd(az); ...
       sind(el)];
los_mean = los * mean_weight(:);
los_norm = norm(los_mean);
if ~isfinite(los_norm) || los_norm <= eps
    z_fused = zeros(2, 0);
    R_fused = zeros(2, 2, 0);
    return;
end
los_mean = los_mean / los_norm;
z_fused = [atan2d(los_mean(1), los_mean(2)); ...
           atan2d(los_mean(3), hypot(los_mean(1), los_mean(2)))];

info_matrix = zeros(2, 2);
for j = 1:n
    Rj = make_spd_local(R_all(:, :, j));
    R_all(:, :, j) = Rj;
    info_matrix = info_matrix + reliability(j) * (Rj \ eye(2));
end
R_base = make_spd_local(info_matrix \ eye(2));

dispersion = 0;
for j = 1:n
    residual = [angle_diff_deg(ang_deg(1, j), z_fused(1)); ...
                ang_deg(2, j) - z_fused(2)];
    S_res = make_spd_local(R_all(:, :, j) + R_base);
    dispersion = dispersion + mean_weight(j) * (residual' * (S_res \ residual)) / 2;
end
dispersion = max(1, dispersion);

R_scale = 1;
if n > 1
    R_scale = get_cfg_scalar(cfg, 'passive_meas_fuse_R_scale', 1.5);
end
R_fused = make_spd_local(R_base * dispersion * R_scale);

info.n_used = n;
info.fused = n > 1;
info.reliability = reliability;
info.mean_weight = mean_weight;
info.dispersion_inflate = dispersion;
info.R_scale_applied = R_scale;
end

function info = make_info(n_input)
info = struct('n_input', n_input, 'n_valid', 0, 'n_used', 0, ...
    'fused', false, 'reliability', zeros(1, 0), ...
    'mean_weight', zeros(1, 0), 'dispersion_inflate', 1, ...
    'R_scale_applied', 1);
end

function cost = normalize_cost(cost, n)
if nargin < 1 || isempty(cost)
    cost = zeros(1, n);
else
    cost = double(cost(:).');
    if isscalar(cost)
        cost = repmat(cost, 1, n);
    elseif numel(cost) ~= n
        error('assoc_cost 的元素数必须等于角度数');
    end
end
end

function R_all = normalize_covariance(R_deg2, n)
if isempty(R_deg2)
    error('R_deg2 不能为空');
elseif isequal(size(R_deg2), [2, 2])
    R_all = repmat(R_deg2, 1, 1, n);
elseif ndims(R_deg2) == 3 && size(R_deg2, 1) == 2 && ...
        size(R_deg2, 2) == 2 && size(R_deg2, 3) == n
    R_all = R_deg2;
else
    error('R_deg2 必须为 2x2 或 2x2xN');
end
end

function v = get_cfg_scalar(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name)) && isnumeric(cfg.(name)) && ...
        isscalar(cfg.(name)) && isfinite(cfg.(name))
    v = double(cfg.(name));
else
    v = default_value;
end
end

function v = get_cfg_text(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name))
    raw = cfg.(name);
    if isa(raw, 'string') && isscalar(raw)
        raw = char(raw);
    end
    if ischar(raw)
        v = lower(strtrim(raw));
        return;
    end
end
v = default_value;
end

function d = angle_diff_deg(a, b)
d = mod(a - b + 180, 360) - 180;
end

function A = make_spd_local(A)
A = (double(A) + double(A)') / 2;
[V, D] = eig(A);
d = real(diag(D));
scale = max(1, max(abs(d)));
d(~isfinite(d)) = scale;
d = max(d, 1e-10 * scale);
A = real(V * diag(d) * V');
A = (A + A') / 2;
end
