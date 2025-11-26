function derivatives = get_path_derivatives(s, ref_path)
% Get path derivatives at arc length s - with error handling

if length(ref_path.s) < 2
    derivatives = [1; 0];
    return;
end

% Safety checks
if isnan(s) || isinf(s)
    s = 0;
end

s = max(ref_path.s(1), min(s, ref_path.s(end)));
idx = find(ref_path.s >= s, 1) - 1;

if isempty(idx)
    idx = 1;
end

idx = max(1, min(idx, length(ref_path.s) - 1));

ds = ref_path.s(idx+1) - ref_path.s(idx);
if ds < 1e-6 || isnan(ds) || isinf(ds)
    derivatives = [1; 0];
    return;
end

dx_ds = (ref_path.x(idx+1) - ref_path.x(idx)) / ds;
dy_ds = (ref_path.y(idx+1) - ref_path.y(idx)) / ds;

% Final safety check
if any(isnan([dx_ds, dy_ds])) || any(isinf([dx_ds, dy_ds]))
    derivatives = [1; 0];
else
    derivatives = [dx_ds; dy_ds];
end
end
