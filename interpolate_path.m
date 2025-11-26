function point = interpolate_path(s, ref_path)
% Interpolate path point at arc length s

if s <= ref_path.s(1)
    point = [ref_path.x(1); ref_path.y(1)];
    return;
end

if s >= ref_path.s(end)
    point = [ref_path.x(end); ref_path.y(end)];
    return;
end

idx = find(ref_path.s >= s, 1) - 1;
idx = max(1, min(idx, length(ref_path.s) - 1));

ds = ref_path.s(idx+1) - ref_path.s(idx);
if ds < 1e-6
    t = 0;
else
    t = (s - ref_path.s(idx)) / ds;
end

x = ref_path.x(idx) + t * (ref_path.x(idx+1) - ref_path.x(idx));
y = ref_path.y(idx) + t * (ref_path.y(idx+1) - ref_path.y(idx));
point = [x; y];
end
