function J = mpccCostFunction(X, U, e, data)
% MPCC cost function: exactly mirrors C++ implementation
% Inputs: X (states), U (inputs), e (slack), data (nlmpc controller info)

persistent ref_path track_length
% Lazy-load trajectory one time
if isempty(ref_path)
    trajectory_data = readmatrix('aut_centerline.csv');
    nan_rows = any(isnan(trajectory_data), 2);
    trajectory_data = trajectory_data(~nan_rows, :);
    ref_path.x = trajectory_data(:, 1);
    ref_path.y = trajectory_data(:, 2);
    ref_path.left_bound = trajectory_data(:, 3);
    ref_path.right_bound = trajectory_data(:, 4);
    ref_path.s = zeros(length(ref_path.x),1);
    for i = 2:length(ref_path.x)
        ds = sqrt((ref_path.x(i) - ref_path.x(i-1))^2 + (ref_path.y(i) - ref_path.y(i-1))^2);
        ref_path.s(i) = ref_path.s(i-1) + ds;
    end
    track_length = ref_path.s(end);
end

% Controller constants matching C++ exactly
Q11 = 100.0; Q22 = 100.0;  % kc, kl from C++
q_theta = 50.0;          % kth from C++ (note: different from your 5000)
Pk = 300.0;                 % obstacle penalty constant
vehicle_radius = 0.6;       % constants.vehicleradius

% R matrix - control rate penalty as diagonal
v_max = 0.2; omega_max = pi/2; v_s_max = 0.2;  % Match your simulation limits
R = diag([0.1 / v_max^2, 1.0 / omega_max^2, 0.1 / v_s_max^2]);  % kv, kω, kvs

dt = 0.1;
p = data.PredictionHorizon;

global current_obstacles current_time_global
total_cost = 0;
prev_u = [0; 0; 0];

p_actual = min(p, size(X,1)-1);

for i = 1:p_actual+1
    if i > size(X,1), break; end
    
    % States
    x_veh = X(i,1); y_veh = X(i,2); s_veh = X(i,4);
    s_veh = mod(s_veh, track_length); 
    s_veh = max(0, min(s_veh, track_length-1e-6));
    
    % Reference point and derivatives
    try
        ref_point = interpolate_path(s_veh, ref_path);
        ref_x = ref_point(1); ref_y = ref_point(2);
        dxy = get_path_derivatives(s_veh, ref_path);
        dx_ds = dxy(1); dy_ds = dxy(2);
    catch
        ref_x = ref_path.x(1); ref_y = ref_path.y(1);
        dx_ds = 1; dy_ds = 0;
    end
    
    theta = atan2(dy_ds, dx_ds);
    dx = x_veh - ref_x; dy = y_veh - ref_y;
    e_c = sin(theta)*dx - cos(theta)*dy;    % Contour error
    e_l = -cos(theta)*dx - sin(theta)*dy;   % Lag error
    
    % Tracking cost (matches C++ Q matrix)
    total_cost = total_cost + Q11*e_c^2 + Q22*e_l^2;
    
    % Progress reward (negative cost to encourage forward motion)
    total_cost = total_cost - q_theta * s_veh; 
    % Obstacle avoidance - EXACT C++ methodology
    if ~isempty(current_obstacles)
        for obs_idx = 1:length(current_obstacles)
            obs = current_obstacles(obs_idx);

            % CRITICAL: Match C++ obstacle position prediction exactly
            pred_time = dt * (i-1);  % Time offset for this prediction step

            if abs(obs.vs) <= 1e-6
                % Static obstacle - use current position
                try
                    obs_point = interpolate_path(obs.s, ref_path);
                    obs_x = obs_point(1);
                    obs_y = obs_point(2) + 0.1;  % Slight y-offset as in your simulation
                catch
                    continue;
                end
            else
                % Moving obstacle - predict future position
                future_s = obs.s + obs.vs * pred_time;
                future_s = mod(future_s, track_length);
                try
                    obs_point = interpolate_path(future_s, ref_path);
                    obs_x = obs_point(1);
                    obs_y = obs_point(2) + 0.1;
                catch
                    continue;
                end
            end

            % Distance calculation (matches C++ computeV2ODistance exactly)
            DV2O = sqrt((x_veh - obs_x)^2 + (y_veh - obs_y)^2) - obs.radius - vehicle_radius;
            DSft_O = obs.safety_dist;

            % Adaptive weight calculation (matches C++ getAdaptiveObstacleWeight)
            qV2O = 0;
            if DV2O < 0
                qV2O = Pk;  % Maximum penalty for collision
            elseif DV2O <= DSft_O && DV2O >= 0
                ratio = DV2O / DSft_O;
                qV2O = Pk * exp(-2.0 * ratio^2);  % Exponential decay
            end

            % Apply obstacle cost (matches C++ objective function)
            if qV2O > 0
                eV2O = DV2O - DSft_O;
                total_cost = total_cost + qV2O * (eV2O^2);
            end
        end
    end
    
    % Control effort penalty (rate-based, matches C++)
    if i <= size(U,1)
        u_curr = [U(i,1); U(i,2); U(i,3)];
        du = u_curr - prev_u;
        total_cost = total_cost + du' * R * du;
        prev_u = u_curr;
    end
end

J = total_cost;
end
