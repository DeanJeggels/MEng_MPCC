function cineq = mpccIneqConFunction(X, U, e, data)
    global ref_path_global current_obstacles current_time_global
    
    vehicle_radius = 0.6;  % Ensure this matches your simulation parameters
    dt = 0.1;              % Time step (matches nlmpcobj.Ts)
    
    N = size(X, 1);  % Number of prediction steps (PH + 1)
    cineq = [];
    
    for i = 1:N
        s = X(i, 4);  % Track coordinate s
        x = X(i, 1);
        y = X(i, 2);

        % Get reference point at s
        ref_x = interp1(ref_path_global.s, ref_path_global.x, s, 'linear', 'extrap');
        ref_y = interp1(ref_path_global.s, ref_path_global.y, s, 'linear', 'extrap');

        theta = atan2( ...
            interp1(ref_path_global.s, ref_path_global.y, s+0.1, 'linear', 'extrap') - ref_y, ...
            interp1(ref_path_global.s, ref_path_global.x, s+0.1, 'linear', 'extrap') - ref_x );

        dx = x - ref_x;
        dy = y - ref_y;

        % Lateral error in track frame
        e_l = -cos(theta)*dx - sin(theta)*dy;

        % Get boundaries
        left_bd = interp1(ref_path_global.s, ref_path_global.left_bound, s, 'linear', 'extrap');
        right_bd = interp1(ref_path_global.s, ref_path_global.right_bound, s, 'linear', 'extrap');

        % Track boundary constraints (similar to C++ code)
        cineq(end+1) = e_l - (left_bd - vehicle_radius);
        cineq(end+1) = -e_l - (right_bd - vehicle_radius);
        
        % % OBSTACLE AVOIDANCE CONSTRAINTS (NEW)
    %     if ~isempty(current_obstacles)
    %         x_veh = X(i, 1);
    %         y_veh = X(i, 2);
    % 
    %         % Predicted time for this step (i-1 because MATLAB is 1-indexed)
    %         pred_time = dt * (i - 1);
    % 
    %         % Loop through all obstacles
    %         for j = 1:length(current_obstacles)
    %             obstacle = current_obstacles(j);
    % 
    %             % Predict obstacle position at future time
    %             future_s = obstacle.s + obstacle.vs * pred_time;
    % 
    %             % Wrap s coordinate if using circular track
    %             track_length = ref_path_global.s(end);
    %             while future_s >= track_length
    %                 future_s = future_s - track_length;
    %             end
    %             while future_s < 0
    %                 future_s = future_s + track_length;
    %             end
    % 
    %             % Get obstacle position from reference path
    %             obs_x = interp1(ref_path_global.s, ref_path_global.x, future_s, 'linear', 'extrap');
    %             obs_y = interp1(ref_path_global.s, ref_path_global.y, future_s, 'linear', 'extrap');
    %             obs_y = obs_y + 0.1;  % Offset (matches C++ code)
    % 
    %             % Calculate distance from vehicle to obstacle
    %             dx_obs = x_veh - obs_x;
    %             dy_obs = y_veh - obs_y;
    %             distance = sqrt(dx_obs*dx_obs + dy_obs*dy_obs);
    % 
    %             % Distance between surfaces (vehicle surface to obstacle surface)
    %             D_v2o = distance - obstacle.radius - vehicle_radius;
    % 
    %             % Constraint: D_v2o >= safety_dist
    %             % Formulated as: -D_v2o + safety_dist <= 0
    %             cineq(end+1) = -D_v2o + obstacle.safety_dist;
    %         end
    %     end
    % end
    
    cineq = cineq(:);  % Ensure column vector
end
