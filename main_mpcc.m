%% MPCC Controller - WORKING VERSION with Proper Rewards
clear; close all; clc;
fprintf('🏁 MPCC Fixed Version Starting...\n');

%% Load trajectory data
trajectory_data = readmatrix('aut_centerline.csv');
ref_path.x = trajectory_data(:, 1);
ref_path.y = trajectory_data(:, 2);
ref_path.left_bound = trajectory_data(:, 3);
ref_path.right_bound = trajectory_data(:, 4);

% Compute arc length
ref_path.s = zeros(length(ref_path.x), 1);
for i = 2:length(ref_path.x)
    ds = sqrt((ref_path.x(i) - ref_path.x(i-1))^2 + (ref_path.y(i) - ref_path.y(i-1))^2);
    ref_path.s(i) = ref_path.s(i-1) + ds;
end

track_length = ref_path.s(end);
fprintf('✅ Track loaded: %d points, %.2f m\n', length(ref_path.x), track_length);

%% Calculate track bounds for fixed axis limits
x_min = min(ref_path.x); x_max = max(ref_path.x);
y_min = min(ref_path.y); y_max = max(ref_path.y);
x_margin = (x_max - x_min) * 0.1;
y_margin = (y_max - y_min) * 0.1;
axis_limits = [x_min - x_margin, x_max + x_margin, y_min - y_margin, y_max + y_margin];

fprintf('📊 Track bounds: X=[%.1f, %.1f], Y=[%.1f, %.1f]\n', x_min, x_max, y_min, y_max);

%% Parameters - FIXED: Much faster speeds and better dt
v_max = 1.0;         % INCREASED from 0.2 to 1.0 m/s - much faster!
omega_max = pi/2;    % Keep same
v_s_max = 2.0;       % INCREASED from 0.2 to 2.0 m/s - much faster s progress!
vehicle_radius = 0.6;

dt = 0.1;            % FIXED: Use consistent dt = 0.1 everywhere

% Cost parameters - FIXED: Better balance
k_c = 25; k_l = 25; 
k_th = 1000;         % REDUCED from 10000 - less emphasis on s position
Pk_val = 300;
k_v_vals = 0.1; k_omega_vals = 5.0; k_vs_vals = 0.1;

%% Setup NLMPC
nx = 4; ny = 4; nu = 3;
nlmpcobj = nlmpc(nx, ny, 'MV', [1 2 3]);
nlmpcobj.Ts = dt;                    % FIXED: Consistent dt
nlmpcobj.PredictionHorizon = 3;
nlmpcobj.ControlHorizon = 3;
nlmpcobj.Model.IsContinuousTime = false;

% Set functions
nlmpcobj.Model.StateFcn = "mpccStateFunction";
nlmpcobj.Optimization.CustomCostFcn = @(X,U,e,data) mpccCostFunction(X,U,e,data,k_c,k_l,k_th,Pk_val,k_v_vals,k_omega_vals,k_vs_vals);

% Input constraints - FIXED: Use the new faster limits!
nlmpcobj.ManipulatedVariables(1).Min = 0.1*v_max;     % Min speed
nlmpcobj.ManipulatedVariables(1).Max = v_max;         % Max speed
nlmpcobj.ManipulatedVariables(2).Min = -omega_max;    % Min omega
nlmpcobj.ManipulatedVariables(2).Max = omega_max;     % Max omega  
nlmpcobj.ManipulatedVariables(3).Min = 0.1*v_s_max;   % Min vs - MUCH higher!
nlmpcobj.ManipulatedVariables(3).Max = v_s_max;       % Max vs - MUCH higher!

%% Initialize
x0 = [0; 0.0; 0; 0];
u0 = [0.2*v_max; 0.0; 0.2*v_s_max];  % FIXED: Start with higher initial speeds

try
    validateFcns(nlmpcobj, x0, u0);
    fprintf('✅ Functions validated\n');
catch ME
    fprintf('⚠️ Validation warning: %s\n', ME.message);
end

%% Simulation Setup
desired_laps = 1;     % Start with just 2 laps for testing
max_steps = 5000;     % Reduced for faster testing

% Live Plot
figure('Position', [100, 100, 1400, 800]);
hold on; grid on; 
axis(axis_limits);
axis equal;

% Plot reference path
plot(ref_path.x, ref_path.y, 'k--', 'LineWidth', 3, 'DisplayName', 'Reference Path');

% Plot handles
h_actual = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');
h_current = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', 'Current Vehicle');
h_ref_point = plot(NaN, NaN, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'green', 'DisplayName', 'Reference Point (s)');

legend('Location', 'best');
xlabel('X (m)'); ylabel('Y (m)'); title('MPCC Fixed Version - Live Trajectory');

% Storage
XVec = x0';
UVec = u0';
TimeVec = 0;
RefXVec = []; RefYVec = [];
laps_completed = 0;
total_s_traveled = 0;
lap_start_times = [];

%% Main Loop
fprintf('\n🚀 Starting simulation...\n');
tic;

try
    for k = 1:max_steps
        current_time = k * dt;
        
        % Multi-lap handling
        if x0(4) >= track_length
            laps_completed = laps_completed + 1;
            lap_start_times(end+1) = current_time;
            total_s_traveled = total_s_traveled + track_length;
            
            fprintf('🏁 Completed Lap %d/%d at time %.1fs\n', laps_completed, desired_laps, current_time);
            
            % Reset s coordinate for next lap
            x0(4) = x0(4) - track_length;
            x0(4) = max(0, min(x0(4), track_length - 1e-6));
            
            if laps_completed >= desired_laps
                fprintf('🎉 ALL %d LAPS COMPLETED! 🎉\n', desired_laps);
                break;
            end
        end
        
        % Get reference point
        current_s = mod(x0(4), track_length);
        current_s = max(0, min(current_s, track_length - 1e-6));
        
        try
            ref_point = interpolate_path(current_s, ref_path);
            if isempty(ref_point) || length(ref_point) < 2 || any(~isfinite(ref_point))
                ref_point = [ref_path.x(1); ref_path.y(1)];
            end
        catch
            ref_point = [ref_path.x(1); ref_path.y(1)];
        end
        
        RefXVec(end+1) = ref_point(1);
        RefYVec(end+1) = ref_point(2);
        
        % Progress report with DEBUG INFO
        if mod(k, 50) == 0  % More frequent updates
            lap_progress = 100 * current_s / track_length;
            tracking_error = sqrt((x0(1) - ref_point(1))^2 + (x0(2) - ref_point(2))^2);
            fprintf('Step %d: Lap %d (%.1f%%) | v=%.3f, vs=%.3f | s=%.3f | Error=%.3f m\n', ...
                   k, laps_completed + 1, lap_progress, u0(1), u0(3), current_s, tracking_error);
        end
        
        % NLMPC step
        [~, ~, info] = nlmpcmove(nlmpcobj, x0, u0);
        u_opt = info.MVopt(1, :);
        
        % MANUAL state update - FIXED: Use consistent dt
        x_next = zeros(4,1);
        x_next(1) = x0(1) + u_opt(1)*cos(x0(3))*dt;
        x_next(2) = x0(2) + u_opt(1)*sin(x0(3))*dt;
        x_next(3) = x0(3) + u_opt(2)*dt;
        x_next(4) = x0(4) + u_opt(3)*dt;
        
        % Ensure s coordinate doesn't go negative
        x_next(4) = max(0, x_next(4));
        
        x0 = x_next;
        u0 = u_opt';
        
        % Store data
        XVec = [XVec; x0'];
        UVec = [UVec; u0'];
        TimeVec = [TimeVec; current_time];
        
        % Update plot
        if mod(k, 10) == 0  % More frequent plot updates
            set(h_actual, 'XData', XVec(:,1), 'YData', XVec(:,2));
            set(h_current, 'XData', x0(1), 'YData', x0(2));
            set(h_ref_point, 'XData', ref_point(1), 'YData', ref_point(2));
            
            axis(axis_limits);
            
            title(sprintf('MPCC: Lap %d/%d (%.1f%%) | s=%.2f | v=%.2f | vs=%.2f', ...
                   laps_completed + 1, desired_laps, 100*current_s/track_length, current_s, u0(1), u0(3)));
            drawnow;
        end
    end
    
catch ME
    fprintf('❌ Error during simulation: %s\n', ME.message);
end

simulation_time = toc;

%% Results
fprintf('\n============ RESULTS ============\n');
fprintf('Completed laps: %d/%d\n', laps_completed, desired_laps);
fprintf('Simulation time: %.2f seconds\n', simulation_time);
fprintf('Total distance traveled: %.2f m\n', total_s_traveled + x0(4));

if ~isempty(XVec) && ~isempty(RefXVec)
    min_length = min(size(XVec, 1), length(RefXVec));
    if min_length > 1
        XVec_trim = XVec(1:min_length, :);
        RefXVec_trim = RefXVec(1:min_length);
        RefYVec_trim = RefYVec(1:min_length);
        
        tracking_errors = sqrt((XVec_trim(:,1) - RefXVec_trim').^2 + (XVec_trim(:,2) - RefYVec_trim').^2);
        avg_error = mean(tracking_errors);
        fprintf('Average tracking error: %.3f m\n', avg_error);
    end
end

try
    final_s = mod(x0(4), track_length);
    final_ref = interpolate_path(final_s, ref_path);
    
    if ~isempty(final_ref) && length(final_ref) >= 2 && all(isfinite(final_ref))
        final_error = sqrt((x0(1) - final_ref(1))^2 + (x0(2) - final_ref(2))^2);
        fprintf('Final tracking error: %.3f m\n', final_error);
    else
        start_error = sqrt((x0(1) - ref_path.x(1))^2 + (x0(2) - ref_path.y(1))^2);
        fprintf('Final distance from start: %.3f m\n', start_error);
    end
catch ME
    fprintf('Final vehicle position: (%.3f, %.3f)\n', x0(1), x0(2));
    fprintf('Final s coordinate: %.3f m\n', x0(4));
end

if laps_completed > 0 && length(lap_start_times) > 1
    fprintf('\n--- LAP TIMES ---\n');
    for i = 1:length(lap_start_times)-1
        lap_time = lap_start_times(i+1) - lap_start_times(i);
        fprintf('Lap %d: %.2f seconds\n', i, lap_time);
    end
end

fprintf('🏁 Simulation complete!\n');
