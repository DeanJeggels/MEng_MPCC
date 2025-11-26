function x_next = mpccStateFunction(x, u)
% MPCC state function - Kinematic bicycle model
% State vector x = [x, y, psi, s]
%   x: x position (m)
%   y: y position (m)
%   psi: heading angle (rad)
%   s: arc length progress (m)
%
% Input vector u = [v, omega, vs]
%   v: linear velocity (m/s)
%   omega: angular velocity (rad/s)
%   vs: virtual velocity along path (m/s)

% Fixed time step matching simulation
dt = 0.1;

% Validate inputs
if length(x) ~= 4
    error('State vector must have 4 elements: [x, y, psi, s]');
end

if length(u) ~= 3
    error('Input vector must have 3 elements: [v, omega, vs]');
end

% Pre-compute trigonometric functions for efficiency
cos_psi = cos(x(3));
sin_psi = sin(x(3));

% Initialize output state
x_next = zeros(4, 1);

% Kinematic bicycle model (discrete-time integration)
% Matching typical C++ MPCC state equations:
%   dx/dt = v * cos(psi)
%   dy/dt = v * sin(psi)
%   dpsi/dt = omega
%   ds/dt = vs

x_next(1) = x(1) + u(1) * cos_psi * dt;     % x position update
x_next(2) = x(2) + u(1) * sin_psi * dt;     % y position update
x_next(3) = x(3) + u(2) * dt;               % heading angle update
x_next(4) = x(4) + u(3) * dt;               % arc length progress update

% Normalize heading angle to [-pi, pi]
x_next(3) = wrapToPi(x_next(3));

% Ensure arc length stays non-negative
% Note: Wrapping for lap completion is handled in main simulation loop
x_next(4) = max(0, x_next(4));

% Validate output for numerical stability
if any(isnan(x_next)) || any(isinf(x_next))
    warning('NaN or Inf detected in state update. Using previous state.');
    x_next = x;
end

end
