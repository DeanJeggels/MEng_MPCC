function y = mpccOutputFunction(x, u)
% MPCC Output Function for discrete-time NLMPC
% Simply output the states
%
% Inputs:
%   x - state vector [x, y, psi, s]
%   u - input vector [v, omega, vs]
%
% Output:
%   y - output vector (same as states)

y = x;

end
