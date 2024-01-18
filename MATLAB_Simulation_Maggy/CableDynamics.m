function [l_dot, l_ddot] = CableDynamics(xdot, u, b)
%   xdot:
%      u: matrix of the 4 cable unit vectors
% 

% Velocities and accelerations of the platform
vx = xdot(1);
vy = xdot(2);
omega = xdot(3);    % Angular Velocity
ax = xdot(4);
ay = xdot(5);
alpha = xdot(6);    % Angular Acceleration

% Vector form
v_p = [vx;vy;0];
omega_p = [0;0;omega];
a_p = [ax;ay;0];
alpha_p = [0;0;alpha];


% Dynamic Equation by Verhoeven

% Rate of change of cables
l_dot = zeros(4,1);
for i = 1:4
    b_dot = b_dot_g(v_p, omega_p, psi, b);
    l_dot(i) = -[u(:,i), 0]'*b_dot;
end


l_ddot = zeros(4,1);

