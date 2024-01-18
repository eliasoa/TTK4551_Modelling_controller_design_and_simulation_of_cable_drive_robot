function b_dot = b_dot_g(v, omega, psi, b)
%       v: Three-dimensional linear velocity vector of the platform
%   omega: Three-dimensional angular velocity vector of the platform
%     psi: Rotation of body frame
%       b: attachment point in bodyframe
%   b_dot: The "absolute velocity" of the single point b_i in global coordinates


R = [cos(psi), -sin(psi);sin(psi),cos(psi)];
bR = [R*b;0];   % 3D because of the cross-product below

% b_dot re
b_dot = v + cross(omega, bR); 