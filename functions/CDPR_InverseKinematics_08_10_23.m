function [L,l, u] = CDPR_InverseKinematics(y, a, b)
% INPUTS
% y: Position and orientation of EndEffector (Platform)
% a: 2x4 Matrix of Cable Attachment points on the pulleys (INERTIA-frame)
% b: 2x2 Matrix of Cable Attachment points on the platform (BODY-frame)

% OUTPUTS
% L: 2x4 Matrix of Cable-vectors
% l: 4x1 vector of Cable lengths (scalars)
% u: 2x4 Matrix of Unit Vectors of L

m = 4;

p = y(1:2);
phi = y(3);

R = [cosd(phi) -sind(phi);sind(phi) cosd(phi)];


% Memory allocation
L = zeros(2, 4);
l = zeros(4, 1);
u = zeros(2, 4);

% Cable Vectors
for i = 1:m
    L(:,i) = a(:,i) - p - R*b(:,i);
end
% Cable Lengths
for i = 1:m
    l(i) = norm(L(:,i));
end

% Unit Cable Vectors
for i = 1:m
    u(:,i) = L(:,i)/l(i);
end
