function [L,l, A_transposed] = InverseKinematicsWrenchMatrix(y, a, b)
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

R = [cosd(phi) -sind(phi);
     sind(phi) cosd(phi)];


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

% Two-dimensional Vectors
u1 = [u(:,1)];
u2 = [u(:,2)];
u3 = [u(:,3)];
u4 = [u(:,4)];

b1 = R*[b(:,1)];    % Transformed into INERTIA-frame
b2 = R*[b(:,2)];    % Transformed into INERTIA-frame
b3 = R*[b(:,3)];
b4 = R*[b(:,4)];

% Cross-product in 2 dimensions
h1 = b1(1)*u1(2) - b1(2)*u1(1);
h2 = b2(1)*u2(2) - b2(2)*u2(1);
h3 = b3(1)*u3(2) - b3(2)*u3(1);
h4 = b4(1)*u4(2) - b4(2)*u4(1);

% Columns of A_transposed
A1 = [u1;h1];
A2 = [u2;h2];
A3 = [u3;h3];
A4 = [u4;h4];

% Wrench Matrix
A_transposed = [A1 A2 A3 A4];
end