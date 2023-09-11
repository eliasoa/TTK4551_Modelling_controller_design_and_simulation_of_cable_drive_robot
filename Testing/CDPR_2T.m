% Frame of robot  
% A2     1m     A3
% |‾‾‾‾‾‾‾‾‾‾‾‾‾‾|
% |  B1    B2    |
% |    -----     |  1m
% |     B0       |
% |______________|
% A1            A4
%
% A2 = (0,1)'    A3 = (1,1)'
% A1 = (0,0)'    A4 = (1,0)'   
% Ai = (x,y)'
%
% B1 = (-0.2 0) in {b}
% B2 = (0.2 0) in {b}
%
% r = (0.5 0.5)'
%__________________________________________________________________________
origin = [0 0]';            % Same as {i}
% Proximal anchor points
A1 = [0 0]';
A2 = [0 1]';
A3 = [1 1]';
A4 = [1 0]';

% Vectors of proximal anchor points in {i}
a1 = A1 - origin;
a2 = A2 - origin;
a3 = A3 - origin;
a4 = A4 - origin;

% Cartesian position of 0_{b} mobile platform in {i}
r = [0.5 0.5]';

% Distal anchor points in {b}
body_origin = [0 0]';       % 0_{b}
B1 = [-0.2 0]';
B2 = [0.2 0]';

% Vectors of distal anchor points in {b}
b1 = B1 - body_origin;
b2 = B2 - body_origin;

% Vectors of cables in {i}
theta = 0;                  % Attitude of m.p
l1 = a1 - r - R(theta)*b1;
l2 = a2 - r - R(theta)*b1;
l3 = a3 - r - R(theta)*b2;
l4 = a4 - r - R(theta)*b2;

% Normalized unit vector of b_i
u1 = l1/norm(l1);
u2 = l2/norm(l2);
u3 = l3/norm(l3);
u4 = l4/norm(l4);


% Structure matrix
A_t = [u1 u2 u3 u4];

% Modified structure matrix
A_hat_t = [l1 l2 l3 l4];


%% Functions
function noe = R(theta)
% 2D rotation matrix about the 'z'-axis in degrees
noe = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
end
