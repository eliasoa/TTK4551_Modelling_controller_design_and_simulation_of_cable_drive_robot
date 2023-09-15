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
origin = [0 0]';            % Same as 0_{i}
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

% Cartesian position of 0_{b} on mobile platform in {i}
r_0 = [0.5 0.5;]';
r_end = [0.5 .5]';
r_step_time = 5;
T = 1;  % Filter for not having step

% Attitude of mobile platform {degrees}
theta_0 = 0;
theta_end = 45;
theta_step_time = 5;

% Distal anchor points in {b}
body_origin = [0 0]';       % 0_{b}
B1 = [-0.2 0]';
B2 = [0.2 0]';

% Vectors of distal anchor points in {b}
b1 = B1 - body_origin;
b2 = B2 - body_origin;

% Vectors of cables in {i}
r = [0.5 0.5]';
l1 = norm(a1 - r - R(theta)*b1,2);
l2 = norm(a2 - r - R(theta)*b1,2);
l3 = norm(a3 - r - R(theta)*b2,2);
l4 = norm(a4 - r - R(theta)*b2,2);
l = [l1 l2 l3 l4]';

% Normalized unit vector of b_i
u1 = l1/norm(l1);
u2 = l2/norm(l2);
u3 = l3/norm(l3);
u4 = l4/norm(l4);

% Structure matrix
A_t = [u1 u2 u3 u4];

% Modified structure matrix
A_hat_t = [l1 l2 l3 l4];

%% Run simulink
sim("Simulator_inverse_kinematics_1R2T.slx")

% %% Plot
% % Frame of the CDPR
% xmin=0;
% xmax=1;
% ymin=0;
% ymax=1;
% rectangle('position',[xmin ymin xmax ymax])
% % Fix 'zoom'
% axis([-0.2 1.2 -0.2 1.2])
% % Add vectors, l_i, b_i etc
% arrow(r+R(theta)*b1,r+b1+l1,'Color','g')    %l_1
% arrow(r+R(theta)*b1,r+b1+l2,'Color','g')    %l_2
% arrow(r+R(theta)*b2,r+b2+l3,'Color','g')    %l_3
% arrow(r+R(theta)*b2,r+b2+l4,'Color','g')    %l_4
% g =r+R(theta)*b1;
% j =r+b1+l1;
% % annotation("textarrow",[g(1) j(1)],[g(2) j(2)])


%% Functions
function noe = R(theta)
% 2D rotation matrix about the 'z'-axis in degrees
noe = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
end