%% Parameters
g       = 9.81;

% Mobile Platform parameters
l       = 0.1;      % m
b       = 0.05;     % m
d       = 0.01;    % m
V       = l*b*d;    % m^3
rho     = 2710;     % kg/m^3
m       = V*rho;     
Iz      = 1/12*m*(l^2+b^2);  % kg m^2

% Matrices
M       = diag([m m Iz]);
D_tau = diag([5 5 5]);
Wp      = m*[0 -g 0]';


%% Model
a = [ 0 0 1 1;
      0 1 1 0];

% @ B_L = 0.15m B_H = 0.05m
b = [-0.075 -0.075  0.075  0.075;
     -0.01   0.01   0.01  -0.01];



x_ref   = 0.75;
y_ref   = 0.5;
phi_ref = 0;

% Wires
f_min = 10;
f_max = 100;
f_level = 2;

%% PID
K_p     = diag([10 10 1]);
K_d     = diag([2 80 1]);
K_i     = diag([15 15 1]);


%% Simulink
% Initial state
y_0 = [.25 .5 0]';

% Reference
y_reference   = [x_ref y_ref phi_ref]';
sim('SIMULATOR_15_11.slx')
%% øjlkf


% Rotation angle about z axis [rad]
phi             = deg2rad(0);
% Construc pose vector
y               = [y_0; phi];


% Cable length
[L,l,u]         = InverseKinematics(a, b, y);

% Wrench matrix
A               = WrenchMatrix(b,u,y);

[f,~]           = Kernel_translation(A,Wp,f_min,f_max,f_level)

y               = [y_reference; phi];
% Cable length
[L,l,u]         = InverseKinematics(a, b, y);

% Wrench matrix
A               = WrenchMatrix(b,u,y);

[f,~]           = Kernel_translation(A,Wp,f_min,f_max,f_level)