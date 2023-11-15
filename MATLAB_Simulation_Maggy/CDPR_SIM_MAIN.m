%% Simulation Cable-Driven Parallell Robot

%% Preamble
clc
clear all
close all

%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        U S E R   I N P U T                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Constants
m = 1;      % Mass of platform
Jz = 1;    % Inertia of platform in CG (z-axis)
 
M = diag([m m Jz]);
D_tau = diag([5 5 5]);
G = m*[0; -9.81; 0];




% PD Controller
K_p     = diag([10 10 1]);
K_d     = diag([8 8 1]);
K_i     = diag([15 15 1]);

% Setpoints
y_ref       = [0.5;0.5;0];
y_dot_ref   = [0;0;0];
y_ddot_ref   = [0;0;0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Geometric Information
% Length of end-effector rod CONSTANT
L_end = 0.4;

% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
a1 = [0;0];
a2 = [0;1];
a3 = [1;1];
a4 = [1;0];

% Cable attachment point PLATFORM
b1 = [-L_end/2;0];
b2 = [L_end/2;0];

% Matrices of the attachment points
a = [a1 a2 a3 a4];
b = [b1 b1 b2 b2];

% Cables forces
fmin = 1;
fmax = 10;


%% Electric Motor and Winch
% Constants
I_m = zeros(1, 4);      % Motor shaft inertia
K_t = zeros(1, 4);      % Motor Constants
R_w  = zeros(1, 4);     % Pulley radius
% Gearbox?





% 




%% Simulation




