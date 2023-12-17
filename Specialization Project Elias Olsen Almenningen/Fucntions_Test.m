close all
clear
%% Folder magic
% Get the current script's directory
currentFolder = fileparts(which("TEST_FUNCTIONS.m"));

% Add the functions folder to the MATLAB path
addpath(fullfile(currentFolder, 'Functions'));

%% USER INPUT %%
% Uncomment the functions to test them
r                   = [-0.4, -0.4]';                  % m
phi                 = deg2rad(0);               % degrees
% Cable force limits
f_min               = 10;                        % [N]
f_max               = 100;                       % [N]
f_level             = 2;                        % Force level

y                   = [r;phi];                  % pose, DO NOT CHANGE
%% Geometric Information
% Size of MP
l_MP                = 0.15;
h_MP                = 0.02;
% Size of frame
l_F                 = 1;
h_F                 = 1;

[a, b, Wp, ~, ~]    = initRobot(l_MP,h_MP,l_F,h_F);

%% Inverse Kinematics
[L,l,~]             = InverseKinematics(a, b, y);

%% Direct Kinematics
[y]                 = DirectKinematics(a,b,l);

%% Force Distribution
% Cable length
[~,~,u]             = InverseKinematics(a, b, y);
% Wrench matrix
A                   = WrenchMatrix(b,u,y);
% Cable forces
% [f,flag]            = Kernel_translation(A,Wp,f_min,f_max,f_level);

%% Translational Workspace
dim = [1;1];
TranslationWorkspace(phi,a,b, dim, f_min,f_max,f_level, Wp)

%% Orientation Workspace
[phi_min, phi_max] = OrientationWorkspace(r,a,b,f_min,f_max,f_level,Wp)