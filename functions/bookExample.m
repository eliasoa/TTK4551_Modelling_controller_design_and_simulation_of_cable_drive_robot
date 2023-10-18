close all
clear
clc
%% Test using planar robot from Pott
a = [ -2 2 2 -2;  
       2 2 0  0 ];
b = [-0.05 0.05 0.05 -0.05;
      0.1  0.1  0.0   0.0 ];

% Cartesian position of 0_{b} on mobile platform in {i}
r_real = [1.9 0.1]';

% Attitude of mobile platform {degrees} phi ∈ [-45, 45 ] deg
phi_d = 0;
phi = deg2rad(phi_d);

y_real = [r_real;phi];

[~,l] = inverseKinematics(r_real,RotMat2D(phi),4,a,b);
tic
y_DK = DirectKinematics(a,b,l);
toc

disp('The real pose is:')
disp(y_real(1:2))
disp(phi_d)
disp('The DK pose is:')
disp(y_DK(1:2))
disp(rad2deg(y_DK(3)))
disp('The error is')
feil = [r_real(1) - y_DK(1); r_real(2) - y_DK(2); rad2deg(phi) - rad2deg(y_DK(3))];
disp(feil)