clear
close
clc
%% GeoGebra model
a = [ 0 0 1 1;
      0 1 1 0];

% @ B_L = 0.15m B_H = 0.05m
b = [-0.075 -0.075  0.075 0.075;
     -0.025  0.025 -0.025 0.025];
r_real = [0.5 0.5]';

% Attitude of mobile platform {degrees} phi ∈ [-45, 45 ] deg
phi_d = 0;
phi = deg2rad(phi_d);

y_real = [r_real;phi];

%% Kinematics
[~,l] = inverseKinematics(r_real,RotMat2D(phi),4,a,b);
tic
y_DK = DirectKinematics(a,b,l);
toc
%% Print
disp('The real pose is:')
disp(y_real(1:2))
disp(phi_d)
disp('The DK pose is:')
disp(y_DK(1:2))
disp(rad2deg(y_DK(3)))
disp('The error is')
feil = [r_real(1) - y_DK(1); r_real(2) - y_DK(2); rad2deg(phi) - rad2deg(y_DK(3))];
disp(feil)