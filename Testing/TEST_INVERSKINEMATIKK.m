% TEST INVERSKINEMATIKK

% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
a1 = [0;1];
a2 = [1;1];
a3 = [0;0];
a4 = [1;0];

% Length of end-effector rod CONSTANT
L_end = 0.3;

% Cable attachment points Body (In BODY coordinates) CONSTANT
b1 = [0;L_end/2];
b2 = [0;-L_end/2];

% Rotational Angle between INERTIA and BODY frame, VARIABLE
theta_body = 0;     % Degrees, righthand rule

% Rotation Matrix (from INERTIA to BODY)
R = [cos(theta_body) -sin(theta_body);sin(theta_body) cos(theta_body)];

% Position of CM of BODY (In INERTIA Coordinates)
p = [0.5;0.5];

% TEST TIME VARYING MATRIX DET HER ER IKKE TESTA BTW
%h = 0.2;
% theta = 0:h:20;
% 
% R = {}
% R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
% t = 0 : h : h*(length(R)-1);
% 
% R_t = timeseries(R, t);


% Simulate SIMULINK
sim("CDPR_INVERSKINEMATIKK_14_09_23.slx")
