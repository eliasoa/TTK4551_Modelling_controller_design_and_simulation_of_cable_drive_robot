close all
clear
%% Folder magic
% Get the current script's directory
currentFolder       = fileparts(which("TEST_FUNCTIONS.m"));

% Add the functions folder to the MATLAB path
addpath(fullfile(currentFolder, 'Functions'));

%% User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial pose
y_0                 = [0,0,0]';
% Reference pose
y_reference         = [0.4,-0.4,deg2rad(20)]';

% PID control
Kp = diag([8 8 4]);
Kd = diag([3 3 1]);
Ki = diag([3 3 3]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Geometric Information
% Size of MP
l_MP                = 0.15;
h_MP                = 0.02;
% Size of frame
l_F                 = 1;
h_F                 = 1;

[a, b, Wp, m, Iz]   = initRobot(l_MP,h_MP,l_F,h_F);


%% Simulink
simtime             = 10;
simout =            sim("PoseSimulation.slx");

%% Plot
% t = simout.pose.Time;
% pose = simout.pose.Data;
% % Help from ChatGPT to remove one column from simout
% pose = squeeze(pose);
% 
% x_p = pose(1,:);
% y_p = pose(2,:);
% p_p = rad2deg(pose(3,:));
% 
% pose_d = simout.desired_pose.Data;
% % Help from ChatGPT to remove one column from simout
% pose_d = squeeze(pose_d)';
% 
% xd_p = pose_d(1,:);
% yd_p = pose_d(2,:);
% pd_p = rad2deg(pose_d(3,:));
% 
% force = simout.force.Data;
% % Help from ChatGPT to remove one column from simout
% force = squeeze(force);
% figure(1)
% subplot(311)
% hold on
% plot(t,x_p)
% plot(t,xd_p,'--')
% hold off
% legend("x", "x_{ref}","Interpreter","tex")
% ylabel("[m]")
% grid
% title("x position of the MP", "Interpreter","tex")
% 
% subplot(312)
% hold on
% plot(t,y_p)
% plot(t,yd_p,'--')
% hold off
% legend("y", "y_{ref}","Interpreter","tex")
% ylabel("[m]")
% grid
% title("y position of the MP", "Interpreter","tex")
% 
% subplot(313)
% hold on
% plot(t,p_p)
% plot(t,pd_p,'--')
% hold off
% legend("\phi", "\phi_{ref}","Interpreter","tex")
% ylabel("Degrees")
% xlabel("t")
% grid
% title("MP orientation angle \phi", "Interpreter","tex")
% 
% figure(4)
% hold on
% plot(t,force)
% hold off
% legend('f_1','f_2','f_3','f_4')
% ylabel("[N]")
% xlabel("t")
% grid
% title("Forces in each cable", "Interpreter","tex")