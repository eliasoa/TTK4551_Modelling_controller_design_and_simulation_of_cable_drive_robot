close all
clear
%% Folder magic
% Get the current script's directory
currentFolder = fileparts(which("TEST_FUNCTIONS.m"));

% Add the functions folder to the MATLAB path
addpath(fullfile(currentFolder, 'Functions'));

%% Geometric Information
% Size of MP
l_MP        = 0.15;
h_MP        = 0.02;
% Size of frame
l_F        = 1;
h_F        = 1;

[a, b, Wp, ~, ~] = initRobot(l_MP,h_MP,l_F,h_F);



%% Simulink
[simout] = sim("DirectKinematics_Test.slx");

%% Plot
% estimated_pose_sim = simout.estimated_pose;
% ep_t = estimated_pose_sim.time;
% estimated_pose_sim = estimated_pose_sim.signals.values; 
% 
% [numRows, ~, numCols] = size(estimated_pose_sim);
% estimated_pose = zeros(numRows, numCols);
% for i = 1:numCols
%     estimated_pose(:, i) = squeeze(estimated_pose_sim(:, :, i));
% end
% 
% reference_pose_sim = simout.reference_pose;
% rp_t = reference_pose_sim.time;
% reference_pose = reference_pose_sim.signals.values'; 
% 
% error = reference_pose - estimated_pose;
% 
% figure(1)
% subplot(311)
% hold on
% plot(ep_t,estimated_pose(1,:))
% plot(rp_t,reference_pose(1,:),'--')
% hold off
% title("Position $x$","Interpreter","latex")
% legend("$x_{DK}$", "$x_{ref}$","Interpreter","latex")
% xlabel("t")
% ylabel("[m]")
% 
% subplot(312)
% hold on
% plot(ep_t,estimated_pose(2,:))
% plot(rp_t,reference_pose(2,:),'--')
% hold off
% title("Position $y$","Interpreter","latex")
% legend("$y_{DK}$", "$y_{ref}$", "Interpreter","latex")
% xlabel("t")
% ylabel("[m]")
% 
% subplot(313)
% hold on
% plot(ep_t,estimated_pose(3,:))
% plot(rp_t,reference_pose(3,:),'--')
% hold off
% title("Orientation $\phi$", "Interpreter","latex")
% legend("$\phi_{DK}$", "$\phi_{ref}$","Interpreter","latex")
% xlabel("t")
% ylabel("Degrees")
% 
% figure(2)
% subplot(311)
% plot(ep_t,error(1,:))
% title("Error in $x$","Interpreter","latex")
% legend("$x_e$","Interpreter","latex")
% xlabel("t", "Interpreter","latex")
% ylabel("[m]")
% xlim([0,10])
% grid on;
% subplot(312)
% plot(ep_t,error(2,:))
% title("Error in $y$","Interpreter","latex")
% legend("$y_e$", "Interpreter","latex")
% xlabel("t")
% ylabel("[m]")
% xlim([0,10])
% xlim
% grid on;
% subplot(313)
% plot(ep_t,error(3,:))
% title("Error $\phi$", "Interpreter","latex")
% legend("$\phi_e$","Interpreter","latex")
% xlabel("t", "Interpreter","latex")
% ylabel("Degrees")
% xlim([0,10])
% grid on;