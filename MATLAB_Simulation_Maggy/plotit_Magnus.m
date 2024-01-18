% %% Preamble
 close all
 clear
 clc
%% Simulink Sim

out = sim("cable_rob_Magnus.slx");

%% Ball Simulator
length = 0.4;
height = 0.2;

% Ole Morten
time = out.simout.Time;
data = out.simout.Data;



p = data(:,1:2);
psi = data(:,3);

ball_t = out.ball_p.Time;
ball_p = out.ball_p.Data;

% Anchor points
a1 = [1;1];
a2 = [-1;1];
a3 = [-1;-1];
a4 = [1;-1];

% Body anchors
b1 = [length/2;height/2];
b2 = [-length/2;height/2];
b3 = [-length/2;-height/2];
b4 = [length/2;-height/2];

axes_limits = [-1,1,-1,1];
for n = 1:max(size(time))
    % Body anchors in global coordinates
    b1g = b_to_g(p(n,:)',psi(n),b1);
    b2g = b_to_g(p(n,:)',psi(n),b2);
    b3g = b_to_g(p(n,:)',psi(n),b3);
    b4g = b_to_g(p(n,:)',psi(n),b4);
    sq = [b1g,b2g,b3g,b4g,b1g];
    h=plot(sq(1,:),sq(2,:),'b');
    hold on;
    plot([a1(1),b1g(1)],[a1(2),b1g(2)],'r');
    plot([a2(1),b2g(1)],[a2(2),b2g(2)],'r');
    plot([a3(1),b3g(1)],[a3(2),b3g(2)],'r');
    plot([a4(1),b4g(1)],[a4(2),b4g(2)],'r');
    % plot(ball_p(n,1),ball_p(n,2),'*');
    hold off;
    axis(axes_limits);
    set(h,"LineWidth",1);
    % axis equal;
    
    % Fra ChatGPT for å vise simuleringstid
    % Calculate midpoint for x-coordinate of the text
    mid_x = mean(axes_limits(1:2));
    % Set y-coordinate for the text slightly above the top of the plot
    text_y = axes_limits(4) + 0.05;

    % Displaying the simulation time
    text(mid_x, text_y, sprintf('Time: %.2f seconds', time(n)), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

    if n==1
        pause;
    else
        pause(0.001);
    end
end

%% Test Plotting

%% Cable Tensions

cableT_time = out.cableTensions.Time;

cableT_data = zeros(4, length(cableT_time));

for i = 1:length(cableT_time)
    cableT_data(:, i) = out.cableTensions.Data(:,:,i);
end

cableT1 = cableT_data(1,:);
cableT2 = cableT_data(2,:);
cableT3 = cableT_data(3,:);
cableT4 = cableT_data(4,:);

figure(2)
subplot(4,1,1)
plot(cableT_time, cableT1)

subplot(4,1,2)
plot(cableT_time, cableT2)

subplot(4,1,3)
plot(cableT_time, cableT3)

subplot(4,1,4)
plot(cableT_time, cableT4)

%% Motor RPM
motorRPM_time = out.motorRPM.Time;
motorRPM_data = out.motorRPM.Data;

% Convert data to rpm
motor1 = motorRPM_data(:,1)*60/2*pi;
motor2 = motorRPM_data(:,2)*60/2*pi;
motor3 = motorRPM_data(:,3)*60/2*pi;
motor4 = motorRPM_data(:,4)*60/2*pi;

figure(3)
subplot(4,1,1)
plot(motorRPM_time, motor1)
title("Motor 1")
xlabel("Time")
ylabel("RPM")

subplot(4,1,2)
plot(motorRPM_time, motor2)
title("Motor 2")
xlabel("Time")
ylabel("RPM")

subplot(4,1,3)
plot(motorRPM_time, motor3)
title("Motor 3")
xlabel("Time")
ylabel("RPM")

subplot(4,1,4)
plot(motorRPM_time, motor4)
title("Motor 4")
xlabel("Time")
ylabel("RPM")



