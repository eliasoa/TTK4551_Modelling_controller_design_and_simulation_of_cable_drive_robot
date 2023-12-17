function TranslationWorkspace(phi_0, a, b, dim, f_min, f_max, level, w)
% TranslationWorkspace(phi_0,a,b, dim, f_min,f_max,level, w)
% TranslationWorkspace calculates and plots the translational workspace
% given a orientation, tension limit, tension level and external wrench
%
%
% Author:    Magnus Grøterud


% Extract Lengths of the Base
x_dim = dim(1);
y_dim = dim(2);


% Grid of the "Base Workspace"
x_grid = linspace(-x_dim/2, x_dim/2, 50);
y_grid = linspace(-y_dim/2, y_dim/2, 50);

X = length(x_grid);
Y = length(y_grid);

% Allocations for solutions
f_positive = zeros(X,Y);

for i=1:X
    for j=1:Y
        pose = [x_grid(i); y_grid(j);phi_0];
        [~,~,u] = InverseKinematics(a, b, pose);
        A = WrenchMatrix(b, u, pose);
        [~, flag] = Kernel_translation(A,w,f_min,f_max,level);
        if not(flag)
            f_positive(i,j) = 1;
        end
    end
end

%% Plotting
% Displaying the matrix where 1s are plotted and adjusting the axes
% figure(1)
% imagesc(x_grid, y_grid, f_positive);
% colormap('gray'); % Setting colormap to grayscale for better visibility
% colorbar; % Displaying colorbar to indicate values
% Flipping the y-axis
% axis yx;



%% Magnus
% Find the (x, y) coordinates where the matrix has a value of 1
[row, col] = find(f_positive == 1);

% Plot the points
figure(2)
scatter(x_grid(row), y_grid(col), 'filled', 'MarkerFaceColor', 'red'); % 'filled' option fills the markers
grid on;
% Set axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
title(['$\mathcal{W}_{CO}, \phi = ' num2str(rad2deg(phi_0)) '^\circ$'], 'Interpreter', 'latex');


% Optionally, set axis limits based on the matrix size
axis([-x_dim/2, x_dim/2, -y_dim/2, y_dim/2]);