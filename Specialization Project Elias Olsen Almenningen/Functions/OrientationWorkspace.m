function [phi_min, phi_max] = OrientationWorkspace(r_0, a, b, f_min, f_max, level, w)
% [phi_min, phi_max] = OrientationWorkspace(r_0, a, b, f_min, f_max, level, w)
% OrientationWorkspace calculates the Orientation workspace given a
% position, tension limit, tension level and external wrench
%
%
% Author:    Magnus Grøterud


% Phi
phi = linspace(-90, 90, 30);

P = length(phi);

% Allocations for solutions
f_positive = zeros(P,1);

for i=1:P
    pose = [r_0; deg2rad(phi(i))];
    [~,~,u] = InverseKinematics(a, b, pose);
    A = WrenchMatrix(b, u, pose);
    [~, flag] = Kernel_translation(A, w, f_min, f_max, level);
    if not(flag)
        f_positive(i) = 1;
    end
end

% Find the index of the first occurrence of 1
firstIndex = find(f_positive == 1, 1, 'first');

% Find the index of the last occurrence of 1
lastIndex = find(f_positive == 1, 1, 'last');

% Define the interval of angles for the position
phi_min = phi(firstIndex);
phi_max = phi(lastIndex);