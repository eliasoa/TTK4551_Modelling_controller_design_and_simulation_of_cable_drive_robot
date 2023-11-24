function [phi_min, phi_max] = OrientationWorkspace(r_0,a,b, f_min,f_max, w)

% Phi
phi = linspace(-90, 90, 360);

P = length(phi);

% Allocations for solutions
f_positive = zeros(P);

for i=1:P
    pose = [r_0;phi(i)];
    [~,~,A_transpose] = CDPR_InverseKinematics_V2(pose, a, b);
    [f, flag] = Kernel_translation_Elias(A_transpose',w,f_min,f_max,2);
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