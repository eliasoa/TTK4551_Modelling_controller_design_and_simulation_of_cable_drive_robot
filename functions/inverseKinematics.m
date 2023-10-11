function [L,l] = inverseKinematics(r,R,m,a,b)
% l = inverseKinematics(r,R,m,a,b) calculates the cable length vector l of 
% size m (number of cables) for a given pose (r,R) using the vector of 
% proximal attechent points a in K_0 and the vector of relative position 
% of the distal attachment points in K_p
%
% Equation 4.2 in Cable-Driven Parallel Robots by Andreas Pott
% l_i = |I_i| = |a_i − r − Rb_i| for i = 1, . . . , m
% 
% Author:    Elias Olsen Almenningen
% Date:      04 Oct 2023
% Revisions:

m = 4;

L = zeros(2,m);  % preallocating memory
for i = 1:m
    L(:,i) = a(:,i) - r - R*b(:,i);
end
l = zeros(1,m);
for i = 1:m
    l(i) = norm(L(:,i));
end