function [L,l,u] = InverseKinematics(a, b, y)
% [L,l,u] = InverseKinematics(a, b, y, dimFlag) 
% InverseKinematics calculates the vector L with x and y comoponents of
% cable length of the scalar vector l. It also calculates the normalized
% cable vector u.
%
% L is calculated using Equation 4.2 in Cable-Driven Parallel Robots 
% by Andreas Pott:
% l_i = |I_i| = |a_i − r − Rb_i| for i = 1, . . . , m
% 
% Inputs:
% a: vector of proximal attachment points in {i} on the form
%    a   =   | a1_x ... am_x |
%            | a1_y ... am_y |
%
% b: vector of distal attachment points in {b} on the form 
%    b   =   | b1_x ... bm_x |
%            | b1_y ... bm_y |
%
% y: vector with pose of the platform in {i} containg the carthesian
%    position vector r and the angle phi denoting the roation about the 
%    z axis [rad]
%
%    y   =   |  r  | =  |  x  | 
%            | phi |    |  y  | 
%                       | phi | 
%
% Outputs:
% L: x and y component of cable i = 1,...,m
%
%    L   =   | l1_x ... lm_x |
%            | l1_y ... lm_y |
% 
% l: length of cable i = 1,...,m
%    
%    l   =   | l1 ... lm|
%
% u: normalized leght of l
%
%    u   =   Li/li
%
%
% Author:    Elias Olsen Almenningen
% Date:      04 Oct 2023
% Revisions: 08 Nov 2023
%            Updated input and outputs to better fit with WrencMatrix()

m           = length(a);



r           = y(1:2);
phi         = y(3);
R           = [ cos(phi) -sin(phi);
                sin(phi)  cos(phi)];

% preallocating memory for output vectors
L           = zeros(2,m);  
l           = zeros(1,m);
u           = zeros(2,m);
% Cable Vector
for i = 1:m
    L(:,i) = a(:,i) - r - R*b(:,i);
end

% Cable Lengths
for i = 1:m
    l(i) = norm(L(:,i));
end

% Unit Cable Vectors
for i = 1:m
    u(:,i) = L(:,i)/l(i);
end
end