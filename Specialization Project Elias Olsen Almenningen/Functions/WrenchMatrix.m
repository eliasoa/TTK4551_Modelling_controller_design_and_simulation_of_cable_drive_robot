function A = WrenchMatrix(b, u, y)
% A = WrenchMatrix(b, u, y)
% WrenchMatrix calculates the transpose of the structure matrix 
% Equation 3.5 in Cable-Driven Parallel Robots 
% by Andreas Pott
%
% Inputs:
% b: vector of distal attachment points in {b} on the form 
%    b   =   | b1_x ... bm_x |
%            | b1_y ... bm_y |
%
% u: normalized leght of l
%
%    u   =   Li/li
%
% y: vector with pose of the platform in {i} containg the carthesian
%    position vector r and the angle phi denoting the roation about the 
%    z axis [rad]
%
%    y   =   |  r  | =  |  x  | 
%            | phi |    |  y  | 
%                       | phi | 
%
%
% Outputs:
% A: matrix containing unit vector and cross product (in 2D) between distal
% attachment point and unit vector
%
%    A  =   | u_1 ... b_1 X u_1 |
%           |  .   .      .     |
%           |  .   .      .     |
%           |  .   .      .     |
%           | u_m ... b_m X u_m |
%
%
% Author:    Elias Olsen Almenningen
% Date:      17 Dec 2023
% Revisions: 


m           = 4; %lenght(b);
phi         = y(3);
R           = [ cos(phi) -sin(phi);
                sin(phi)  cos(phi)];
b_F = zeros(2,m);
for i = 1:m
   b_F(:,i) = R*b(:,i); 
end


% Cross-product in 2 dimensions
h           = zeros(1,m);
for i = 1:m
    h(i) = b_F(1,i)*u(2,i) - b_F(2,i)*u(1,i);
end


% Assemble A matrix
A           = [ u(:,1)' h(1);
                u(:,2)' h(2);
                u(:,3)' h(3);
                u(:,4)' h(4)];
end