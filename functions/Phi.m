function phi = Phi(a,y,b,l)
% phi = Phi(a,y,b,l) calculates the objective function phi(l,r,R) 
% eq. (4.46) in Pott, p 146
%
%   a   =   | a1_x ... am_x |
%           | a1_y ... am:y |
%
%   b   =   | b1_x ... bm_x |
%           | b1_y ... bm:y |
%
%   l   =   | l1 ... lm|
%
%   y   =   |  x  |
%           |  y  |
%           | phi |
%
% Author:    Elias Olsen Almenningen
% Date:      16.10.2023
% Revisions: 

% Preallocate memory
phi = zeros(4,1);    

% Calculate g = nu_i^2  eq. (4.57) in Pott p.148
for i = 1:4
    phi(i) = ( (a(1,i) - y(1) - b(1,i)*cos(y(3)) + b(2,i)*sin(y(3)))^2 ...
        +      (y(2) - a(2,i) + b(2,i)*cos(y(3)) + b(1,i)*sin(y(3)))^2 ...
        -       l(i)^2 )^2;
end
end