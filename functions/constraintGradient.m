function [G, Gx, Gy, Gphi] = constraintGradient(y,a,b,l)
% G = gradient(y,a,b) calculates the gradient of the objective function
% g(l,r,R) = sum_i^m nu^2_i (eq. 4.46 in Pott, p 164) for 4 cables
% where a is the row vector containg the proximal anchor points
% and b is the row vector containg the distal anchor points and l is the
% row vector containg the cable lengths
%
%   a   =   | a1_x ... am_x |
%           | a1_y ... am:y |
%
%   b   =   | b1_x ... bm_x |
%           | b1_y ... bm:y |
%
%   l   =   | l1 ... lm|
%
% Author:    Elias Olsen Almenningen
% Date:      08 Oct 2023
% Revisions: 
m = 4;
r = y(1:2);
phi = y(3);
Gx = [0 0 0 0]';
Gy = [0 0 0 0]';
Gphi = [0 0 0 0]';
for i = 1:m
    % Gx(i) = -2*((a(1,i) - r(1) - b(1,i)*cosd(phi) + b(2,i)*sind(phi))^2 + (r(2) - a(2,i) + b(2,i)*cosd(phi) + b(2,i)*sind(phi))^2 - l(i)^2)*(2*a(1,i) - 2*r(1) - 2*b(1,i)*cosd(phi) + 2*b(2,i)*sind(phi));
    Gx(i) = dgdx(r(1),r(2),a(1,i),a(2,i),b(1,i),b(2,i),phi,l(i));
    % Gy(i) = 2*((a(1,i) - r(1) - b(1,i)*cosd(phi) + b(2,i)*sind(phi))^2 + (r(2) - a(2,i) + b(2,i)*cosd(phi) + b(1,i)*sind(phi))^2 - l(i)^2)*(2*r(2) - 2*a(2,i) + 2*b(2,i)*cosd(phi) + 2*b(1,i)*sind(phi));
    Gy(i) = dgdy(r(1),r(2),a(1,i),a(2,i),b(1,i),b(2,i),phi,l(i));
    % Gphi(i) = 2*(2*(b(2,i)*cosd(phi) + b(1,i)*sind(phi))*(a(1,i) - r(1) - b(1,i)*cosd(phi) + b(2,i)*sind(phi)) + 2*(b(1,i)*cosd(phi) - b(2,i)*sind(phi))*(r(2) - a(2,i) + b(2,i)*cosd(phi) + b(1,i)*sind(phi)))*((a(1,i) - r(1) - b(1,i)*cosd(phi) + b(2,i)*sind(phi))^2 + (r(2) - a(2,i) + b(2,i)*cosd(phi) + b(1,i)*sind(phi))^2 - l(i)^2);
    Gphi(i) = dgdphi(r(1),r(2),a(1,i),a(2,i),b(1,i),b(2,i),phi,l(i));
end
G = [sum(Gx);sum(Gy);sum(Gphi)];
