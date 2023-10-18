function R = RotMat2D(theta)
% R = RotMat2Dd(theta) calculates rotation matrix in two dimensions about 
% the z-axis in radians
%
% Author:    Elias Olsen Almenningen
% Date:      04 Oct 2023
% Revisions: 

R = [   cos(theta) -sin(theta); 
        sin(theta)  cos(theta)];
end