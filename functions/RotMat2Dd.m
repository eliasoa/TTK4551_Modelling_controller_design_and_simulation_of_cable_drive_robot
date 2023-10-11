function R = RotMat2Dd(theta)
% R = RotMat2Dd(theta) calculates rotation matrix in two dimensions about 
% the z-axis in degrees
%
% Author:    Elias Olsen Almenningen
% Date:      04 Oct 2023
% Revisions: 

R = [   cosd(theta) -sind(theta); 
        sind(theta)  cosd(theta)];
end