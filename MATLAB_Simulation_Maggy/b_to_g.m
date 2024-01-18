function [p_g] = b_to_g(p,psi,b)
% p: origin of body frame in global coordinates
% psi: rotation of body frame
% b: point in body frame.
% p_g: Returns position of point b in global frame.

R = [cos(psi), -sin(psi);sin(psi),cos(psi)];
p_g = p + R*b;


