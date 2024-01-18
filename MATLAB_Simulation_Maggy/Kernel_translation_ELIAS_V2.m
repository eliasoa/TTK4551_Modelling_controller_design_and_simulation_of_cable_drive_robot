function [f, flag] = Kernel_translation(A,w,f_min,f_max,level)
% [f, flag] = Kernel_translation(A,w,f_min,f_max,level)
% Kernel_Translation the vector of feasible cable tensions
% If there exists no solution, flag is set to true and f = 0
%
% Inputs:
% A: matrix containing unit vector and cross product (in 2D) between distal
% attachment point and unit vector
%
%    A  =   | u_1 ... b_1 X u_1 |
%           |  .   .      .     |
%           |  .   .      .     |
%           |  .   .      .     |
%           | u_m ... b_m X u_m |
%
% w: wrench effecting the MP
%
%    w  =   | x y z |
%
% f_min: scalar minimum cable tension threshold
%
% f_max: scalar maximum cable tension threshold
%
% level: placement of cable tensions. If level = 1, cable tensions will be
% close or equal to f_min. If level = 2, cable tensions will lie between
% f_min and f_max. If level = 3, cable tensions will lie close to or equal
% to f_max
% 
%    level = 1, 2 or 3
%
% Outputs:
% f: vector of cable tensions
%    f  =   | f_1 |
%           |  .  |
%           | f_m |
%
% flag: flag is cable tensions are not feasible
%    flag = true or false
%
%
% Author:    Elias Olsen Almenningen
% Date:      17 Dec 2023
% Revisions: 




% Calculate the kernel h of A^T
m = length(A);
A_t = A';

[~, ~, V] = svd(A_t);
h = V(:, end);

% Check if signs are equal
if ~all(sign(h) == sign(h(1)))
    flag = true;
    f = [0;0;0;0];
    return;
end

A_pseudo = pinv(A_t);
 
f_lsq = -A_pseudo * w;

%% Equation 4.4 in Verhoven
% f_lsq = f_0

% min lambda
lambda_l = zeros(m,1);
for i = 1:m
    lambda_l(i) = (f_min - f_lsq(i))/h(i);
end
lambda_min = max(lambda_l);

% max lambda 
lambda_h = zeros(m,1);
for i = 1:m
    lambda_h(i) = (f_max - f_lsq(i))/h(i);
end
lambda_max = min(lambda_h);

if level == 1
    f = -A_pseudo*w + h*lambda_min;
elseif level == 2
    delta_lambda = (lambda_max - lambda_min)/2;
    f = -A_pseudo*w + h*(lambda_min+delta_lambda);
elseif level == 3
    f = -A_pseudo*w + h*lambda_max;
else
    f = [0 0 0 0]'; % Fordi simulink
end

for i = 1:4
    if f(i) > f_max
        flag = true;
    elseif f(i) < f_min
        flag = true;
    else 
        flag = false;
    end
    if flag
        break
    end
end
end