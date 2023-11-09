function [f, flag] = Kernel_translation(A,w,f_min,f_max,level)
% Calculate the kernel h of A
% level: 1 = min, 2 = mid, 3 = max
m = length(A);
A_t = A';

h = null(A');
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
end
for i = 1:size(f)
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