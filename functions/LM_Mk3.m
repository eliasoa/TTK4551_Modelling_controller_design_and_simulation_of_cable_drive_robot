function [y, iter] = LM_Mk3(a,b,l)
% [] = LM() is implementation of Algorithm 3.16 in METHODS FOR
% NON-LINEAR LEAST SQUARES PROBLEMS 2nd Edition, April 2004 by
% K. Madsen, H.B. Nielsen, O. Tingleff
%
% a is a vector
%
%
% Author: Elias Olsen Almenningen
% Date: 16.10.2023
% Revisions:

x0          = InitialPose(l,a,b); % Function that calculates inintal pose
phi         = 0;    % Setting intial estimate of phi to zero, because reasons
y           = [x0;phi];
J           = Jacobian(y,a,b,l);

epsilon1    = 1e-12;
epsilon2    = epsilon1;

tau = 1e-6;
A = J.'*J;
diag_A = diag(A);
mu = tau*max(diag_A);
g = J'*Phi(a,y,b,l);
h = A\-g;

iter_max    = 200;          % Max number of iterations
iter        = 0;            % Iteration number
% h           = [0.01;0.01;0.01];      % Inital step size

%cond1       = false;        % norm_2( h_i ) < e_2 * (norm_2(y_i) + e2)
%cond2       = false;        % norm_2( J(y_i) * phi(y_i) ) < e1
stop        = false;
while ~stop && iter < iter_max
    iter = iter + 1;
    cond1 = norm(h,2) < epsilon2 * (norm(y,2));
    if cond1                            % If small step size, stop
        stop = true;
        disp("Condition 1 is true after ");
        disp(iter); 
        disp("iterations");
    else                                % Take step
        y = y + h;
        J = Jacobian(y,a,b,l);
        A = J' * J + mu * eye(3);
        g = J' * Phi(a,y,b,l);
        h = (A\-g);
        cond2 = norm(g,2) < epsilon1; 
        if cond2
            stop = true;
            disp("Condition 2 is true after ");
            disp(iter); 
            disp("iterations");
        end
    end

    % disp("End of loop");
end
if iter == iter_max
    disp("Too many iterations");
end
end

