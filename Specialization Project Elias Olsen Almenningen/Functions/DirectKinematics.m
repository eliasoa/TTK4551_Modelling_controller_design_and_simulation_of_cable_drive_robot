function [y] = DirectKinematics(a,b,l)
% [y] = DirectKinematics(a,b,l) is implementation of Algorithm 3.16 in 
% METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS 2nd Edition, April 2004 by
% K. Madsen, H.B. Nielsen, O. Tingleff
%
% It is specifically written to solve the direct kinematics of an
% overconstraind Cable Driven Paralell Robot
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
% l: length of cable i = 1,...,m
%    
%    l   =   | l1 ... lm|
%
% Outputs:
% y: vector with pose of the platform in {i} containg the carthesian
%    position vector r and the angle phi denoting the roation about the 
%    z axis [rad]
%
%    y   =   |  r  | =  |  x  | 
%            | phi |    |  y  | 
%                       | phi | 
%
%
% Author: Elias Olsen Almenningen
% Date: 16.10.2023
% Revisions:

r0          = InitialPose(l,a,b);   % Calculate inintal pose
phi0        = 0;                    % Initial estimate of phi
y           = [r0;phi0];            % Initial pose
J           = Jacobian(y,a,b,l);    % Calculate Jacobian of inital pose

epsilon1    = 10e-17;               % Threshold parameter 1, set by user
epsilon2    = epsilon1;             % Threshold parameter 2, set by user


A           = J.'*J;                % Firs A is without + mu*I
g           = J'*Phi(a,y,b,l);      % 

tau         = 1e-6;                 % Chosen after footnote 3
mu          = tau*max(diag(A));     % Damping factor eq (3.14)
nu          = 2;                    % Factor preventing fluctutations in mu

iter_max    = 200;                  % Max number of iterations
iter        = 0;                    % Iteration number

%cond1       = false;               % norm_2( h_i ) < e_2 * (norm_2(y_i) + e2)
%cond2       = false;               % norm_2( J(y_i) * phi(y_i) ) < e1

stop        = false;
while ~stop && iter < iter_max
    iter = iter + 1;
    h = A\-g;
    cond1 = norm(h,2) < epsilon2 * (norm(y,2));
    if cond1                            % If small step size, stop
        stop = true;
        % disp("Condition 1 is true after ");
        % disp(iter); 
        % disp("iterations");
    else                                % Take step
        y_new = y + h;
        F_new = norm(Phi(a,y_new,b,l))^2;
        F = norm(Phi(a,y,b,l))^2;
        rho = ( F - F_new ) / ( 1/2 * h' * (mu * h - g) );
        if rho > 0
            y = y_new;
            J = Jacobian(y,a,b,l);
            A = J' * J + mu * eye(3);
            g = J' * Phi(a,y,b,l);
            cond2 = norm(g,2) < epsilon1; 
            if cond2
                stop = true;
                % disp("Condition 2 is true after ");
                % disp(iter); 
                % disp("iterations");
            end
            mu = mu * max(1/3,1-(2*rho-1)^3);
            nu = 2;
        else
            mu = mu * nu;
            nu = 2 * nu;
        end
    end
end
if iter == iter_max
    disp("Too many iterations");
end
end