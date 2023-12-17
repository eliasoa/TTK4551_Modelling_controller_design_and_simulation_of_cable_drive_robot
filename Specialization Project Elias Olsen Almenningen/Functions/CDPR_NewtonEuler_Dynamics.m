function q_ddot = CDPR_NewtonEuler_Dynamics(f, A_transposed, w_p, m, Jz)

% Mass-Inertia matrix
M = diag([m,m,Jz]);

% Solve Newton-Euler equation for state
q_ddot = (M\(A_transposed * f + w_p));

% % Extract accelerations
% ddx = q_ddot(1);
% ddy = q_ddot(2);
% ddphi = q_ddot(3);