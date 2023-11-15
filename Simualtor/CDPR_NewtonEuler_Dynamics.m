function q_ddot = CDPR_NewtonEuler_Dynamics(q_dot, f, A_transposed, w_p, m, Jz)

% Mass-Inertia matrix
M = diag([m,m,Jz]);

% Damping Matrix
D = diag([0,0,0]);

% Solve Newton-Euler equation for state
q_ddot = (M\(A_transposed * f + w_p + D*q_dot));

% % Extract accelerations
% ddx = q_ddot(1);
% ddy = q_ddot(2);
% ddphi = q_ddot(3);