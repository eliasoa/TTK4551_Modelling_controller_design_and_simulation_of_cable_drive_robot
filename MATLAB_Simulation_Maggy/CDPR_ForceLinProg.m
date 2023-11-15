% Force Computations of CRPM CDPR
% as a linear constrained optimization problem
%
% min c^T f   s.t   A^T f = -w_p
%                 fmin <= f <= fmax
%

function f = CDPR_ForceLinProg(A,wp,fmin,fmax)

% Bounds for OptProblem
lb = [fmin;fmin;fmin;fmin];
ub = [fmax;fmax;fmax;fmax];

c = [1;1;1;1];

% Equality Constraint
Aeq = A';
Beq = -wp;

% Solve Linear Problem
f = linprog(-c', [],[],Aeq,Beq,lb,ub);
end