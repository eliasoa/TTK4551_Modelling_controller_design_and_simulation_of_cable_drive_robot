function J = Jacobian(y,a,b,l)
% J = Jacobian(y,a,b) calculates the Jacobian using the gradient of g
% Eq 4.43 and 4.47 in Cable-Driven Parallel Robots by Andreas Pott
% Author:    Elias Olsen Almenningen
% Date:      11 Oct 2023
% Revisions: 

[~, Gx, Gy, Gphi] = constraintGradient(y,a,b,l);

J = [Gx(1) Gy(1) Gphi(1);
     Gx(2) Gy(2) Gphi(2);
     Gx(3) Gy(3) Gphi(3);
     Gx(4) Gy(4) Gphi(4)];

end