% Function that calculates the vector nu = [nu1;nu2;nu3;nu4]
%, which is the constraint stated in the function g(y)
function nu = ObjectiveConstraint(y,a,b)

% Extract States from y
r = y(1:2);
phi = y(3);

% Memory allocation
nu = zeros(4,1);
m = 4;

for i=1:m
    ls = ((a(1,i)-r(1)-cosd(phi)*b(1,i) + sind(phi)*b(2,i))^2 + (a(2,i)-r(2)-sind(phi)*b(1,i) - cosd(phi)*b(2,i))^2)^(1/2);
    nu(i) = (r(1) + cosd(phi)*b(1,i) - sind(phi)*b(2,i) - a(1,i) )^2 +(r(2) + sind(phi)*b(1,i) + cosd(phi)*b(2,i) - a(2,i) )^2 - ls^2; % NOT SQUARED
end