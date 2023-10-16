function PHI = Phi(a,y,b,l)
PHI = zeros(4,1);    
for i = 1:4
    PHI(i) = ( (a(1,i) - y(1) - b(1,i)*cos(y(3)) + b(2,i)*sin(y(3)))^2 ...
        +      (y(2) - a(2,i) + b(2,i)*cos(y(3)) + b(1,i)*sin(y(3)))^2 ...
        -       l(i)^2 )^2;
end
end