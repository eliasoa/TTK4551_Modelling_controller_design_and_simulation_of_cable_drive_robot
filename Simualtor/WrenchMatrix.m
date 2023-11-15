function A = WrenchMatrix(b, u, y)
% A = WrenchMatrix(a, b, l)
m           = 4; %lenght(b);
phi         = y(3);
R           = [ cos(phi) -sin(phi);
                sin(phi)  cos(phi)];
b_F = zeros(2,m);
for i = 1:m
   b_F(:,i) = R*b(:,i); 
end


% Cross-product in 2 dimensions
h           = zeros(1,m);
for i = 1:m
    h(i) = b_F(1,i)*u(2,i) - b_F(2,i)*u(1,i);
end


% Assemble A matrix
A           = [ u(:,1)' h(1);
                u(:,2)' h(2);
                u(:,3)' h(3);
                u(:,4)' h(4)];
end