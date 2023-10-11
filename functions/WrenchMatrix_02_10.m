function A_transposed = WrenchMatrix(u, b, R) 
% u: 2x4 Matrix with four 2-dimensional unit vectors
% b: 2x2 Matrix with two 2-dimensional vectors (expressed in BODY)

% Two-dimensional Vectors
u1 = [u(:,1)];
u2 = [u(:,2)];
u3 = [u(:,3)];
u4 = [u(:,4)];

b1 = R*[b(:,1)];    % Transformed into INERTIA-frame
b2 = R*[b(:,2)];    % Transformed into INERTIA-frame

% Cross-product in 2 dimensions
h1 = b1(1)*u1(2) - b1(2)*u1(1);
h2 = b1(1)*u2(2) - b1(2)*u2(1);
h3 = b2(1)*u3(2) - b2(2)*u3(1);
h4 = b2(1)*u4(2) - b2(2)*u4(1);

% Columns of A_transposed
A1 = [u1;h1];
A2 = [u2;h2];
A3 = [u3;h3];
A4 = [u4;h4];

% Wrench Matrix
A_transposed = [A1 A2 A3 A4];
end
