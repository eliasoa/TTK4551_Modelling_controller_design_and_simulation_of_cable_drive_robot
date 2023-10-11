function A_t = structureMatrix(l,b)
% A_t = structureMatrix(l) calculates the structure matrix A^T using the
% calbe lengths vector l
%
% ONLY 1R2T FOR NOW
% A^T = | u_1 ... u_m | 
%       | h_1 ... h_2 | 
% where u_m is the nomralized l_m and h_m is the "2D"-cross procuct? 
% h_m = b_(m,x) * u_(m,y) - b_(m,y) * u_(m,x)
%
%
% Author:    Elias Olsen Almenningen
% Date:      04 Oct 2023
% Revisions: 

m = length(l);                          % Number of cables

% Preallocate memory
u = zeros(2,m);                         
h = zeros(1,m);

% Calculate u
for i = 1:m
    u(:,i) = l(:,i)/norm(l(:,i));        % Normalizes l_i
end

% Calculate h
for i = 1:m
    h(:,i) = b(1,i) * u(2,i) - b(2,i) * u(1,i);
end

% Assemble the matrix
A_t = [ u(:,1) u(:,2) u(:,3) u(:,4); 
        h(1)   h(2)   h(3)   h(4)    ];
end