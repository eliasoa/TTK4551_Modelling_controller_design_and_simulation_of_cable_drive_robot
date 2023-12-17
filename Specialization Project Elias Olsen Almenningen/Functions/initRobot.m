function [a, b, Wp, m, Iz] = initRobot(MP_len_x, MP_len_y, F_len_x, F_len_y)

% Physical parameters 
g               = 9.81;                     % m/s^2

% Mobile Platform parameters
l               = MP_len_x;                 % m
h               = MP_len_y;                 % m
d               = 0.01;                     % m
V               = l*h*d;                    % m^3
rho             = 2710;                     % kg/m^3
m               = V*rho;                    % kg
m               = 0.5;                      % TEMPORARY
Iz              = 1/12*m*(l^2+h^2);         % kg m^2

% Wrench due to gravity
Wp              = m*[0 -g 0]';

% Lengths of mobile platform
% MP_len_x                                  % [m]
% MP_len_y                                  % [m]
        
% Dimension of the frame
% F_len_x                                   % [m]                
% F_len_y                                   % [m]

% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
a1              = [-F_len_x/2;-F_len_y/2];  
a2              = [-F_len_x/2;F_len_y/2];
a3              = [F_len_x/2;F_len_y/2];    
a4              = [F_len_x/2;-F_len_y/2];   

% Cable attachment point PLATFORM
b1              = [-MP_len_x/2;-MP_len_y/2];  
b2              = [-MP_len_x/2;MP_len_y/2];   
b3              = [MP_len_x/2;MP_len_y/2];    
b4              = [MP_len_x/2;-MP_len_y/2];   

% Matrices of the attachment points
a               = [a1 a2 a3 a4];        
b               = [b1 b2 b3 b4];
end