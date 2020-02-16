%% Init everything
% Clear memory, command window, and figures
clear all
close all
clc
% Some RGB values:
red   = [1;0;0]; 
green = [0;1;0]; 
blue  = [0;0;1]; 
black = [0;0;0]; 
grey  = [0.3;0.3;0.3];
mag   = [1;0;1];
yel   = [1;1;0];

%% Symbolic analysis for three subsequent rotations:
% Declear all variables as real, without telling MATLAB that they are
% functions of time: 
syms alpha alpha_dot real;
syms beta  beta_dot  real;
syms gamma gamma_dot real;

% Define transformation as a simple rotation about the 3-axis:
A_IC1 = [+cos(gamma), -sin(gamma), 0;
         +sin(gamma), +cos(gamma), 0;
          0         ,  0         , 1];
% Define a second transformation about the 2-axis::
A_C1C2= [+cos(beta), 0, +sin(beta);
          0        , 1, 0;
         -sin(beta), 0, +cos(beta)];
% Define a third transformation about the 1-axis::
A_C2C3= [1,  0         ,  0;
         0, +cos(alpha), -sin(alpha);
         0, +sin(alpha), +cos(alpha)];
% Combine transformations:
A_IC = A_IC1*A_C1C2*A_C2C3;
% Create inverse by transposing:
A_CI = A_IC';
% Compute derivatives:
A_IC_dot = sym(zeros(3));  % Allocate space for results:
for i = 1:3 % Columns
    for j = 1:3 % Rows
        % Compute partial derivative using the 'diff'-command. Then
        % multiply result by 'beta_dot' and 'gamma_dot'. Sum over all
        % terms: 
        A_IC_dot(i,j) = diff(A_IC(i,j),alpha)*alpha_dot + ...
                        diff(A_IC(i,j),beta)*beta_dot + ...
                        diff(A_IC(i,j),gamma)*gamma_dot; 
    end
end     
% Compute the matrix omega:
C_omega_IC = A_CI*A_IC_dot;
% ... and symbolically simplify the result:
C_omega_IC = simplify(expand(C_omega_IC));
% Extract the vector of angular velocity:
C_omega_IC_vec = [C_omega_IC(3,2);
                  C_omega_IC(1,3);
                  C_omega_IC(2,1)];
C_omega_IC_vec = simplify(expand(C_omega_IC_vec));
disp('C_omega_IC_vec:');
disp(C_omega_IC_vec);

    
% From this, we extract by hand:
% *************************************************************************
% ToDo:                                                                  
% Finish the expression for B to better understand the linear nature of the
% cardan velocities:
% *************************************************************************
B = ;
disp('B:')
disp(B)
 
% We can now create the inverse of A_IC*B
AB_inv = inv(A_IC*B);
AB_inv = simplify(expand(AB_inv));
disp('AB_inv:');
disp(AB_inv);
% And create a matlab function for this matrix, as well as for the transformation A_IC:
matlabFunction(AB_inv,'file','AB_inv_fct.m', 'vars', [alpha, beta, gamma]);
matlabFunction(A_IC,'file','A_IC_fct.m', 'vars', [alpha, beta, gamma]);

%% Create an animation in which the angular velocity is constant and
%% represents a rotation about the direction of [1;1;1]) with unit velocity
%% i.e.: I_omega_IC_vec = sqrt(1/3)*[1;1;1].
%
% Possibility two (computing cardan angular rates and integrating them):
%
% 
n = 100;
delta_t = 2*pi/n;
% Initialize the vector of Cardan angles:
alpha_num = 0;
beta_num  = 0;
gamma_num = pi/2;
% Initialize the rotation matrix
A_IC = [+cos(gamma_num), -sin(gamma_num), 0;
        +sin(gamma_num), +cos(gamma_num), 0;
         0             ,  0             , 1];
% Create a 'physical' (graphical) environment:
Env = EnvironmentCLASS();
% Inertial frame:
I = CoSysCLASS(Env,eye(3));
I.color = green;
I.name = 'I';
% Rotated frame:
C = CoSysCLASS(Env,A_IC);
C.color = red;
C.name = 'C';
% Define rotation velocity
I_omega_IC_vec = sqrt(1/3)*[1;1;1];
% Show angular velocity vector:
v_omega = VectorCLASS(Env, I, I_omega_IC_vec);
v_omega.color = yel;
% Animate:
for i = 1:n
    % Compute derivatives of cardan angles:
% *************************************************************************
% ToDo:                                                                  
% Write code to compute the vector of derivatives of the cardan angles
% [alpha, beta, gamma]' to implement your knowledge about derivatives of
% transformations. Do so using the automatically created matlab function
% 'AB_inv_fct.m'.   
% *************************************************************************
    card_vec_dot = ;
    
    % Integration:
% *************************************************************************
% ToDo:                                                                  
% Write code to update the three angles with an Euler forward integration
% step to learn how simple integration can be done in Matlab.
% *************************************************************************
    alpha_num = alpha_num + ;
    beta_num  = beta_num + ;
    gamma_num = gamma_num + ;
    
    % Compute new transformation from cardan angles:
    A_IC_num = A_IC_fct(alpha_num, beta_num, gamma_num);
    C.A_IC = A_IC_num;
	drawnow();
end
print(gcf,'-r600','-djpeg','Problem_8b_Output.jpg');    
