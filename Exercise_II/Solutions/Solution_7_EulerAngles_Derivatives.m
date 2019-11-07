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
% Combine transformations:
A_IC2 = A_IC1*A_C1C2;
     
% Define a third transformation about the 1-axis::
A_C2C3= [1,  0         ,  0;
         0, +cos(alpha), -sin(alpha);
         0, +sin(alpha), +cos(alpha)];

% Combine transformations:
A_IC3 = A_IC2*A_C2C3;
disp('A_IC3:');
disp(A_IC3);

% Create inverse by transposing:
A_C3I = A_IC3';
disp('A_C3I:');
disp(A_C3I);

% Direct time derivatives will only work for the next matlab version (at
% least that's what they promise, so far I can't see it).  Here we work
% with a little trick: 
% Since 'A_IC3' is a function of 'beta' and 'gamma' and the time
% derivatives of 'beta' and 'gamma' are suppposed to be known ('beta_dot'
% and 'gamma_dot'), we can apply the chain-rule for differentiation: 
% given f(x(t), y(t)) -> d_(f(x(t), y(t))/d_t = 
%                        = d_f/d_x * d_x/d_t + d_f/d_y * d_y/d_t 
%                        = d_f/d_x * x_dot + d_f/d_y * y_dot.
%
% I.e.,: A_IC3_dot = d_A_IC3/d_t = 
%                  = d_A_IC3/d_alpha * alpha_dot + d_A_IC3/d_beta * beta_dot + d_A_IC3/d_gamma * gamma_dot
%
% Since A_IC3 is a matrix, we do this element per element:
A_IC3_dot = sym(zeros(3));  % Allocate space for results:
for i = 1:3 % Columns
    for j = 1:3 % Rows
        % Compute partial derivative using the 'diff'-command. Then
        % multiply result by 'beta_dot' and 'gamma_dot'. Sum over all
        % terms: 
        A_IC3_dot(i,j) = diff(A_IC3(i,j),alpha)*alpha_dot + ...
                         diff(A_IC3(i,j),beta)*beta_dot + ...
                         diff(A_IC3(i,j),gamma)*gamma_dot; 
    end
end     
disp('A_IC3_dot:');
disp(A_IC3_dot);

% Compute the matrix omega:
C3_omega_IC3 = A_C3I*A_IC3_dot;
% ... and symbolically simplify the result:
C3_omega_IC3 = simplify(expand(C3_omega_IC3));
disp('C3_omega_IC3:');
disp(C3_omega_IC3);

C3_omega_IC3_vec = [C3_omega_IC3(3,2);
                    C3_omega_IC3(1,3);
                    C3_omega_IC3(2,1)];
C3_omega_IC3_vec = simplify(expand(C3_omega_IC3_vec));
disp('C3_omega_IC3_vec:');
disp(C3_omega_IC3_vec);



%% Visualize this result with some examples:
% Create numerical functions from the analytical ones:
matlabFunction(A_IC1,'file','A_IC1_fct.m', 'vars', [alpha, beta, gamma]);
matlabFunction(A_IC2,'file','A_IC2_fct.m', 'vars', [alpha, beta, gamma]);
matlabFunction(A_IC3,'file','A_IC3_fct.m', 'vars', [alpha, beta, gamma]);
matlabFunction(C3_omega_IC3,'file','C3_omega_IC3_fct.m', 'vars', [alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot]);
matlabFunction(C3_omega_IC3_vec,'file','C3_omega_IC3_vec_fct.m', 'vars', [alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot]);


% Define numerical values:
alpha_num     = 60*pi/180;
beta_num      = 45*pi/180;
gamma_num     = 30*pi/180;
alpha_dot_num = 1;
beta_dot_num  = 1;
gamma_dot_num = 1;

% Generate numerical representation of these matrices:
A_IC1_num            = A_IC1_fct(alpha_num, beta_num, gamma_num);
A_IC2_num            = A_IC2_fct(alpha_num, beta_num, gamma_num);
A_IC3_num            = A_IC3_fct(alpha_num, beta_num, gamma_num);
C3_omega_IC3_num     = C3_omega_IC3_fct(alpha_num, beta_num, gamma_num, alpha_dot_num, beta_dot_num, gamma_dot_num);
C3_omega_IC3_vec_num = C3_omega_IC3_vec_fct(alpha_num, beta_num, gamma_num, alpha_dot_num, beta_dot_num, gamma_dot_num);

% Create a 'physical' (graphical) environment:
Env = EnvironmentCLASS();

% Inertial frame:
I = CoSysCLASS(Env,eye(3));
I.color = green;
I.name = 'I';

% First rotated frame:
C1 = CoSysCLASS(Env,A_IC1_num);
C1.color = red;
C1.name = 'C1';

% Second rotated frame:
C2 = CoSysCLASS(Env,A_IC2_num);
C2.color = blue;
C2.name = 'C2';

% Third rotated frame:
C3 = CoSysCLASS(Env,A_IC3_num);
C3.color = mag;
C3.name = 'C3';

% Create a vector in C2-coordinates:
C3_u = [1;1;1];
u = VectorCLASS(Env, C3, C3_u);
u.color = black;
u.name = 'u';

% Compute the derivative of this vector in C-coordinates and create a new
% vector: 
C3_u_dot = C3_omega_IC3_num*C3_u;
u_dot = VectorCLASS(Env, C3, C3_u_dot);
u_dot.color = grey;
u_dot.name = 'u\_dot';

% Show angular velocity vector:
v_omega = VectorCLASS(Env, C3, C3_omega_IC3_vec_num);
v_omega.color = yel;
v_omega.name = 'v_{\omega}';

%% Animate for different values of beta and gamma  ( t = 0..pi):
n = 100;
delta_t = pi/n;
alpha_num     = 60*pi/180;
beta_num      = 45*pi/180;
gamma_num     = 30*pi/180;
alpha_dot_num = 1;
beta_dot_num  = 1;
gamma_dot_num = 1;
for i = 1:n
    % Euler forward integration:
    alpha_num = alpha_num + alpha_dot_num*delta_t;
    beta_num  = beta_num  + beta_dot_num*delta_t;
    gamma_num = gamma_num + gamma_dot_num*delta_t;
    
    A_IC1_num            = A_IC1_fct(alpha_num, beta_num, gamma_num);
    A_IC2_num            = A_IC2_fct(alpha_num, beta_num, gamma_num);
    A_IC3_num            = A_IC3_fct(alpha_num, beta_num, gamma_num);
    C3_omega_IC3_num     = C3_omega_IC3_fct(alpha_num, beta_num, gamma_num, alpha_dot_num, beta_dot_num, gamma_dot_num);
    C3_omega_IC3_vec_num = C3_omega_IC3_vec_fct(alpha_num, beta_num, gamma_num, alpha_dot_num, beta_dot_num, gamma_dot_num);
 
    % Set this value to the rotated frames:
    C1.A_IC = A_IC1_num;
    C2.A_IC = A_IC2_num;
    C3.A_IC = A_IC3_num;
    % Update the vectors:
    u.setCoords(C3, C3_u);
    C3_u_dot = C3_omega_IC3_num*C3_u;
    u_dot.setCoords(C3, C3_u_dot);
    v_omega.setCoords(C3, C3_omega_IC3_vec_num);
    % Update graphics:
    drawnow();
end
print(gcf,'-r600','-djpeg','Problem_7_Output.jpg');

