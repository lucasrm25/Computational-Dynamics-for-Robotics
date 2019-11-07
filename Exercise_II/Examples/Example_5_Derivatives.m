% Script to demonstrate the use of the SymbolicMath Toolbox
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   10/12/2018
%   v21
%
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


%% Symbolic analysis:
% Declear variables as real symbolic values:
gamma = sym('gamma','real');
% or shorter (does the same thing):
syms gamma real;

%% Define transformation as a simple rotation:
A_IC = [+cos(gamma), -sin(gamma), 0;
        +sin(gamma), +cos(gamma), 0;
         0         ,  0         , 1];
disp('A_IC:');
disp(A_IC);

%% Create inverse by transposing:
A_CI = A_IC';
disp('A_CI:');
disp(A_CI);


%% Direct time derivatives do work in matlab (see next section) but we work
% with a little trick instead that gives us more control: 
% Since 'A_IC' is only a function of 'gamma' and the time derivative of
% 'gamma' is suppposed to be known (it's called 'gamma_dot'), we can apply
% the chain-rule for differentiation: 
% given f(x(t)) -> d_(f(x(t))/d_t = d_f/d_x * d_x/d_t = d_f/d_x * x_dot.
%
% I.e.,: A_IC_dot = d_A_IC/d_t = d_A_IC/d_gamma * gamma_dot
%
% Since A_IC is a matrix, we do this element per element:
% 1) Define gamma_dot as a symbolic variable:
syms gamma_dot real;
A_IC_dot = sym(zeros(3));  % Allocate space for results:
for i = 1:3 % Columns
    for j = 1:3 % Rows
        % Compute partial derivative using the 'diff'-command. Then
        % multiply result by 'gamma_dot':
        A_IC_dot(i,j) = diff(A_IC(i,j),gamma)*gamma_dot; 
    end
end     
% Show original matrix:
disp('A_IC:');
disp(A_IC);
% And its derivative:
disp('A_IC_dot:');
disp(A_IC_dot);

%% Compute the matrix omega:
C_omega_IC = A_CI*A_IC_dot;
disp('C_omega_IC:');
disp(C_omega_IC);

% Symbolically simplify the result:
C_omega_IC = simplify(expand(C_omega_IC));
disp('C_omega_IC:');
disp(C_omega_IC);

%% Symbolic analysis (Take two, using direct time derivatives...):
% Declear t as a variable and gamma as a function of time:
syms t real;
syms gamma2(t);

% Define transformation as a simple rotation:
A_IC2 = [+cos(gamma2), -sin(gamma2), 0;
         +sin(gamma2), +cos(gamma2), 0;
          0         ,  0         , 1];
disp('A_IC:');
disp(A_IC2);

% Create inverse by transposing:
A_CI2 = A_IC2.';  % Important, don't use A_IC', since you would get the conjugated transpose
disp('A_CI:');
disp(A_CI2);

% Use the built in function to create the derivatives:
A_IC_dot2 = diff(A_IC2,t);
disp('A_IC_dot:');
disp(A_IC_dot2);

% Compute the matrix omega:
C_omega_IC2 = A_CI2*A_IC_dot2;
disp('C_omega_IC:');
disp(C_omega_IC2);

% Symbolically simplify the result:
C_omega_IC2 = simplify(expand(C_omega_IC2));
disp('C_omega_IC:');
disp(C_omega_IC2);

% Let's stop here.  The results are not as pretty (since we can't tell
% MATLAB that D(gamma)(t) = gamma_dot
% I hence recommend the first version, which also explicitly tells you that
% velocities are linear functions. 


%% Visualize this result with some examples:
% Create numerical functions from the analytical ones:
matlabFunction(A_IC,'file','A_IC_fct.m', 'vars', gamma);
matlabFunction(C_omega_IC,'file','C_omega_IC_fct.m', 'vars', [gamma, gamma_dot]);

%% Define numerical values:
gamma_num     = 30*pi/180;
gamma_dot_num = 1;

%% Generate numerical representation of these matrices:
A_IC_num       = A_IC_fct(gamma_num);
C_omega_IC_num = C_omega_IC_fct(gamma_num, gamma_dot_num);
%% Create a 'physical' (graphical) environment:
Env = EnvironmentCLASS();

%% Inertial frame:
I = CoSysCLASS(Env,eye(3));
I.color = green;
I.name = 'I';

%% Rotated frame:
C = CoSysCLASS(Env,A_IC_num);
C.color = red;
C.name = 'C';

%% Create a vector in C-coordinates:
C_u = [0.5;0.5;0.0];
u = VectorCLASS(Env, C, C_u);
u.color = black;

%% Compute the derivative of this vector in C-coordinates and create a new
% vector: 
dot_C_u = [0;0;0]; % That's d(Cu)/dt
C_uDot = dot_C_u + C_omega_IC_num*C_u;
u_dot = VectorCLASS(Env, C, C_uDot);
u_dot.color = grey;

%% Animate for different values of gamma ( t = 0..pi) over n steps: 
n = 200;
delta_t = 2*pi/n;
gamma_num     = 30*pi/180;
gamma_dot_num = 1;
for i = 1:n
    % Euler forward integration:
    gamma_num = gamma_num + gamma_dot_num*delta_t;
    % Set this value to the rotated frame:
    C.A_IC = A_IC_fct(gamma_num);
    % Update the vector:
    u.setCoords(C, C_u);
    
    % And its derivative
    dot_C_u = [0;0;0]; % That's d(Cu)/dt
    C_uDot = dot_C_u + C_omega_IC_fct(gamma_num, gamma_dot_num)*C_u;
    u_dot.setCoords(C, C_uDot);
    
    drawnow();
end
