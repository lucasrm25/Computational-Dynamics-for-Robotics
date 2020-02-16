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

%% Create an animation in which the angular velocity is constant and
%% represents a rotation about the direction of [1;1;1]) with unit velocity
%% i.e.: I_omega_IC_vec = sqrt(1/3)*[1;1;1].
%
% Possibility one (integrating the direction cosine):
%
% Compute A_IC_dot from the relationships:
% C_omega_IC = A_CI*I_omega_IC*A_IC
% A_IC_dot = A_IC * C_omega_IC
%
% A_IC_dot = I_omega_IC * A_IC
% 
n = 100;
delta_t = 2*pi/n;
% Initialize the rotation matrix
gamma = pi/2;
A_IC = [+cos(gamma), -sin(gamma), 0;
        +sin(gamma), +cos(gamma), 0;
         0         ,  0         , 1];
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
% Define rotation velocity (as vector)
I_omega_IC_vec = sqrt(1/3)*[1;1;1];
% *************************************************************************
% ToDo:                                                                  
% Complete the statement to compute the matrix omega-tilde to learn how to
% implement a skew symmetric matrix:
% *************************************************************************
I_omega_IC = ;

% Show angular velocity vector:
v_omega = VectorCLASS(Env, I, I_omega_IC_vec);
v_omega.color = yel;
% Animate:
for i = 1:n
    % Compute derivative:
% *************************************************************************
% ToDo:                                                                  
% Complete the statement to compute the time-derivative of A_IC, to learn
% how to implement derivatives of transformations: 
% *************************************************************************
    A_IC_dot = ;
    % Integration:
    A_IC = A_IC + A_IC_dot*delta_t;
	C.A_IC = A_IC;
    drawnow();
end
print(gcf,'-r600','-djpeg','Problem_8a_Output.jpg');
    