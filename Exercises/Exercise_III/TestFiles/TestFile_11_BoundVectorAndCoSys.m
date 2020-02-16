clear all
close all
clc
red   = [1;0;0];  
green = [0;1;0];  
blue  = [0;0;1];  

% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();

% Create an inertial reference frame:
I = CoSysCLASS(env, eye(3));
I.color = green;
I.name = 'I';

% Create a bound coordinate system with some initial values:
C = BoundCoSysCLASS(env, eye(3), zeros(3,1));
C.color = red;
C.name = 'C';
% Define transformation as a simple rotation about the 3-axis, and update
% the bound CoSys accordingly:
gamma = 30*pi/180;
A_IC = [+cos(gamma), -sin(gamma), 0;
        +sin(gamma), +cos(gamma), 0;
         0         ,  0         , 1];
C.A_IC = A_IC;
% Do the same for the offset:     
I_r_IC = [1;2;0];
C.I_r_IC = I_r_IC;

% Create a bound vector system with some initial values:
v = BoundVectorCLASS(env, C, zeros(3,1), zeros(3,1));
v.color = blue;
v.name = 'vector';
% Redefine the vector and the offset:
C_v    = [0; -2; 0];
C_r_CO = [1;0;0];
v.setCoords(C, C_v, C_r_CO);

% Calling resetOutput, adjusts the scaling of the graphical output window,
% such that both coordinate systems are visible.
env.resetOutput();

% Save screen shot:
print(gcf,'-r600','-djpeg','Problem_11_Output.jpg');
