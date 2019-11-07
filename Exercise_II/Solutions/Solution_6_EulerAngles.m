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

%% Composing rotations:
Env = EnvironmentCLASS();

% Coordinate system I at the same location as the graphics co-sys:
I = CoSysCLASS(Env, eye(3));
I.color = green;
I.name = 'I';

% Create a coordinate system that is rotated by 30 deg about the 3-axis:
gamma = 30*pi/180;
A_IC1 =[cos(gamma), -sin(gamma), 0; 
         sin(gamma),  cos(gamma), 0;
         0,           0,          1];
C1 = CoSysCLASS(Env, A_IC1);
C1.color = red;
C1.name = 'C1';

%% Create a coordinate system that is rotated by 45 deg about the 2-axis:
beta = 45*pi/180;
A_C1C2 =[+cos(beta), 0, +sin(beta);
          0        , 1,  0;
         -sin(beta), 0, +cos(beta)];

A_IC2 = A_IC1 * A_C1C2;
C2 = CoSysCLASS(Env,A_IC2);
C2.color = blue;
C2.name = 'C2';


%% Create a coordinate system that is rotated by 60 deg about the 1-axis:
alpha = 60*pi/180;
A_C2C3 =[1,  0         ,  0;
         0, +cos(alpha), -sin(alpha);
         0, +sin(alpha), +cos(alpha)];
    
A_IC3 = A_IC1 * A_C1C2 * A_C2C3;
C3 = CoSysCLASS(Env,A_IC3);
C3.color = mag;
C3.name = 'C3';

%% Create a vector v
C3_v = [1;1;1];
v = VectorCLASS(Env, C3, C3_v);
v.color = black;
v.name = 'v';

%% Compute numerical results (via transformations and built in functions):
% in I:
disp(A_IC1 * A_C1C2 * A_C2C3 * C3_v)
disp(v.getCoords(I))
% in C1:
disp(A_C1C2 * A_C2C3 * C3_v)
disp(v.getCoords(C1))
% in C2:
disp(A_C2C3 * C3_v)
disp(v.getCoords(C2))
% in C3:
disp(C3_v)
disp(v.getCoords(C3))

print(gcf,'-r600','-djpeg','Problem_6_Output.jpg'); 




