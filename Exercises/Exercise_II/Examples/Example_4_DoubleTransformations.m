% Script to further demonstrate the use of the following components:
% EnvironmentCLASS, VectorCLASS, and CoSysCLASS 
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
close all;
clearvars;

% Some RGB values:
red   = [1;0;0]; 
green = [0;1;0]; 
blue  = [0;0;1]; 
black = [0;0;0]; 
grey  = [0.3;0.3;0.3];
mag   = [1;0;1];
yel   = [1;1;0];

%% Create a 'physical' (graphical) environment:
Env = EnvironmentCLASS();

%% Coordinate system I at the same location as the graphics co-sys:
I = CoSysCLASS(Env, eye(3));
I.color = green;
I.name = 'I';

%% Create a coordinate system that is rotated by 20 deg about the 3-axis:
gamma = 20*pi/180;
A_IC1 =[cos(gamma), -sin(gamma), 0; 
        sin(gamma),  cos(gamma), 0;
        0,           0,          1];
C1 = CoSysCLASS(Env, A_IC1);
C1.color = red;
C1.name = 'C1';

%% Create a coordinate system that is rotated by 15 deg about the 2-axis:
beta = 15*pi/180;
A_C1C2 =[+cos(beta), 0, +sin(beta);
         0         , 1,  0;
         -sin(beta), 0, +cos(beta)];

C2 = CoSysCLASS(Env,A_C1C2);
C2.color = blue;
C2.name = 'C2';

%% Ups, this didn't do what we wanted, since CoSysCLASS needs an A_IC2 
%%  transformation.  Let's fix that
A_IC2 = A_IC1 * A_C1C2;
C2.A_IC = A_IC2;

%% Create a vector and show it in different coordinate systems
C2_v = [1;1;0];
v = VectorCLASS(Env, C2, C2_v);
v.color = black;
v.name = 'v';

I_v = A_IC1 * A_C1C2 * C2_v;
disp('I_v:');
disp(I_v);

v2 = VectorCLASS(Env, I, I_v);
v2.color = grey;
v2.name = 'v2';



