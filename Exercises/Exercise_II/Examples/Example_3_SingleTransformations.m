% Script to demonstrate the use of the following components:
% EnvironmentCLASS, VectorCLASS, and CoSysCLASS 
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   10/12/2018
%   v21
%
clear all

% SET UP THE PATH
% This needs to be done only once:
% Click "Set Path" in the menu of the Matlab main window.
% Click "Add Folder..." and add the folder in which you want to store the 
%       class files (e.g., "...\CDR\Matlab\ClassFiles")
% Click "Save" (so you don't have to repeat this in the future)
% Click "Close"
% Done...
% Now copy every file that ends in ...CLASS into this folder.

% Define some RGB values:
red   = [1;0;0]; 
green = [0;1;0]; 
blue  = [0;0;1]; 
black = [0;0;0]; 
grey  = [0.3;0.3;0.3];
mag   = [1;0;1];
yel   = [1;1;0];

%% Create a 'physical' (graphical) environment:
Env = EnvironmentCLASS();

%% Coordinate system C with the same orientation as the graphics co-sys:
A_IC = eye(3);
C = CoSysCLASS(Env,A_IC);
C.color = green;
C.name = 'C';


%% Create a coordinate system that is rotated by 30 deg about the 3-axis:
gamma = 30*pi/180;
A_IB =[cos(gamma), -sin(gamma), 0; 
       sin(gamma),  cos(gamma), 0;
       0,           0,          1];
B = CoSysCLASS(Env,A_IB);
B.color = red;
B.name = 'B';

%% Define a vector in this coordinate system which has the following
% components:
B_v = [0.5;0.5;0];
v1 = VectorCLASS(Env, B, B_v);
v1.color = black;
v1.name = 'v1';

%% A good visual check is to see if the columns of A_CB are indeed the unit
% vectors of B expressed in C:
disp(A_IB)
%% similarly, the columns of the inverse transformation matrix A_BC = A_CB'
% should be the unit vectors of C expressed in B.  We rotate the 'world',
% so we can check this more easily.  Also, it shows that these are only
% relative descriptions.  No coordinate system is more important than the
% other 
disp(A_IB')
Env.toggleAxis;
camup([0,-0.5,0.87])
%% And rotate back:
camup([0, 0, 1])
Env.toggleAxis;

%% A Vector v2, that has the same components as v1, but is defined in
% coordinate system C, will point in a different direction:
C_v = B_v;
v2 = VectorCLASS(Env, C, C_v);
v2.color = grey;
v2.name = 'v2';

%% But if we convert the B-components into C-components, the two vectors will be identical:
C_v = A_IC'*A_IB*B_v;
v2.setCoords(C, C_v);














