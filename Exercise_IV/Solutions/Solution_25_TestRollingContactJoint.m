% This skript runs the kinematic simulation of a Passive Dynamic
% Walker. 
clear all
clc

%% Define all parameters:
%
% Kinematic parameters:
r = 0.5;    % [Normalized to leg length]
l = 1;      % [Normalized to leg length]
dx = 0.05;  % [Normalized to leg length]
dy = 0.7;   % [Normalized to leg length]

%% Prepare graphical output:
%
close all
% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();
set(env.fig, 'name', '3D Graphics Window');
hold on;

% (part a)
% Define the bodies:
ground    = LinkedRigidBodyDynamicsCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Normalized to leg mass] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
stanceLeg = LinkedRigidBodyDynamicsCLASS(env);
stanceLeg.bodyName    = 'Stance Leg';
stanceLeg.autoUpdate  = false;
stanceLeg.m_B         = 1;   % [Normalized to leg mass]
stanceLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
stanceLeg.scale       = 0.5; % Defines how large the CoSys will be drawn
%
swingLeg  = LinkedRigidBodyDynamicsCLASS(env);
swingLeg.bodyName    = 'Swing Leg';
swingLeg.autoUpdate  = false;
swingLeg.m_B         = 1;   % [Normalized to leg mass]
swingLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
swingLeg.scale       = 0.5; % Defines how large the CoSys will be drawn

% (part b)
% Define the joints and the interconnection of the bodies: 
rollingContact = LinkedRollingContactJointCLASS(env, ground, stanceLeg);
rollingContact.r = r;  % Set the radius of the rolling contact
rollingContact.P_r_PDp = [0;r;0];   % this has to be r above the ground (in the center of the rolling contact);
rollingContact.A_PDp   = eye(3);  % not rotated
rollingContact.S_r_SDs = [-dx;dy-l+r;0];  % in the center of the rolling contact;
rollingContact.A_SDs   = eye(3);  % initially not rotated
rollingContact.jointName = 'Rolling Contact';
rollingContact.autoUpdate = false;
rollingContact.scale      = 0.2;
%
hip = LinkedRotationalJointCLASS(env, stanceLeg, swingLeg);
hip.P_r_PDp = [-dx;dy;0];  % Since the COG is (+dx, -dy) displaced from the hip
hip.A_PDp   = eye(3);    % not rotated;
hip.S_r_SDs = [-dx;dy;0];  % the location is identical in both legs;
hip.A_SDs   = eye(3);    % not rotated;
hip.jointName = 'Hip Joint';
hip.autoUpdate = false;
hip.scale      = 0.2;

% Show the topology:
ground.recursiveOutput('');

% (part c)
% Show the passive dynamic walker in an example configuration
% Set all joint angles:
rollingContact.q = -30*pi/180;
hip.q            = +60*pi/180;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematics();
% Display the position and orientation of the three bodies (angles in deg):
posStance = stanceLeg.A_IB*stanceLeg.B_r_IB;
posSwing  = swingLeg.A_IB*swingLeg.B_r_IB;
x = [posStance(1);
     posStance(2);
     acos(stanceLeg.A_IB(1,1))*180/pi;
     posSwing(1);
     posSwing(2);
     acos(swingLeg.A_IB(1,1))*180/pi];
disp(x);

% (part d)
% Update the graphics
ground.recursiveGraphicsUpdate();
env.resetOutput();
view(90,0)
print(gcf,'-r600','-djpeg','Problem_25_Output.jpg');



%% Not part of the homework, but fun: Show an animation:
for i = 1:50
    % Set the joint variables to the values from the trajectory:
    rollingContact.q = -(i/50)*0.5;  % alpha sweeps from 0 to -0.5 rad
    hip.q            = +(i/50)*1;  % gamma sweeps from 0 to +1 rad
    % Compute forward kinematics using the recursive outward pass:
    ground.recursiveForwardKinematics();
    % Update the graphics and visualize (part d):
    ground.recursiveGraphicsUpdate();
    pause(0.1)
end

%% Compute f(q) analytically (part e):
syms alpha gamma l r dx dy real

% Define parameters analytically
rollingContact.P_r_PDp = [0;r;0];  % this has to be r above the ground;
rollingContact.A_PDp   = sym(eye(3));  % not rotated
rollingContact.S_r_SDs = [-dx;dy-l+r;0];  % at the origin of the ground;
rollingContact.A_SDs    = sym(eye(3));  % initially not rotated
rollingContact.r = r;  % Set the radius of the rolling contact
hip.P_r_PDp = [-dx;dy;0];  % this has to be R above the ground;
hip.A_PDp   = sym(eye(3));  % at the origin of the ground;
hip.S_r_SDs = [-dx;dy;0];  % at the origin of the ground;
hip.A_SDs   = sym(eye(3));  % at the origin of the ground;

% Define joint angles analytically
rollingContact.q = alpha;
hip.q            = gamma;
q = [alpha, gamma];

% Compute kinematics:
ground.recursiveForwardKinematics(sym([0;0;0]),sym(eye(3)));
posStance = stanceLeg.A_IB*stanceLeg.B_r_IB;
posSwing  = swingLeg.A_IB*swingLeg.B_r_IB;
x = [posStance(1);
     posStance(2);
     acos(stanceLeg.A_IB(1,1));
     posSwing(1);
     posSwing(2);
     acos(swingLeg.A_IB(1,1))];
x = simplify(expand(x));
disp(x)

% Compute the Jacobian (part f)
J_f = jacobian(x,q);
J_f = simplify(expand(J_f));
disp(J_f)

