% This script runs the kinematic simulation of a Passive Dynamic
% Walker with velocities, accelerations, and Jacobians. 
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

% Define the bodies:
ground    = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Normalized to leg mass] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
stanceLeg = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
stanceLeg.bodyName    = 'Stance Leg';
stanceLeg.autoUpdate  = false;
stanceLeg.m_B         = 1;   % [Normalized to leg mass]
stanceLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
stanceLeg.scale       = 0.5; % Defines how large the CoSys will be drawn
%
swingLeg  = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
swingLeg.bodyName    = 'Swing Leg';
swingLeg.autoUpdate  = false;
swingLeg.m_B         = 1;   % [Normalized to leg mass]
swingLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
swingLeg.scale       = 0.5; % Defines how large the CoSys will be drawn

% Define the joints and the interconnection of the bodies: 
rollingContact = LinkedRollingContactJointVeAcJaCLASS(env, ground, stanceLeg);
rollingContact.r = r;  % Set the radius of the rolling contact
rollingContact.P_r_PDp = [0;r;0];   % this has to be r above the ground (in the center of the rolling contact);
rollingContact.A_PDp   = eye(3);  % not rotated
rollingContact.S_r_SDs = [-dx;dy-l+r;0];  % in the center of the rolling contact;
rollingContact.A_SDs   = eye(3);  % initially not rotated
rollingContact.jointName = 'Rolling Contact';
rollingContact.autoUpdate = false;
rollingContact.scale      = 0.2;
%
hip = LinkedRotationalJointVeAcJaCLASS(env, stanceLeg, swingLeg);
hip.P_r_PDp = [-dx;dy;0];  % Since the COG is (+dx, -dy) displaced from the hip
hip.A_PDp   = eye(3);    % not rotated;
hip.S_r_SDs = [-dx;dy;0];  % the location is identical in both legs;
hip.A_SDs   = eye(3);    % not rotated;
hip.jointName = 'Hip Joint';
hip.autoUpdate = false;
hip.scale      = 0.2;

% Show the topology:
ground.recursiveOutput('');

% So we can compute Jacobians, we need to specify the total number of
% joints (this is stored in the ground, where the first Jacobian is
% generated) and which joint is located where in the jacobian (stored
% in the individual joints):
ground.nq = 2;
rollingContact.qIndex = 1;
hip.qIndex = 2;

% Show the passive dynamic walker in an example configuration
% Set all joint angles:
rollingContact.q = -30*pi/180;
hip.q            = +60*pi/180;
% Set all joint velocities:
rollingContact.qDot = -0.5;
hip.qDot            = +1;
% Set all joint accelerations:
rollingContact.qDDot = 0;
hip.qDDot            = 0;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematicsVeAcJa();
% Update the graphics
ground.recursiveGraphicsUpdate();
env.resetOutput();
view(90,0)
print(gcf,'-r600','-djpeg','Problem_33_Output.jpg');



%% Not part of the homework, but fun: Show an animation:
for i = 1:50
    % Set the joint variables to the values from the trajectory:
    rollingContact.q = -(i/50)*0.5;  % alpha sweeps from 0 to -0.5 rad
    hip.q            = +(i/50)*1;  % gamma sweeps from 0 to +1 rad
    % velocities are constant
    rollingContact.qDDot = -0.5;
    hip.qDDot            = +1;
    % accelerations are zero:
    rollingContact.qDDot = 0;
    hip.qDDot            = 0;
    % Compute forward kinematics using the recursive outward pass:
    ground.recursiveForwardKinematicsVeAcJa();
    % Update the graphics and visualize (part d):
    ground.recursiveGraphicsUpdate();
    pause(0.1)
end

