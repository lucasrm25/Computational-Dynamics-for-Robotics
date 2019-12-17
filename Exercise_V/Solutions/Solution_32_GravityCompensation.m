% This srkipt tests gravity compensation for the multi body simulation of a
% three segment robot.  
clear all
clc

%% Define all parameters:
%
% Gravity:
I_grav = [0;-9.81;0]; % [N/Kg]

%% Load trajectory:
res = load('TestData_32_RobotTrajectory.mat');
t        = res.t;
q_des    = res.q_des;
    
%% Prepare graphical output:
%
close all
% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();
set(env.fig, 'name', '3D Graphics Window');
hold on;


%% Define the multi body system that represents the robot:
%
% Define the bodies:
ground             = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Kg] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
link1              = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
link1.bodyName     = 'Link 1';
link1.autoUpdate   = false;
link1.m_B   = 3; % [Kg]
link1.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link1.scale        = 0.5; % Defines how large the CoSys will be drawn
%
link2              = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
link2.bodyName     = 'Link 2';
link2.autoUpdate   = false;
link2.m_B   = 3; % [Kg]
link2.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link2.scale        = 0.5; % Defines how large the CoSys will be drawn
%
link3              = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
link3.bodyName     = 'Link 3';
link3.autoUpdate   = false;
link3.m_B   = 3; % [Kg]
link3.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link3.scale        = 0.5; % Defines how large the CoSys will be drawn
%
% The end effector doesn't add any dynamics nor degrees of freedom, but
% by having it in the kinematic tree, we can easily compute the
% Cartesian position, velocity, and acceleration of the end-effector:  
endEffector = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
endEffector.bodyName     = 'End Effector';
endEffector.autoUpdate   = false;
endEffector.m_B   = 0; % [Kg] (end-effector has no mass)
endEffector.scale = 0.4;
%
% Define the joints:
% Rotational joints 1-3
joint1 = LinkedRotationalJointVeAcJaCLASS(env, ground, link1);
joint1.P_r_PDp    = [0;0;0];
joint1.A_PDp      = eye(3);
joint1.S_r_SDs    = [0;+0.5;0];
joint1.A_SDs      = eye(3);
joint1.jointName  = 'Joint 1';
joint1.autoUpdate = false;
joint1.scale      = 0.3;
%
joint2 = LinkedRotationalJointVeAcJaCLASS(env, link1, link2);
joint2.P_r_PDp     = [0;-0.5;0];
joint2.A_PDp      = [0,0,+1;0,1,0;-1,0,0];
joint2.S_r_SDs    = [0;+0.5;0];
joint2.A_SDs      = eye(3);
joint2.jointName  = 'Joint 2';
joint2.autoUpdate = false;
joint2.scale      = 0.3;
%
joint3 = LinkedRotationalJointVeAcJaCLASS(env, link2, link3);
joint3.P_r_PDp    = [0;-0.5;0];
joint3.A_PDp      = [0,0,-1;0,1,0;+1,0,0];
joint3.S_r_SDs    = [0;+0.5;0];
joint3.A_SDs      = eye(3);
joint3.jointName  = 'Joint 3';
joint3.autoUpdate = false;
joint3.scale      = 0.3;
%
% This '0 degree of freedom' joint connects the end-effector to link3.
% It is only used to compute the position of the end effector: 
endEffectorConnection = LinkedGenericJointVeAcJaCLASS(env, link3, endEffector);
endEffectorConnection.P_r_PDp     = [0;-0.5;0];
endEffectorConnection.A_PDp       = eye(3);
endEffectorConnection.S_r_SDs     = [0;0;0];
endEffectorConnection.A_SDs       = eye(3);
endEffectorConnection.jointName   = 'End Effector Connection';
endEffectorConnection.autoUpdate  = false;
endEffectorConnection.scale       = 0.2;

% Show the topology:
ground.recursiveOutput('');

% So we can compute Jacobians, we need to specify the total number of
% joints (this is stored in the ground, where the first Jacobian is
% generated) and which joint is located where in the jacobian (stored
% in the individual joints):
ground.nq          = 3;
joint1.qIndex      = 1;
joint2.qIndex      = 2;
joint3.qIndex      = 3;

% Show the robot in the 'zero' configuration
% Set all joint angles to zero:
joint1.q = 0;
joint2.q = 0;
joint3.q = 0;
% Set all joint velocities to zero:
joint1.qDot = 0;
joint2.qDot = 0;
joint3.qDot = 0;
% Set all joint accelerations to zero:
joint1.qDDot = 0;
joint2.qDDot = 0;
joint3.qDDot = 0;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematicsVeAcJa();
% Update the graphics
ground.recursiveGraphicsUpdate();
axis([-2,2,-2,2,-3.5,0.5])


    
% Prepare vectors for results:
tau = zeros(3,length(t));
% Loop through all time-steps:
for i = 1:length(t)
    % Update the joint angles:
    joint1.q = q_des(1,i);
    joint2.q = q_des(2,i);
    joint3.q = q_des(3,i);
    % Compute the forward kinematics starting with a root at position and
    % orientation zero:
    ground.recursiveForwardKinematicsVeAcJa();
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow();
    
    % To get the joint torques for gravity compensation, we need to sum the
    % gravitational forces over all bodies:
    bodies = {link1, link2, link3};
    % Start with graviational torques of 0:
    g = zeros(3, 1);
    % Compose the equations of motion.  We do this numerically in each
    % integration step.  (This could also be done analytically in
    % advance and then just evaluated with the current values for q and
    % q_do) 
    for i_ = 1:length(bodies) % Iterate over all DYNAMICALLY CONTRIBUTING bodies in the tree
        B = bodies{i_}; % for each body:
        % Add the contribution of the gravitational force acting on this
        % body:
        g = g + B.B_J_S' * B.A_IB' * I_grav * B.m_B + ...
                B.B_J_R' * B.A_IB' * [0; 0; 0] ;
        % It's important that all F_A are expressed in B-coordinates.
        % They must hence be transformed from the inertial frame:
    end
    % Our gravity compensating torque must point in the opposite direction:
    tau(:,i) = -g;
end
% Create a figure for the output:
plotFig = figure();
set(plotFig, 'name', 'Gravity Compensating Torques');
hold on; grid on; box on
plot(t, tau,'-');
print(gcf,'-r600','-djpeg','Problem_32_Output.jpg','opengl'); 
    
