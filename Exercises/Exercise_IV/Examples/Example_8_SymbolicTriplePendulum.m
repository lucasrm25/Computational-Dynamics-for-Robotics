%% INITIALIZE:
clear all
close all
clc

% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();

% Define symbolic variables for geometrical parameters and joint angles:
syms l real


% Define a triple pendulum
% Define the bodies:
ground             = LinkedRigidBodyDynamicsCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 1;   % Since this is ground, m and I do not matter
ground.B_I_B       = eye(3)*0.0001;
ground.scale       = 0.5; % Defines how large the CoSys will be drawn
link1              = LinkedRigidBodyDynamicsCLASS(env);
link1.bodyName     = 'Link 1';
link1.autoUpdate   = false;
link1.m_B          = 10;  % m and I just given for visualization
link1.B_I_B        = diag([0.0042,0.8354,0.8354]);
link1.scale        = l/2; % Defines how large the CoSys will be drawn
link2              = LinkedRigidBodyDynamicsCLASS(env);
link2.bodyName     = 'Link 2';
link2.autoUpdate   = false;
link2.m_B          = 10;  % m and I just given for visualization
link2.B_I_B        = diag([0.0042,0.8354,0.8354]);
link2.scale        = l/2; % Defines how large the CoSys will be drawn
link3              = LinkedRigidBodyDynamicsCLASS(env);
link3.bodyName     = 'Link 3';
link3.autoUpdate   = false;
link3.m_B          = 10;  % m and I just given for visualization
link3.B_I_B        = diag([0.0042,0.8354,0.8354]);
link3.scale        = l/2; % Defines how large the CoSys will be drawn

% Define the joints and the interconnection of the bodies:
joint1 = LinkedRotationalJointCLASS(env, ground, link1);
joint1.P_r_PDp    = [0;0;0];
joint1.A_PDp      = [0,1,0;-1,0,0;0,0,1];
joint1.S_r_SDs    = [-l/2;0;0];
joint1.A_SDs      = eye(3);
joint1.jointName  = 'Joint 1';
joint1.autoUpdate = false;
joint1.scale      = 0.2;

joint2 = LinkedRotationalJointCLASS(env, link1, link2);
joint2.P_r_PDp    = [+l/2;0;0];
joint2.A_PDp      = eye(3);
joint2.S_r_SDs    = [-l/2;0;0];
joint2.A_SDs      = eye(3);
joint2.jointName  = 'Joint 2';
joint2.autoUpdate = false;
joint2.scale      = 0.2;

joint3 = LinkedRotationalJointCLASS(env, link2, link3);
joint3.P_r_PDp    = [+l/2;0;0];
joint3.A_PDp      = eye(3);
joint3.S_r_SDs    = [-l/2;0;0];
joint3.A_SDs      = eye(3);
joint3.jointName  = 'Joint 3';
joint3.autoUpdate = false;
joint3.scale      = 0.2;

% Define the joint angles as symbolic values:
syms q1 q2 q3 real
joint1.q = q1;
joint2.q = q2;
joint3.q = q3;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematics();
I_r_IB3 = link3.A_IB*link3.B_r_IB;
I_r_IB3 = simplify(I_r_IB3);
disp(I_r_IB3)




