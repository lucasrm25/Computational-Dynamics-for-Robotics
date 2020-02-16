%% INITIALIZE:
clear all
close all
clc

% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();


% Define a triple pendulum
% Define the bodies:
ground             = LinkedRigidBodyDynamicsCLASS_v2(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 1;   % Since this is ground, m and I do not matter
ground.B_I_B       = eye(3)*0.0001;
ground.scale       = 0.5; % Defines how large the CoSys will be drawn
link1              = LinkedRigidBodyDynamicsCLASS_v2(env);
link1.bodyName     = 'Link 1';
link1.autoUpdate   = false;
link1.m_B          = 10;  % m and I just given for visualization
link1.B_I_B        = diag([0.0042,0.8354,0.8354]);
link1.scale        = 0.5; % Defines how large the CoSys will be drawn
link2              = LinkedRigidBodyDynamicsCLASS_v2(env);
link2.bodyName     = 'Link 2';
link2.autoUpdate   = false;
link2.m_B          = 10;  % m and I just given for visualization
link2.B_I_B        = diag([0.0042,0.8354,0.8354]);
link2.scale        = 0.5; % Defines how large the CoSys will be drawn
link3              = LinkedRigidBodyDynamicsCLASS_v2(env);
link3.bodyName     = 'Link 3';
link3.autoUpdate   = false;
link3.m_B          = 10;  % m and I just given for visualization
link3.B_I_B        = diag([0.0042,0.8354,0.8354]);
link3.scale        = 0.5; % Defines how large the CoSys will be drawn

% Define the joints and the interconnection of the bodies:
joint1 = LinkedRotationalJointCLASS_v2(env, ground, link1);
joint1.P_r_PDp    = [0;0;0];
joint1.A_PDp      = [0,1,0;-1,0,0;0,0,1];
joint1.S_r_SDs    = [-0.5;0;0];
joint1.A_SDs      = eye(3);
joint1.jointName  = 'Joint 1';
joint1.autoUpdate = false;
joint1.scale      = 0.2;

joint2 = LinkedRotationalJointCLASS_v2(env, link1, link2);
joint2.P_r_PDp    = [+0.5;0;0];
joint2.A_PDp      = eye(3);
joint2.S_r_SDs    = [-0.5;0;0];
joint2.A_SDs      = eye(3);
joint2.jointName  = 'Joint 2';
joint2.autoUpdate = false;
joint2.scale      = 0.2;

joint3 = LinkedRotationalJointCLASS_v2(env, link2, link3);
joint3.P_r_PDp    = [+0.5;0;0];
joint3.A_PDp      = eye(3);
joint3.S_r_SDs    = [-0.5;0;0];
joint3.A_SDs      = eye(3);
joint3.jointName  = 'Joint 3';
joint3.autoUpdate = false;
joint3.scale      = 0.2;

% Show the topology:
ground.recursiveOutput('');

% Define the joint angles:
joint1.q = +pi/6;
joint2.q = -pi/4;
joint3.q = +pi/3;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematics();
% Update the graphics
ground.recursiveGraphicsUpdate();

% Save final screen shot:
axis([-1,1,-3,3,-3.5,0.5]);
print(gcf,'-r600','-djpeg','Problem_30_Output.jpg'); 



%% Animation:
pause(3);
data = load('TestData_30_31_TriplePendulumSim.mat');

delta_t_show = 0.05;
next_t_output = data.t(1);

while next_t_output <  data.t(end)
    ind = find( data.t>=next_t_output, 1);
    
    % Update the joint angles:
    joint1.q = pi/2 + data.q1(ind);
    joint2.q = data.q2(ind);
    joint3.q = data.q3(ind);

    % Compute the forward kinematics starting with a root at position and
    % orientation zero:
    ground.recursiveForwardKinematics();
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow;
    
    next_t_output = next_t_output + delta_t_show;
end

