%% Prepare graphical output:
%
close all
% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();
set(env.fig, 'name', '3D Graphics Window');
hold on;


% Define a bipedal robot that has a floating base that connects it's main 
% body with the ground  


% Define the bodies:
ground = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
ground.bodyName     = 'Ground';
ground.autoUpdate   = false;
ground.m_B          = 0;   % [Normalized to total mass] (ground has no mass)
ground.scale        = 0.4; % Defines how large the CoSys will be drawn
%
mainBody = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
mainBody.bodyName   = 'MainBody';
mainBody.autoUpdate = false;
mainBody.m_B        = 0.7;   % [Normalized to total mass]
mainBody.B_I_B      = diag([0.004; 0.04; 0.04]);  % [Normalized to total mass and leg length]
mainBody.scale      = 0.5; % Defines how large the CoSys will be drawn
%
thighR = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
thighR.bodyName     = 'ThighR';
thighR.autoUpdate   = false;
thighR.m_B          = 0.1;   % [Normalized to total mass]
thighR.B_I_B        = diag([0.0002; 0.002; 0.002]);  % [Normalized to total mass and leg length]
thighR.scale        = 0.35; % Defines how large the CoSys will be drawn
%
thighL = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
thighL.bodyName     = 'ThighL';
thighL.autoUpdate   = false;
thighL.m_B          = 0.1;   % [Normalized to total mass]
thighL.B_I_B        = diag([0.0002; 0.002; 0.002]);  % [Normalized to total mass and leg length]
thighL.scale        = 0.35; % Defines how large the CoSys will be drawn
%
shankR = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
shankR.bodyName     = 'ShankR';
shankR.autoUpdate   = false;
shankR.m_B          = 0.05;   % [Normalized to total mass]
shankR.B_I_B        = diag([0.0001; 0.001; 0.001]);  % [Normalized to total mass and leg length]
shankR.scale        = 0.3; % Defines how large the CoSys will be drawn
%
shankL = LinkedRigidBodyDynamicsVeAcJaCLASS(env);
shankL.bodyName     = 'ShankL';
shankL.autoUpdate   = false;
shankL.m_B          = 0.05;   % [Normalized to total mass]
shankL.B_I_B        = diag([0.0001; 0.001; 0.001]);  % [Normalized to total mass and leg length]
shankL.scale        = 0.3; % Defines how large the CoSys will be drawn
%
% Define the joints:
virtual3DOF = LinkedVirtual3DOFJointVeAcJaCLASS(env, ground, mainBody);
virtual3DOF.jointName  = 'Virtual 3 DOF Joint';
virtual3DOF.P_r_PDp    = [0;0;0];
virtual3DOF.A_PDp      = eye(3);
virtual3DOF.S_r_SDs    = [0;0;0];
virtual3DOF.A_SDs      = eye(3);
virtual3DOF.autoUpdate = false;
virtual3DOF.scale      = 0.2;
%
hipL = LinkedRotationalJointVeAcJaCLASS(env, mainBody, thighL);
hipL.jointName  = 'Left Hip Joint';
hipL.P_r_PDp    = [0;0;+0.2];
hipL.A_PDp      = [0,1,0;-1,0,0;0,0,1];
hipL.S_r_SDs    = [-0.25;0;0];
hipL.A_SDs      = eye(3);
hipL.autoUpdate = false;
hipL.scale      = 0.2;
%
hipR = LinkedRotationalJointVeAcJaCLASS(env, mainBody, thighR);
hipR.jointName  ='Right Hip Joint';
hipR.P_r_PDp    = [0;0;-0.2];
hipR.A_PDp      = [0,1,0;-1,0,0;0,0,1];
hipR.S_r_SDs    = [-0.25;0;0];
hipR.A_SDs      = eye(3);
hipR.autoUpdate = false;
hipR.scale      = 0.2;
%
legExtensionL = LinkedTranslationalJointVeAcJaCLASS(env, thighL, shankL);
legExtensionL.jointName  = 'Left Leg Joint';
legExtensionL.P_r_PDp    = [0.25;0;0];
legExtensionL.A_PDp      = eye(3);
legExtensionL.S_r_SDs    = [-0.25;0;0];
legExtensionL.A_SDs      = eye(3);
legExtensionL.autoUpdate = false;
legExtensionL.scale      = 0.2;
%
legExtensionR = LinkedTranslationalJointVeAcJaCLASS(env, thighR, shankR);
legExtensionR.jointName  = 'Right Leg Joint';
legExtensionR.P_r_PDp    = [0.25;0;0];
legExtensionR.A_PDp      = eye(3);
legExtensionR.S_r_SDs    = [-0.25;0;0];
legExtensionR.A_SDs      = eye(3);
legExtensionR.autoUpdate = false;
legExtensionR.scale      = 0.2;

% So we can compute Jacobians, we need to specify the total number of
% joints (this is stored in the ground, where the first Jacobian is
% generated) and which joint is located where in the jacobian (stored in
% the individual joints):
ground.nq = 7;
virtual3DOF.qIndex = [1;2;3];
hipL.qIndex = 4;
hipR.qIndex = 5;
legExtensionL.qIndex = 6;
legExtensionR.qIndex = 7;
% This gives us the following order:
% [virtual3DOF_x; 
%  virtual3DOF_y; 
%  virtual3DOF_gamma; 
%  hipL_gamma; 
%  hipR_gamma; 
%  legExtensionL_deltaX;
%  legExtensionR_deltaX];

view(90,0)
axis([-1,1,-2,3,-0.5,2])

%% Run the simulation of a walking/running gait that is given as a number 
% of joint angles over time.
% Load the trajectories:
load TestData_34_WalkingData.mat
% load TestData_34_RunningData.mat

% Differentiate joint angles to obtain velocities ...
Ddata.xVec = [0, diff(data.xVec)./diff(data.tVec)];
Ddata.yVec = [0, diff(data.yVec)./diff(data.tVec)];
Ddata.phiVec = [0, diff(data.phiVec)./diff(data.tVec)];
Ddata.alphaLVec = [0, diff(data.alphaLVec)./diff(data.tVec)];
Ddata.alphaRVec = [0, diff(data.alphaRVec)./diff(data.tVec)];
Ddata.lLVec = [0, diff(data.lLVec)./diff(data.tVec)];
Ddata.lRVec = [0, diff(data.lRVec)./diff(data.tVec)];
% ...and accelerations:
DDdata.xVec = [0, diff(Ddata.xVec)./diff(data.tVec)];
DDdata.yVec = [0, diff(Ddata.yVec)./diff(data.tVec)];
DDdata.phiVec = [0, diff(Ddata.phiVec)./diff(data.tVec)];
DDdata.alphaLVec = [0, diff(Ddata.alphaLVec)./diff(data.tVec)];
DDdata.alphaRVec = [0, diff(Ddata.alphaRVec)./diff(data.tVec)];
DDdata.lLVec = [0, diff(Ddata.lLVec)./diff(data.tVec)];
DDdata.lRVec = [0, diff(Ddata.lRVec)./diff(data.tVec)];

% For each timestep:
for i = 1:length(data.tVec)
    % Set the joint variables to the values from the trajectory:
    virtual3DOF.q    = [data.xVec(i); data.yVec(i); data.phiVec(i)];
    hipL.q           = data.alphaLVec(i);
    hipR.q           = data.alphaRVec(i);
    legExtensionL.q  = data.lLVec(i);
    legExtensionR.q  = data.lRVec(i);
    virtual3DOF.qDot    = [Ddata.xVec(i); Ddata.yVec(i); Ddata.phiVec(i)];
    hipL.qDot           = Ddata.alphaLVec(i);
    hipR.qDot           = Ddata.alphaRVec(i);
    legExtensionL.qDot  = Ddata.lLVec(i);
    legExtensionR.qDot  = Ddata.lRVec(i);
    virtual3DOF.qDDot    = [DDdata.xVec(i); DDdata.yVec(i); DDdata.phiVec(i)];
    hipL.qDDot           = DDdata.alphaLVec(i);
    hipR.qDDot           = DDdata.alphaRVec(i);
    legExtensionL.qDDot  = DDdata.lLVec(i);
    legExtensionR.qDDot  = DDdata.lRVec(i);
    % Compute forward kinematics using the recursive outward pass:
    ground.recursiveForwardKinematicsVeAcJa();
    % Draw the system using a recursive outward pass:
    ground.recursiveGraphicsUpdate();
    pause(0.1)
    % capture the output at frame 90
    if i==90
        print(gcf,'-r600','-djpeg','Output_34_Biped.jpg');
    end
end
