function TestFile_90_DoublePendulum()
% This function runs the multi body simulation of a three segment
% robot.  
%
% The robot's trajectory is defined as a linear motion (with a
% trapezoidal velocity profile) in end-effector space.  For us, this
% desired trajectory is given by generalized joint angles q_des,
% generalized velocities d_q_des, and  generalized accelerations
% dd_q_des.  
%
% The robot is using a feedback controller to follow these pre-defined
% trajectories in joint-space.  Three different controllers are
% implemented:   
% (1) Using only PD-feedback on joint-level. 
% (2) Using PD-feedback on joint-level and gravity compensation. 
% (3) Using PD-feedback on joint-level and the method of computed
%     torques.      


% Gravity:
I_grav = [0;-9.81;0]; % [N/Kg]


%% Prepare graphical output:
%
close all
% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();
set(env.fig, 'name', '3D Graphics Window');
hold on;


%% Define the multi body system that represents the robot:

% Define the bodies:
ground             = LinkedRigidBodyDynamicsMfgCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Kg] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
link1              = LinkedRigidBodyDynamicsMfgCLASS(env);
link1.bodyName     = 'Link 1';
link1.autoUpdate   = false;
link1.m_B          = 4; % [Kg]
link1.B_I_B        = diag([0.0042,0.8354,0.8354]); % [Kg m^2]
link1.I_grav       = I_grav; % Define direction of gravity
link1.scale        = 0.5; % Defines how large the CoSys will be drawn
%
link2              = LinkedRigidBodyDynamicsMfgCLASS(env);
link2.bodyName     = 'Link 2';
link2.autoUpdate   = false;
link2.m_B          = 4; % [Kg]
link2.B_I_B        = diag([0.0042,0.8354,0.8354]); % [Kg m^2]
link2.I_grav       = I_grav; % Define direction of gravity
link2.scale        = 0.5; % Defines how large the CoSys will be drawn


% Define the joints:
% Rotational joints 1-3
joint1 = LinkedRotationalJointVeAcJaCLASS(env, ground, link1);
joint1.P_r_PDp    = [0;0;0];
joint1.A_PDp      = eye(3);
joint1.S_r_SDs    = [-0.5;0;0];
joint1.A_SDs      = eye(3);
joint1.jointName  = 'Joint 1';
joint1.autoUpdate = false;
joint1.scale      = 0.3;
%
joint2 = LinkedRotationalJointVeAcJaCLASS(env, link1, link2);
joint2.P_r_PDp    = [0.5;0;0];
joint2.A_PDp      = eye(3);
joint2.S_r_SDs    = [-0.5;0;0];
joint2.A_SDs      = eye(3);
joint2.jointName  = 'Joint 2';
joint2.autoUpdate = false;
joint2.scale      = 0.3;
 

% Show the topology:
ground.recursiveOutput('');

% So we can compute Jacobians, we need to specify the total number of
% joints (this is stored in the ground, where the first Jacobian is
% generated) and which joint is located where in the jacobian (stored
% in the individual joints):
ground.nq          = 2;
joint1.qIndex      = 1;
joint2.qIndex      = 2;

% Show the robot in the initial configuration
% Set all joint angles:
joint1.q = 30*pi/180;
joint2.q = 10*pi/180;
% Set all joint velocities:
joint1.qDot = -2*pi/180;
joint2.qDot = -7*pi/180;
% Set all joint accelerations:
joint1.qDDot = 1*pi/180;
joint2.qDDot = -2*pi/180;

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematicsVeAcJa();

disp('\n B_a_B:')
link2.B_a_B
disp('\n B_v_B:')
link2.B_v_B
disp('\n B_omegaDot_B:')
link2.B_omegaDot_B
disp('\n B_omega_B:')
link2.B_omega_B
disp('\n B_J_R:')
link2.B_J_R
disp('\n B_J_S:')
link2.B_J_S


joint1.qDDot = 0;
joint2.qDDot = 0;
ground.recursiveForwardKinematicsVeAcJa();
% Compute the components of the equations of motion:
[M, f, g] = ground.recursiveComputationOfMfg();

M
f
g



% Update the graphics
ground.recursiveGraphicsUpdate();
axis([-2,2,-2,2,-2,2])




%***** Simulation *****:
% Set initial conditions to match the beginning of the desired
% trajectory (q and d_q are stacked, as we convert this second order
% ODE to a first order ODE):  
y_0 = [joint1.q; joint2.q; joint1.qDot; joint2.qDot];
t = [0,10];

% Simulate motion    
[t, y] = ode45(@ODE, t, y_0);


figure; hold on; grid on;
plot(t,rad2deg(y(:,1)))
plot(t,rad2deg(y(:,2)))



% Animate motion
for i = 1:length(t)
    % Set the joint variables to the values from the trajectory created
    % during simulation: 
    q_   = y(i,1:ground.nq);
    d_q_ = y(i,ground.nq+1:2*ground.nq);
    % Set joint angles:
    joint1.q = q_(1);
    joint2.q = q_(2);
    % Set joint velocities:
    joint1.qDot = d_q_(1);
    joint2.qDot = d_q_(2);
    % All joint accelerations are set to zero (this is not correct, but
    % we are only interested in position and velocity of the
    % end-effector):  
    joint1.qDDot = 0;
    joint2.qDDot = 0;
    % Compute forward kinematics using the recursive outward pass
    ground.recursiveForwardKinematicsVeAcJa();
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow();
end


%% Functions used in the integration.  
% This includes the right hand sides of the ODDEs for unconstrained and
% constrained motion, event handling, and control  
%
% The right hand side of the ordinary differential equation for the
% unconstrained system. 
function d_y_ = ODE(t_, y_)
    
    % Extract the generalized coordinates q and velocities d_q.
    q_   = y_(1:ground.nq);
    d_q_ = y_(ground.nq+1:end);
    
    % Set joint angles:
    joint1.q = q_(1);
    joint2.q = q_(2);
    
    % Set joint velocities:
    joint1.qDot = d_q_(1);
    joint2.qDot = d_q_(2);
    
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    joint1.qDDot = 0;
    joint2.qDDot = 0;

    % Recursively compute the Jacobians, positions, velocities,
    % and bias-accelerations (by setting q_ddot to 0).  They are stored
    % in the individual Body-objects that are then access from the
    % subsequent routines.
    % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
    % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
    ground.recursiveForwardKinematicsVeAcJa();
    % Compute the components of the equations of motion:
    [M, f, g] = ground.recursiveComputationOfMfg();
    % Call the controller function to get the actuator torques that are
    % supplied to the motor: 
    tau = 0; 
    % Solve for the accelerations:
    dd_q = M\(f + g + tau);
    % Prepare the output variable d_y:
    d_y_ = zeros(2*ground.nq,1);
    % Compute the derivative of the positions from the velocities.
    d_y_(1:ground.nq)     = d_q_;
    % The derivative of the velocities are the generalized
    % accelerations: 
    d_y_(ground.nq+1:end) = dd_q;
end


end