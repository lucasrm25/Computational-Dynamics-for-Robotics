function TestFile_37_ControllerEvaluation()
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

%% Define all parameters:
%
% Controller parameters:
P = 10; % [Nm/rad] 
D = 50; % [Nms/rad] 
% Gravity:
I_grav = [0;-9.81;0]; % [N/Kg]

%% Load trajectory:
res = load('TestData_37_ControllerEvaluation.mat');
t        = res.t;
q_des    = res.q_des;
d_q_des  = res.d_q_des;
dd_q_des = res.dd_q_des;
x_des    = res.x_des;
d_x_des  = res.d_x_des;

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
ground             = LinkedRigidBodyDynamicsMfgCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Kg] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
link1              = LinkedRigidBodyDynamicsMfgCLASS(env);
link1.bodyName     = 'Link 1';
link1.autoUpdate   = false;
link1.m_B   = 3; % [Kg]
link1.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link1.I_grav       = I_grav; % Define direction of gravity
link1.scale        = 0.5; % Defines how large the CoSys will be drawn
%
link2              = LinkedRigidBodyDynamicsMfgCLASS(env);
link2.bodyName     = 'Link 2';
link2.autoUpdate   = false;
link2.m_B   = 3; % [Kg]
link2.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link2.I_grav       = I_grav; % Define direction of gravity
link2.scale        = 0.5; % Defines how large the CoSys will be drawn
%
link3              = LinkedRigidBodyDynamicsMfgCLASS(env);
link3.bodyName     = 'Link 3';
link3.autoUpdate   = false;
link3.m_B   = 3; % [Kg]
link3.B_I_B = diag([0.2; 0.02; 0.2]); % [Kg m^2]
link3.I_grav       = I_grav; % Define direction of gravity
link3.scale        = 0.5; % Defines how large the CoSys will be drawn
%
% The end effector doesn't add any dynamics nor degrees of freedom, but
% by having it in the kinematic tree, we can easily compute the
% Cartesian position, velocity, and acceleration of the end-effector:  
endEffector = LinkedRigidBodyDynamicsMfgCLASS(env);
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

% Show the robot in the initial configuration
% Set all joint angles:
joint1.q = q_des(1, 1);
joint2.q = q_des(2, 1);
joint3.q = q_des(3, 1);
% Set all joint velocities:
joint1.qDot = d_q_des(1, 1);
joint2.qDot = d_q_des(2, 1);
joint3.qDot = d_q_des(3, 1);
% Set all joint accelerations:
joint1.qDDot = dd_q_des(1, 1);
joint2.qDDot = dd_q_des(2, 1);
joint3.qDDot = dd_q_des(3, 1);

% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematicsVeAcJa();
% Update the graphics
ground.recursiveGraphicsUpdate();
axis([-2,2,-2,2,-3.5,0.5])





% Run a forward dynamic simulation with different controllers:
% uncomment the desired controller type:
% controllerType = 'PD_only';
% controllerType = 'GravityComp';
controllerType = 'CompTorques';

%***** Simulation *****:
% Set initial conditions to match the beginning of the desired
% trajectory (q and d_q are stacked, as we convert this second order
% ODE to a first order ODE):  
y_0 = [q_des(:, 1); d_q_des(:, 1)];
% Simulate motion    
[~, y] = ode45(@ODE, t, y_0);

% Animate motion and record the cartesian position and velocity of the
% end-effector in inertial coordinates: 
I_rIE = zeros(3, length(t));
I_v_E = zeros(3, length(t));
for i = 1:length(t)
    % Set the joint variables to the values from the trajectory created
    % during simulation: 
    q_   = y(i,1:ground.nq);
    d_q_ = y(i,ground.nq+1:2*ground.nq);
    % Set joint angles:
    joint1.q = q_(1);
    joint2.q = q_(2);
    joint3.q = q_(3);
    % Set joint velocities:
    joint1.qDot = d_q_(1);
    joint2.qDot = d_q_(2);
    joint3.qDot = d_q_(3);
    % All joint accelerations are set to zero (this is not correct, but
    % we are only interested in position and velocity of the
    % end-effector):  
    joint1.qDDot = 0;
    joint2.qDDot = 0;
    joint3.qDDot = 0;
    % Compute forward kinematics using the recursive outward pass
    ground.recursiveForwardKinematicsVeAcJa();
    % Store the Cartesian position and velocity of the endeffector
    I_rIE(:, i) = endEffector.A_IB *endEffector.B_r_IB;
    I_v_E(:, i) = endEffector.A_IB *endEffector.B_v_B;
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow();
end
% plot the resulting trajectory in cartesian space:
plotFig = figure();
set(plotFig, 'name', 'Forward Dynamics');
hold on; grid on; box on
axis([0,2,-2.5,2.5]);
plot(t, [I_rIE; I_v_E],'-');
ax = gca(); ax.ColorOrderIndex = 1; % Reset the color cycling, so the next plot command uses the same colors:
plot(t, [x_des; d_x_des],':');
legend('x','z','y','dx','dz','dy');
print(gcf,'-r600','-djpeg','Output_37_ControllerEvaluation.jpg','-opengl');



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
    joint3.q = q_(3);
    % Set joint velocities:
    joint1.qDot = d_q_(1);
    joint2.qDot = d_q_(2);
    joint3.qDot = d_q_(3);
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    joint1.qDDot = 0;
    joint2.qDDot = 0;
    joint3.qDDot = 0;

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
    tau = controller(t_, q_, d_q_);
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

% This is the implementation of a controller.  It returns motor torques
% that act on the joints to make them follow the desired trajectory:
% Three different controllers are implemented:  
% (1) Using only PD-feedback on joint-level. 
% (2) Using PD-feedback on joint-level and gravity compensation. 
% (3) Using PD-feedback on joint-level and the method of computed
%     torques.       
function tau = controller(t_, q_, d_q_)
    % Use interpolation to get the values of the desired joint angles,
    % velocities, and accelerations at time 't_':
    q_des_    = interp1(t, q_des', t_)';
    d_q_des_  = interp1(t, d_q_des', t_)';
    dd_q_des_ = interp1(t, dd_q_des', t_)';
    % Compute the components of the EOM:
    % (The following is not strictly necessary, as we just computed M,
    % f, and g in the ODE function.  However, in order to seperate
    % controller and simulation, we repeat the step here as we would
    % have to do it in a physical implementation of the controller)   
    % 1) Set the current (measured) joint angles and velocities. Set
    % the accelerations to 0:
    % Set joint angles:
    joint1.q = q_(1);
    joint2.q = q_(2);
    joint3.q = q_(3);
    % Set joint velocities:
    joint1.qDot = d_q_(1);
    joint2.qDot = d_q_(2);
    joint3.qDot = d_q_(3);
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    joint1.qDDot = 0;
    joint2.qDDot = 0;
    joint3.qDDot = 0;
    % 2) Recursively update all Cartesian velocities,
    % BIAS-accelerations, and Jacobians: 
    ground.recursiveForwardKinematicsVeAcJa();
    % 3) Recursively compute the components of the equations of motion:
    [M, f, g] = ground.recursiveComputationOfMfg();

    % Pick one of three controllers
    switch controllerType 
        case 'PD_only'
            tau = P*(q_des_-q_) + D*(d_q_des_-d_q_);
        case 'GravityComp' 
            tau = -g + P*(q_des_-q_) + D*(d_q_des_-d_q_);
        case 'CompTorques'
            tau = M*dd_q_des_ - f - g + P*(q_des_-q_) + D*(d_q_des_-d_q_);
        otherwise
            tau = zeros(n_q, 1);
    end
end
end