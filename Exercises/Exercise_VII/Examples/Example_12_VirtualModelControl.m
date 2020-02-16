function Example_12_VirtualModelControl()
% This function runs the multibody simulation of a bipedal walking
% system that is in double-stance (i.e., both feet are on the ground)

%% Define all parameters:
%
% Gravity:
% Gravity is pointing straigt down.
I_grav = [0;-1;0]; % [Normalized to g]

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
ground              = LinkedRigidBodyDynamicsMfgCLASS(env);
ground.bodyName     = 'Ground';
ground.autoUpdate   = false;
ground.m_B          = 0;   % [Normalized to total mass] (ground has no mass)
ground.scale        = 0.4; % Defines how large the CoSys will be drawn
%
mainBody            = LinkedRigidBodyDynamicsMfgCLASS(env);
mainBody.bodyName   = 'MainBody';
mainBody.autoUpdate = false;
mainBody.m_B        = 0.7;   % [Normalized to total mass]
mainBody.B_I_B      = diag([0.004; 0.04; 0.04]);  % [Normalized to total mass and leg length]
mainBody.I_grav     = I_grav; % Define direction of gravity
mainBody.scale      = 0.5; % Defines how large the CoSys will be drawn
%
thighR              = LinkedRigidBodyDynamicsMfgCLASS(env);
thighR.bodyName     = 'ThighR';
thighR.autoUpdate   = false;
thighR.m_B          = 0.1;   % [Normalized to total mass]
thighR.B_I_B        = diag([0.0002; 0.002; 0.002]);  % [Normalized to total mass and leg length]
thighR.I_grav       = I_grav; % Define direction of gravity
thighR.scale        = 0.35; % Defines how large the CoSys will be drawn
%
thighL              = LinkedRigidBodyDynamicsMfgCLASS(env);
thighL.bodyName     = 'ThighL';
thighL.autoUpdate   = false;
thighL.m_B          = 0.1;   % [Normalized to total mass]
thighL.B_I_B        = diag([0.0002; 0.002; 0.002]);  % [Normalized to total mass and leg length]
thighL.I_grav       = I_grav; % Define direction of gravity
thighL.scale        = 0.35; % Defines how large the CoSys will be drawn
%
shankR              = LinkedRigidBodyDynamicsMfgCLASS(env);
shankR.bodyName     = 'ShankR';
shankR.autoUpdate   = false;
shankR.m_B          = 0.05;   % [Normalized to total mass]
shankR.B_I_B        = diag([0.0001; 0.001; 0.001]);  % [Normalized to total mass and leg length]
shankR.I_grav       = I_grav; % Define direction of gravity
shankR.scale        = 0.3; % Defines how large the CoSys will be drawn
%
shankL              = LinkedRigidBodyDynamicsMfgCLASS(env);
shankL.bodyName     = 'ShankL';
shankL.autoUpdate   = false;
shankL.m_B          = 0.05;   % [Normalized to total mass]
shankL.B_I_B        = diag([0.0001; 0.001; 0.001]);  % [Normalized to total mass and leg length]
shankL.I_grav       = I_grav; % Define direction of gravity
shankL.scale        = 0.3; % Defines how large the CoSys will be drawn
%
footR = LinkedRigidBodyDynamicsMfgCLASS(env);
footR.bodyName    = 'FootR';
footR.autoUpdate  = false;
footR.m_B         = 0;   % [Normalized to total mass] (foot has no mass)
footR.scale       = 0.25; % Defines how large the CoSys will be drawn
%
footL = LinkedRigidBodyDynamicsMfgCLASS(env);
footL.bodyName    = 'FootL';
footL.autoUpdate  = false;
footL.m_B         = 0;   % [Normalized to total mass] (foot has no mass)
footL.scale       = 0.25; % Defines how large the CoSys will be drawn
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
%
% These '0 degree of freedom' joints connect the feet to the shanks.
% they are only used to compute the constraint jacobian:
footConnectionR = LinkedGenericJointVeAcJaCLASS(env, shankR, footR);
footConnectionR.jointName  = 'Right Foot Connector';
footConnectionR.P_r_PDp = [0.25;0;0];
footConnectionR.A_PDp   = eye(3);
footConnectionR.S_r_SDs = [0;0;0];
footConnectionR.A_SDs   = eye(3);
footConnectionR.scale   = 0.2;
%
footConnectionL = LinkedGenericJointVeAcJaCLASS(env, shankL, footL);
footConnectionL.jointName  = 'Left Foot Connector';
footConnectionL.P_r_PDp = [0.25;0;0];
footConnectionL.A_PDp   = eye(3);
footConnectionL.S_r_SDs = [0;0;0];
footConnectionL.A_SDs   = eye(3);
footConnectionL.scale   = 0.2;

% Show the topology:
ground.recursiveOutput('');

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
        
% Define initial conditions:
[q_0, q_dot_0] = GetInitialConditions();

% Show the robot in the initial configuration
% Set all joint angles:
virtual3DOF.q    = q_0(1:3);
hipL.q           = q_0(4);
hipR.q           = q_0(5);
legExtensionL.q  = q_0(6);
legExtensionR.q  = q_0(7);
% Set all joint velocities:
virtual3DOF.qDot    = q_dot_0(1:3);
hipL.qDot           = q_dot_0(4);
hipR.qDot           = q_dot_0(5);
legExtensionL.qDot  = q_dot_0(6);
legExtensionR.qDot  = q_dot_0(7);
% Set all joint accelerations to 0:
virtual3DOF.qDDot    = [0;0;0];
hipL.qDDot           = 0;
hipR.qDDot           = 0;
legExtensionL.qDDot  = 0;
legExtensionR.qDDot  = 0;
     
% Show initial condition (without computing accelerations):
% Compute the forward kinematics starting with a root at position and
% orientation zero:
ground.recursiveForwardKinematicsVeAcJa();
% Update the graphics
ground.recursiveGraphicsUpdate();
view(90,0);
axis([-1,1,0,2,-0.5,1.5])

%***** Simulation *****:
t_end = 5;      % [normalized units]
delta_t = 0.05; % [normalized units]
% Set initial conditions to match the beginning of the desired
% trajectory (q and d_q are stacked, as we convert this second order
% ODE to a first order ODE):  
y_0 = [q_0; q_dot_0];
% Simulate motion    
[t, y] = ode45(@ODE, 0:delta_t:t_end, y_0);
    
% Animate motion using our framework and record the position of the main
% body::
pos = zeros(3, length(t));
for i = 1:length(t)
    % Set the joint variables to the values from the trajectory created
    % during simulation: 
    q_   = y(i,1:ground.nq);
    d_q_ = y(i,ground.nq+1:2*ground.nq);
    % Set joint angles:
    virtual3DOF.q    = q_(1:3);
    hipL.q           = q_(4);
    hipR.q           = q_(5);
    legExtensionL.q  = q_(6);
    legExtensionR.q  = q_(7);
    % Set joint velocities:
    virtual3DOF.qDot    = d_q_(1:3);
    hipL.qDot           = d_q_(4);
    hipR.qDot           = d_q_(5);
    legExtensionL.qDot  = d_q_(6);
    legExtensionR.qDot  = d_q_(7);
    % All joint accelerations are set to zero (this is not correct, but
    % we are only interested in positions and velocities):
    virtual3DOF.qDDot    = [0;0;0];
    hipL.qDDot           = 0;
    hipR.qDDot           = 0;
    legExtensionL.qDDot  = 0;
    legExtensionR.qDDot  = 0;
    % Compute forward kinematics using the recursive outward pass
    ground.recursiveForwardKinematicsVeAcJa();
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow();
    pos(:,i) = mainBody.A_IB *mainBody.B_r_IB;
end
figure(2)
plot(t, pos);
grid on
box on
axis([0,5,-2,2]);

function [q_0, q_dot_0] = GetInitialConditions()
    % This is were we would have to solve the explicit constraint
    % g(q) = 0
    % The following values will do the job, as well:
    q_0      = [1; cos(pi/6); 0; 0; pi/6; cos(pi/6)-1; 0];
    q_dot_0  = [0; 0; 0; 0; 0; 0; 0];
end

function y_dot = ODE(t_, y_)
    % Extract the generalized coordinates q and velocities d_q.
    q_   = y_(1:ground.nq);
    d_q_ = y_(ground.nq+1:end);
    % Set joint angles:
    virtual3DOF.q    = q_(1:3);
    hipL.q           = q_(4);
    hipR.q           = q_(5);
    legExtensionL.q  = q_(6);
    legExtensionR.q  = q_(7);
    % Set joint velocities:
    virtual3DOF.qDot    = d_q_(1:3);
    hipL.qDot           = d_q_(4);
    hipR.qDot           = d_q_(5);
    legExtensionL.qDot  = d_q_(6);
    legExtensionR.qDot  = d_q_(7);
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    virtual3DOF.qDDot    = [0;0;0];
    hipL.qDDot           = 0;
    hipR.qDDot           = 0;
    legExtensionL.qDDot  = 0;
    legExtensionR.qDDot  = 0;
    
    % Recursively compute the Jacobians, positions, velocities,
    % and bias-accelerations (by setting q_ddot to 0).  They are stored
    % in the individual Body-objects that are then access from the
    % subsequent routines.
    % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
    % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
    ground.recursiveForwardKinematicsVeAcJa();
    % Compute the components of the equations of motion:
    [M, f, g] = ground.recursiveComputationOfMfg();
    % Virtual Model Control:
    tau = controller(q_, d_q_);
    % Get the constraint Jacobian:
    [J_lambda, sigma_lambda] = GetConstraintTerms();
    % Set up the equation A*x = b for the constrained system:
    if isempty(J_lambda) % no constraints:
        A = M;
        b = f + g + tau;
    else % with constraints:
        n_lambda = size(J_lambda,1);
        A = [M,       -J_lambda';
             J_lambda, zeros(n_lambda)];
        b = [f + g + tau;
             -sigma_lambda];
    end
    % Solve for the accelerations and constraint forces:
    x = A\b;
    % Extract accelerations:
    q_ddot = x(1:ground.nq);
    % Prepare the output variable y_dot:
    y_dot = zeros(2*ground.nq,1);
    % Compute the derivative of the positions from the velocities.  For
    % holonomic systems this is simply mapping q_dot to the derivative
    % of q.  Again, this might be different for non-holonomic systems,
    % in which different variables are used for position and velocity:
    y_dot(1:ground.nq)     = d_q_;
    % The derivative of the velocities are always the generalized
    % accelerations, both for holonomic systems and non-holonomic
    % systems:
    y_dot(ground.nq+1:end) = q_ddot;
end


        
function [J_lambda, sigma_lambda] = GetConstraintTerms()
    % Since we're simulating the ground contact as a pure rotation
    % about a point-foot, we can simply use the first two rows of the
    % translational jacobian:
    J_lambda = [footR.A_IB(1:2,1:2) * footR.B_J_S(1:2,:);
                footL.A_IB(1:2,1:2) * footL.B_J_S(1:2,:)];
    sigma_lambda = [footR.A_IB(1:2,1:2) * footR.B_a_B(1:2);
                        footL.A_IB(1:2,1:2) * footL.B_a_B(1:2)];
    % For Baumgarte-stabilization, add the corrective terms to
    % sigma_lambda here.  We only correct drift in velocities,
    % since we don't know, where the feet should be:  
    b = 10;
    sigma_lambda = sigma_lambda + b*[footR.A_IB(1:2,1:2) * footR.B_v_B(1:2);
                                             footL.A_IB(1:2,1:2) * footL.B_v_B(1:2)];

    % Uncomment the following lines to only constrain then motion of
    % one foot.
%    J_lambda = J_lambda(3:4,:);
%    sigma_lambda = sigma_lambda(3:4);
end

function tau = controller(q, q_dot)
    % Setting up a virtual model controller.  Many steps (such as
    % computing 'g' might seem redundant, since they are already done
    % in the simulation, but keep in mind that normally you would have
    % to implement this function in the controller of a real robot. So
    % q and q_dot are generally measurements obtained from sensors. And
    % not just byproducts of a simulation.
    %
    % Set joint angles:
    virtual3DOF.q    = q_(1:3);
    hipL.q           = q_(4);
    hipR.q           = q_(5);
    legExtensionL.q  = q_(6);
    legExtensionR.q  = q_(7);
    % Set joint velocities:
    virtual3DOF.qDot    = d_q_(1:3);
    hipL.qDot           = d_q_(4);
    hipR.qDot           = d_q_(5);
    legExtensionL.qDot  = d_q_(6);
    legExtensionR.qDot  = d_q_(7);
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    virtual3DOF.qDDot    = [0;0;0];
    hipL.qDDot           = 0;
    hipR.qDDot           = 0;
    legExtensionL.qDDot  = 0;
    legExtensionR.qDDot  = 0;
    
    % Recursively compute the Jacobians, positions, velocities,
    % and bias-accelerations (by setting q_ddot to 0).  They are stored
    % in the individual Body-objects that are then access from the
    % subsequent routines.
    % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
    % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
    ground.recursiveForwardKinematicsVeAcJa();
    %
    % Spring forces:
    I_x_des   = [1.25;1]; % Desired position of the COG (in inertial coordinates)
    I_v_des   = [0;0];    % Desired velocity of the COG (in inertial coordinates)
    I_x = mainBody.A_IB(1:2,1:2) * mainBody.B_r_IB(1:2,:); % Actual position
    I_v = mainBody.A_IB(1:2,1:2) * mainBody.B_v_B(1:2,:); % Actual velocity
    k = 5; % Virtual stiffness
    b = 2; % Virtual damping
    %
    I_phi_des = 0.2;  % Desired pitch of the main body(in inertial coordinates)
    I_omega_des = 0;  % Desired pitch velocity of the main body(in inertial coordinates)
    I_phi = atan2(mainBody.A_IB(2,1),mainBody.A_IB(1,1));  % Actual pitch
    I_omega = mainBody.B_omega_B(3,1);  % Actual pitch velocity
    k_rot = 10;   % Virtual rotational stiffness
    b_rot = 5;    % Virtual rotational damping
    %
    I_F_virt = (I_x_des - I_x)*k +...
               (I_v_des - I_v)*b;
    I_M_virt =  (I_phi_des   - I_phi)*k_rot +...
                (I_omega_des - I_omega)*b_rot;
    %
    % Gravity compensation. For simplicity, we're computing the full
    % set of values (incl M and f), but we will just use the g-part of
    % the EOM. This is obviously not the most efficient implementation:
    [M, f, g] = ground.recursiveComputationOfMfg();
    %
    % The Jacobian of the COG (this is how the virtual forces and the
    % virtual moment get mapped into the generalized coordinate space:
    J_MB = [ mainBody.A_IB(1:2,1:2)*mainBody.B_J_S(1:2,:);
              mainBody.B_J_R(3,:)];
    % The Jacobian of the contact points (this is how the ground
    % contact forces get mapped into the generalized coordinate space:
    [J_lambda, sigma_lambda] = GetConstraintTerms();
    % Compute the desired generalized torques:
    tau_des = J_MB'*[I_F_virt;I_M_virt] - g;
    %
    % use the pseudo-inverse to compute the desired contact forces from
    % the desired passive torques: 
    lambda_des = pinv(J_lambda(1:4,1:3)')*tau_des(1:3);
    %
    % Compute the active joint torques that create the desired torques
    % and balance the contact forces:
    tau_active = tau_des(4:7) - J_lambda(1:4,4:7)'*lambda_des;
    % The full torque vector is 0 in the first 3 columns and the active
    % torques:
    tau = [0;0;0;tau_active];
end
end
