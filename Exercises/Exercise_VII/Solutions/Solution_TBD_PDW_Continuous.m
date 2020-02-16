function TestFile_47_PDW_Continuous()
% This function runs the multi body simulation of a Passive Dynamic
% Walker  

%% Define all parameters:
%
% Kinematic parameters:
r = 0.5;    % [Normalized to leg length]
l = 1;      % [Normalized to leg length]
dx = 0.05;  % [Normalized to leg length]
dy = 0.7;   % [Normalized to leg length]
% Gravity:
% Gravity is pointing a little bit to the right, to power the walker.
% This is equivalent to having it walk on a shallow ramp.
delta = 1*pi/180;
I_grav = [1*sin(delta);-1*cos(delta);0]; % [Normalized to g]

%% Prepare graphical output:
%
close all
% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();
set(env.fig, 'name', '3D Graphics Window');
hold on;

% Define the bodies:
ground    = LinkedRigidBodyDynamicsMfgCLASS(env);
ground.bodyName    = 'Ground';
ground.autoUpdate  = false;
ground.m_B         = 0;   % [Normalized to leg mass] (ground has no mass)
ground.scale       = 0.4; % Defines how large the CoSys will be drawn
%
stanceLeg = LinkedRigidBodyDynamicsMfgCLASS(env);
stanceLeg.bodyName    = 'Stance Leg';
stanceLeg.autoUpdate  = false;
stanceLeg.m_B         = 1;   % [Normalized to leg mass]
stanceLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
stanceLeg.I_grav      = I_grav; % Define direction of gravity
stanceLeg.scale       = 0.5; % Defines how large the CoSys will be drawn
%
swingLeg  = LinkedRigidBodyDynamicsMfgCLASS(env);
swingLeg.bodyName    = 'Swing Leg';
swingLeg.autoUpdate  = false;
swingLeg.m_B         = 1;   % [Normalized to leg mass]
swingLeg.B_I_B       = diag([.03,.01,.03]);  % [Normalized to leg mass and length]
swingLeg.I_grav      = I_grav; % Define direction of gravity
swingLeg.scale       = 0.5; % Defines how large the CoSys will be drawn
%
swingLegJointCenter  = LinkedRigidBodyDynamicsMfgCLASS(env);
swingLegJointCenter.bodyName    = 'Swing Leg Joint Center';
swingLegJointCenter.autoUpdate  = false;
swingLegJointCenter.m_B         = 0;   % [Normalized to leg mass] Virtual Body, no mass
swingLegJointCenter.scale       = 0.25; % Defines how large the CoSys will be drawn

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
%
fixedConnection = LinkedGenericJointVeAcJaCLASS(env, swingLeg, swingLegJointCenter);
fixedConnection.P_r_PDp = [-dx;dy-l+r;0];  % In the center of the rolling contact joint
fixedConnection.A_PDp   = eye(3);    % not rotated;
fixedConnection.S_r_SDs = [0;0;0];  % the location is identical in both legs;
fixedConnection.A_SDs   = eye(3);    % not rotated;
fixedConnection.jointName = 'Fixed Connection';
fixedConnection.autoUpdate = false;
fixedConnection.scale      = 0.2;
%
% Show the topology:
ground.recursiveOutput('');

% So we can compute Jacobians, we need to specify the total number of
% joints (this is stored in the ground, where the first Jacobian is
% generated) and which joint is located where in the jacobian (stored
% in the individual joints):
ground.nq = 2;
rollingContact.qIndex = 1;
hip.qIndex = 2;

% Show the passive dynamic walker in the initial configuration
% Set all joint angles:
rollingContact.q = 0*pi/180;
hip.q            = 0*pi/180;
% Set all joint velocities:
rollingContact.qDot = -0.1;
hip.qDot            = +0.5;
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

%***** Simulation *****:
% Set initial conditions (q and d_q are stacked, as we convert this
% second order ODE to a first order ODE):  
y_0 = [rollingContact.q; hip.q; rollingContact.qDot; hip.qDot];
% Simulate motion    
[t_one, y_one] = ode45(@ODE, [0,2], y_0,odeset('Events',@groundContact,'AbsTol',1e-6,'RelTol',1e-3));
disp(['Contact at t = ',num2str(t_one(end))]);

%***** Compute the collision *****:
% Extract the generalized coordinates q_MINUS and velocities d_q_MINUS from
% the last point of the computed trajectory:
q_MINUS   = y_one(end,1:ground.nq)';
d_q_MINUS = y_one(end,ground.nq+1:end)';
% Compute the components of the equations of motion:
[M, f, g] = ground.recursiveComputationOfMfg();
[J_lambda, sigma_lambda] = GetConstraintTerms();
% Compute the impulses:
Lambda = -inv(J_lambda*(M\J_lambda'))*J_lambda*d_q_MINUS;
% Calculate post-impact generalized velocities
q_PLUS = q_MINUS;
d_q_PLUS = d_q_MINUS + (M\J_lambda')*Lambda;
%
disp(['q_PLUS: ',num2str(q_PLUS')]);
disp(['d_q_PLUS: ',num2str(d_q_PLUS')]);

% Animate motion using our framework:
for i = 1:length(t)
    % Set the joint variables to the values from the trajectory created
    % during simulation: 
    q_   = y(i,1:ground.nq);
    d_q_ = y(i,ground.nq+1:2*ground.nq);
    % Set joint angles:
    rollingContact.q = q_(1);
    hip.q = q_(2);
    % Set joint velocities:
    rollingContact.qDot = d_q_(1);
    hip.qDot = d_q_(2);
    % All joint accelerations are set to zero (this is not correct, but
    % we are only interested in position and velocity of the
    % end-effector):  
    rollingContact.qDDot = 0;
    hip.qDDot = 0;
    % Compute forward kinematics using the recursive outward pass
    ground.recursiveForwardKinematicsVeAcJa();
    % Update the graphics
    ground.recursiveGraphicsUpdate();
    drawnow();
    [value, isterminal, direction] = groundContact(t(i), y(i,:));
    figure(100)
    hold on
    plot(t(i),value,'r.')
end
print(gcf,'-r600','-djpeg','Problem_47_Output.jpg');

% Animate motion using better graphics:
addpath([pwd,'\BetterGraphics']);
pdw = PassiveWalker3DCLASS([I_grav(1),I_grav(2), l, 0, 1, dy, dx, r, 0]);
for i = 1:length(t)
    pdw.update([y(i,1),y(i,3),y(i,2),y(i,4)]);
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
    rollingContact.q = q_(1);
    hip.q = q_(2);
    % Set joint velocities:
    rollingContact.qDot = d_q_(1);
    hip.qDot = d_q_(2);
    % All joint accelerations are set to zero, so the subsequent call
    % to recursiveForwardKinematicsVeAcJa will produce bias
    % accelerations, not real accelerations:
    rollingContact.qDDot = 0;
    hip.qDDot = 0;

    % Recursively compute the Jacobians, positions, velocities,
    % and bias-accelerations (by setting q_ddot to 0).  They are stored
    % in the individual Body-objects that are then access from the
    % subsequent routines.
    % Note:  Since we did set q_ddot to zero, B_a_B, and B_omegaDot_B
    % will be the BIAS ACCELERATIONS, not the full accelerations!!!!
    ground.recursiveForwardKinematicsVeAcJa();
    % Compute the components of the equations of motion:
    [M, f, g] = ground.recursiveComputationOfMfg();
    % Solve for the accelerations without any driving torques:
    dd_q = M\(f + g);
    % Prepare the output variable d_y:
    d_y_ = zeros(2*ground.nq,1);
    % Compute the derivative of the positions from the velocities.
    d_y_(1:ground.nq)     = d_q_;
    % The derivative of the velocities are the generalized
    % accelerations: 
    d_y_(ground.nq+1:end) = dd_q;
end
%
% This event-handler detects when the wire in which the robot arm got
% entangled is fully stretched:
function [value, isterminal, direction] = groundContact(~, y_)
    % Extract the generalized coordinates q (we do not need the velocities)
    q_   = y_(1:ground.nq);
    % Set joint angles:
    rollingContact.q = q_(1);
    hip.q = q_(2);
    % Call recursive forwardkinematics.  Note: we do not need to compute
    % velocities, accelerations and jacobians:   
    ground.recursiveForwardKinematicsVeAcJa();
    % Comptute constraint-violation:
    contactPos = swingLegJointCenter.A_IB * swingLegJointCenter.B_r_IB - [0;r;0];
    value      = contactPos(2);
    isterminal = 1;
    direction  = -1;
end
end



