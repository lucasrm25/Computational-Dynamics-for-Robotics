% *************************************************************************
% ToDo:                                                                  
% Rename file and add it to the TestFiles folder
% *************************************************************************
function TestFile_38_PassiveDynamicWalker()
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

% Show the passive dynamic walker in the initial configuration
% Set all joint angles:
rollingContact.q = 0*pi/180;
hip.q            = 0*pi/180;
% Set all joint velocities:
rollingContact.qDot = -0.1;
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

%***** Simulation *****:
% Time of simulation:
t = linspace(0,2);
% Set initial conditions (q and d_q are stacked, as we convert this
% second order ODE to a first order ODE):  
y_0 = [rollingContact.q; hip.q; rollingContact.qDot; hip.qDot];
% Simulate motion    
[~, y] = ode45(@ODE, t, y_0);


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
end
print(gcf,'-r600','-djpeg','Problem_38_Output.jpg');

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

 % *************************************************************************
% ToDo:                                                                  
% Add all the code necessary to compute a forward dynamics simulation to
% learn how to implement the Newton-Euler Equations of motion. That is,
% compute dd_q using the recursive framework: 
% *************************************************************************
    dd_q = ;
    % Prepare the output variable d_y:
    d_y_ = zeros(2*ground.nq,1);
    % Compute the derivative of the positions from the velocities.
    d_y_(1:ground.nq)     = d_q_;
    % The derivative of the velocities are the generalized
    % accelerations: 
    d_y_(ground.nq+1:end) = dd_q;
end
end



