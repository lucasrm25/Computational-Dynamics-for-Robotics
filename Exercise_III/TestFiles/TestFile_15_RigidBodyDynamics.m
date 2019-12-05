%% INITIALIZE:
clear all
close all
clc
green = [0;1;0];  

% Create a 'physical' (graphical) environment:
env = EnvironmentCLASS();

% Create an inertial reference frame:
I = CoSysCLASS(env, eye(3));
I.color = green;
I.name = 'I';

% Create a body, and fill in some arbitrary values to visualize the
% different kinematic and dynamic values: 
B = RigidBodyDynamicsCLASS(env);
B.bodyName = 'B';
B.A_IB          = eye(3);
B.B_omega_B     = [ .01; 0.01; 1];
B.B_omegaDot_B  = [ 0; 1; 1];
B.B_r_IB        = [ 1; 1; 1];
B.B_v_B         = [0; 0; 0];
B.B_a_B         = [ 0;-1; 0];
% Define mass and inertia for this rigid body:
B.m_B   = 1;             % [Kg]
B.B_I_B = diag([1.5,2,2.5]); % [Kg*m^2]
% Resize the graphics window to show all components
env.resetOutput();

%% QUESTION 15a:
% Define initial conditions for position/orientation and velocities
B.A_IB          = eye(3);
B.B_omega_B     = [ 1; 1; 1];
B.B_r_IB        = [ 0; 0; 0];
B.B_v_B         = [ 0; 0; 0];

% Set the auto updating of graphics to false, so the simulation runs a bit
% faster:
B.autoUpdate = false;

% Set up integration
delta_t = 0.001;     % Stepsize of integrator
delta_t_show = 0.1;  % Stepsize of output
t_start = 0;
t_end   = 30;

% Perform integration:
t = t_start;
next_t_output = t_start;
while t<t_end
    t = t + delta_t;
    % Compute the accelerations
    B.computeNaturalDynamics();
    % Call the integration step
    B.integrationStep(delta_t);
    % Every delta_t_show, the current situation is visualized.  By calling
    % the function 'updateGraphics', the graphical objects that represent
    % the body and its motion are updated. 'env.resetOutput' makes sure
    % that the graphical windows adapts in size, and 'drawnow()' instructs
    % matlab to not wait until the loop is done to update the graphics
    % window.
    if t > next_t_output
        B.updateGraphics();
        env.resetOutput();
        drawnow()
        next_t_output = next_t_output + delta_t_show;
    end
end

% Display the orientation and position at the end of the run for comparison
disp(B.A_IB);
disp(B.B_r_IB);

% Save final screen shot:
B.updateGraphics();
env.resetOutput();
print(gcf,'-r600','-djpeg','Problem_15_Output.jpg'); 