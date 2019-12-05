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
% different kinematic values: 
B = RigidBodyKinematicsCLASS(env);
B.bodyName = 'B';
B.A_IB          = eye(3);
B.B_omega_B     = [ 1; 1; 0];
B.B_omegaDot_B  = [ 0; 1; 1];
B.B_r_IB        = [ 1; 1; 1];
B.B_v_B         = [-1; 0; 0];
B.B_a_B         = [ 0;-1; 0];
% Resize the graphics window to show all components
env.resetOutput();

%% Problem 14:
% Define initial conditions:
B.A_IB          = eye(3);
B.B_omega_B     = [ 0; 1.5; 1.5];
B.B_omegaDot_B  = [ 0; 0; 0];
B.B_r_IB        = [ 0; 0; 0];
B.B_v_B         = [ 0; 0; 0];
B.B_a_B         = [ 1.5; 1.5; 0];

% Set up integration
delta_t = 0.01;     % Stepsize of integrator
delta_t_show = 0.01;  % Stepsize of output
t_start = 0;
t_end   = 1;

% Perform integration:
t = t_start;
next_t_output = t_start;
while t<t_end
    t = t + delta_t;
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

% Save final screen shot:
B.updateGraphics();
env.resetOutput();
print(gcf,'-r600','-djpeg','Problem_14_Output.jpg'); 