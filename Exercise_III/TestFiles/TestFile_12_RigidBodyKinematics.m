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
print(gcf,'-r600','-djpeg','Problem_12_Output.jpg'); 


%% Problem 12a:
% Define initial conditions:
B.A_IB          = eye(3);
B.B_omega_B     = [ 0; 1.5; 0];
B.B_omegaDot_B  = [ 0; 0; 0];
B.B_r_IB        = [ 0; 0; 0];
B.B_v_B         = [ 0; 0; 0];
B.B_a_B         = [ 0; 0; 0];

% Set the auto updating of graphics to false, so the simulation runs a bit
% faster:
B.autoUpdate = false;

% Set up integration
delta_t = 0.01;     % Stepsize of integrator
delta_t_show = 0.1;  % Stepsize of output
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

% Display the orientation and position at the end of the run for comparison
disp(B.A_IB);
disp(B.B_r_IB);

%% Problem 12b:
% Define initial conditions:
B.A_IB          = eye(3);
B.B_omega_B     = [ 0; 0; 4];
B.B_omegaDot_B  = [ 0; 0; 0];
B.B_r_IB        = [ 0; 0; 0];
B.B_v_B         = [ 0; 0; 0];
B.B_a_B         = [ 0; 3; 0];

% Set up integration
delta_t = 0.001;     % Stepsize of integrator
delta_t_show = 0.1;  % Stepsize of output
t_start = 0;
t_end   = 3;
% Initialize variables to store the motion of points B, Q.
n = (t_end-t_start)/delta_t;
I_r_IB = zeros(3,n);
I_v_B  = zeros(3,n);
I_a_B  = zeros(3,n);
I_r_IQ = zeros(3,n);
I_v_Q  = zeros(3,n);
I_a_Q  = zeros(3,n);

% Perform integration:
t = t_start;
next_t_output = t_start;
i = 0;
while t<t_end
    t = t + delta_t;
    i = i +1;
    % Call the integration step
    B.integrationStep(delta_t);
    % Save positions, velocities, and accelerations of B and Q
    I_r_IB(:,i) = B.positionOfPoint([0;0;0]);
    I_v_B(:,i)  = B.velocityOfPoint([0;0;0]);
    I_a_B(:,i)  = B.accelerationOfPoint([0;0;0]);
    I_r_IQ(:,i) = B.positionOfPoint([0;3/16;0]);
    I_v_Q(:,i)  = B.velocityOfPoint([0;3/16;0]);
    I_a_Q(:,i)  = B.accelerationOfPoint([0;3/16;0]);
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

% Show results:
figure
subplot(2, 2, 1)
hold on
plot(I_r_IB(1,:),I_r_IB(2,:),'r');
plot(I_r_IQ(1,:),I_r_IQ(2,:),'b');
title('Positions')
legend('Point B','Point Q')
grid on
axis equal

subplot(2, 2, 2)
hold on
plot(I_v_B(1,:),I_v_B(2,:),'r');
plot(I_v_Q(1,:),I_v_Q(2,:),'b');
title('Velocities')
legend('Point B','Point Q')
grid on
axis equal

subplot(2, 2, 3)
hold on
plot(I_a_B(1,:),I_a_B(2,:),'r');
plot(I_a_Q(1,:),I_a_Q(2,:),'b');
title('Accelerations')
legend('Point B','Point Q')
grid on
axis equal
% Save screen shot:
print(gcf,'-r600','-djpeg','Problem_12b_Output.jpg'); 


