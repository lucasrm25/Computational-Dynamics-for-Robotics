function Solution_44_EventDetection()
    % Define timing and intial parameters:
    t_end   = 1.0;
    delta_t = 0.01;
    q_0     = [1; 1; pi/4];
    q_dot_0 = [0; 0; 0];
    m     = 0.5;
    theta =  1;
    grav  = 9.81;
    d     = 0;
    s     = 0.5; % [m]
    % Combined vector of initial generalized positions and velocities:
    y_0 = [q_0; q_dot_0];
    % Set up options with our event function:
    options = odeset('Events',@events);

    % run simulation/integration
    [t, y] = ode45(@ODE, 0:delta_t:t_end, y_0, options);

    % Plot results:
    figure;
    hold on;
    grid on;
    box on
    plot(t, y);
    print(gcf,'-r600','-djpeg','Problem_44_Output.jpg');
    
    % This function implements the right hand side of the differential
    % equation. 
    function y_dot = ODE(~, y)
        q_     = y(1:3);
        q_dot_ = y(4:end);
        A_num_ = A_fct(q_(1), q_(2), q_(3), q_dot_(1), q_dot_(2), q_dot_(3), m, theta, grav, d);
        b_num_ = b_fct(q_(1), q_(2), q_(3), q_dot_(1), q_dot_(2), q_dot_(3), m, theta, grav, d);
        x_ = A_num_\b_num_;
        % Extract accelerations:
        q_ddot_ = x_(1:3);
        % Prepare the output variable y_dot:
        y_dot = zeros(6,1);
        y_dot(1:3)   = q_dot_;
        y_dot(4:end) = q_ddot_;
    end

    % This function is called at each integraton step to check whether an
    % event has happend (i.e., 'value' has changed it's sign).
    function [value, isterminal, direction] = events(~, y)
        % The precise definition for the constraint distance of the second
        % contact is "y(2)*cos(y(3))-(y(1)-s)*sin(y(3))-d = 0".  Note that
        % this can be simplified to "y(3) = 0", since the first contact is
        % already enforced. 
        value      = y(2)*cos(y(3))-(y(1)-s)*sin(y(3))-d;
        isterminal = 1;   % Abort the integration
        direction  = -1;  % Only register a downward motion
    end
end

