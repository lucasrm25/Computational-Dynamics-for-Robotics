function Example_14_PerformanceComparisonNumerical
    clc
    %% Example for efficient and inefficient code generation:
    % This example shows how to compute the Mass Matrix for a simple
    % nq-link robot that consists of segments of length l that are all
    % connected by rotational joints.  The COG of each segment (with mass m
    % and inertia j) is located at a distance s from the joint
    
    % Define kinematics recursively for nq joints in the systems
    nq = 50;
    %nq = 500;
    %nq = 1000;
    %nq = 2000;
    n_trials = 1;
    
    % Compare performances for random configurations of many trials:
    l_num = 1;
    s_num = 0.5;
    m_num = 1;
    j_num = 1;
    p_num = [l_num s_num m_num j_num]';
    
    %% Do it numerical:
    for k = 1:n_trials
        q_num = rand([nq,1])*0;
        M_eval = NumericalOrder3(p_num, q_num);
    end
    
    %% Do it numerically using a slightly optimized inverse dynamics recursive formulation: 
    for k = 1:n_trials
        q_num = rand([nq,1])*0;
        M_eval = NumericalOrder2(p_num, q_num);
    end
    
    disp('DONE');
    
    %% Embedded Functions:
    % Compute the Mass Matrix numerically:
    function M_num3 = NumericalOrder3(p_num, q_num)
        l_num = p_num(1);
        s_num = p_num(2);
        m_num = p_num(3);
        j_num = p_num(4);
        % Positions:
        phi_num = q_num(1);
        % Jacobians:
        J_R_num = zeros(1, nq);
        J_R_num(1,1) = 1;
        J_S_num = zeros(2, nq);
        J_S_num(:,1) = s_num*[-cos(phi_num);-sin(phi_num)];
        % Mass matrix:
        M_num3 = J_R_num'*j_num*J_R_num + J_S_num'*m_num*J_S_num;
        for i_ = 2:nq %nq = nN (# bodies)
            J_R_part = zeros(1, nq);
            J_R_part(i_) = 1;
            J_S_num = J_S_num + (s_num*[-cos(phi_num);-sin(phi_num)])*J_R_num + (l_num-s_num)*[-cos(phi_num + q_num(i_));-sin(phi_num + q_num(i_))]*(J_R_num + J_R_part);
            J_R_num = J_R_num + J_R_part;
            phi_num = phi_num + q_num(i_);
            M_num3 = M_num3 + J_R_num'*j_num*J_R_num + J_S_num'*m_num*J_S_num;
        end
    end

    function M_num4 = NumericalOrder2(p_num, q_num)
        l_num = p_num(1);
        s_num = p_num(2);
        m_num = p_num(3);
        j_num = p_num(4);
        M_num4 = zeros(nq);
        for i_ = 1:nq
            q_ddot_num = zeros(nq,1);
            q_ddot_num(i_) = 1;
            % Recursively compute all position and accelerations.
            % All velocities are set to 0.
            x_     = zeros(2*nq,1);
            phi_   = zeros(nq,1);
            ddx_   = zeros(2*nq,1);
            ddphi_ = zeros(nq,1);
            x_(1:2)   = s_num*[-sin(q_num(1)); cos(q_num(1))];
            phi_(1) = q_num(1);
            ddx_(1:2)   = s_num*[-cos(q_num(1)); -sin(q_num(1))]*q_ddot_num(1);
            ddphi_(1) = q_ddot_num(1);
            for j_ = 2:nq
                % Compute the position and acceleration of this body recursively:
                phi_(j_) = phi_(j_-1) + q_num(j_);
                x_(j_*2-1:j_*2)   = x_(j_*2-3:j_*2-2) + ...
                           (l_num-s_num)*[-sin(phi_(j_-1)); cos(phi_(j_-1))] +...
                                   s_num*[-sin(phi_(j_));   cos(phi_(j_))];
                ddphi_(j_) = ddphi_(j_-1) + q_ddot_num(j_);
                ddx_(j_*2-1:j_*2)   = ddx_(j_*2-3:j_*2-2) + ...
                             (l_num-s_num)*[-cos(phi_(j_-1)); -sin(phi_(j_-1))]*ddphi_(j_-1) +...
                                     s_num*[-cos(phi_(j_));   -sin(phi_(j_))]*ddphi_(j_);
            end
            f_new_ = [0;0];
            m_new_ = 0;
            tau_new_ = zeros(nq,1);
            for j_ = nq:-1:1
                % Create the torques and forces acting on the predecessor:
                m_new_ = m_new_ - j_num*ddphi_(j_) + cross2D([-l_num*sin(phi_(j_));l_num*cos(phi_(j_))],f_new_) - cross2D([-s_num*sin(phi_(j_));s_num*cos(phi_(j_))],m_num*ddx_(j_*2-1:j_*2));
                f_new_ = f_new_ - m_num*ddx_(j_*2-1:j_*2);
                tau_new_(j_) = m_new_;
            end
            M_num4(:,i_) = -tau_new_;
        end
        function z = cross2D(x,y)
            z = x(1)*y(2)- x(2)*y(1);
        end
    end
end
