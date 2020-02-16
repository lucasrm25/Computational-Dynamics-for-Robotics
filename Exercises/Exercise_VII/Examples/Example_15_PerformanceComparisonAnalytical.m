function Example_15_PerformanceComparisonAnalytical
    clc
    %% Example for efficient and inefficient code generation:
    % This example shows how to compute the Mass Matrix for a simple
    % nq-link robot that consists of segments of length l that are all
    % connected by rotational joints.  The COG of each segment (with mass m
    % and inertia j) is located at a distance s from the joint
    
    % Define kinematics recursively for nq joints in the systems
    nq = 6;
    n_trials = 100000;
    
    % Generalized Coordinates
    q = sym('q',[nq 1]);
    assume(q,'real');
    % Parameters 
    l = sym('l','real');
    s = sym('s','real');
    m = sym('m','real');
    j = sym('j','real');
    p = [l s m j]';
    % Kinematics (planar):
    x   = sym('x', [2*nq, 1]);
    phi = sym('phi', [nq, 1]);
    % Start of the chain:
    phi(1) = q(1);
    x(1:2) = s*[-sin(q(1)); cos(q(1))];
    % Add the chain recursively:
    for i = 2:nq
        phi(i)       = phi(i-1) + q(i);
        x(2*i-1:2*i) = x(2*i-3:2*i-2) + ...
                       (l-s)*[-sin(phi(i-1)); cos(phi(i-1))] +...
                           s*[-sin(phi(i));   cos(phi(i))];
        phi(i)       = phi(i);                   
        x(2*i-1:2*i) = x(2*i-1:2*i);
        %x   = simplify(expand(x));
        %phi = simplify(expand(phi));
    end
    disp('phi:')
    disp(phi)
    disp('x:')
    disp(x)

    % Jacobians:
    J_R = jacobian(phi,q);
    J_S = jacobian(x,q);
    
    % Mass matrix:
    M = sym(zeros(nq));
    for i = 1:nq %nq = nN (# bodies)
        M = M + J_R(i,:)'*j*J_R(i,:) + J_S(i*2-1:i*2,:)'*m*J_S(i*2-1:i*2,:);
    end
    disp('M:')
    disp(M)
    
    % Just create a handle to the function without code optimization
    M_fct = matlabFunction(M,'vars', {p, q});
    
    % Use Matlab to generate optimized code:
    matlabFunction(M,'file','M_mat','vars', {p, q});
    
    disp('Created Matlab functions.')

    % Compare performances for random configurations of many trials:
    l_num = 1;
    s_num = 0.5;
    m_num = 1;
    j_num = 1;
    p_num = [l_num s_num m_num j_num]';
    
    %% Use solve
    for k = 1:n_trials
        q_num = rand([nq,1])*0;
        M_eval = DirectEval(p_num, q_num);
    end
    disp('Direct Solve:')
    disp(M_eval)
    
    %% Use the Matlab Function (which exploits recursion):
    for k = 1:n_trials
        q_num = rand([nq,1])*0;
        M_eval = MatlabFct(p_num, q_num);
    end
    disp('Matlab Function:')
    disp(M_eval)
    
   
    
    %% Embedded Functions:
    % Direct evalutation of the symbolically derived function:
    function M_num1 = DirectEval(p_num, q_num)
        M_num1 = M_fct(p_num, q_num);
    end

    % Using the optimized code from 'MatlabFunction'
    function M_num2 = MatlabFct(p_num, q_num)
        M_num2 = M_mat(p_num, q_num);
    end
end
