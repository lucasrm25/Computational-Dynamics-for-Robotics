%% Define symbolic variables and the vector of generalized coordinates:
clear all
clc

syms q1 q2 real
q = [q1; q2];
syms q1_dot q2_dot real
q_dot = [q1_dot; q2_dot];

syms l_1 l_2 m_1 m_2 g real

Fg_1 = [0; -m_1*g];
Fg_2 = [0; -m_2*g];

%% Kinematics:
% We could also do this with our recursive toolbox, but for such a simple
% system it's easier to do it by hand:

x_1 = [+l_1*sin(q1)
       -l_1*cos(q1)];
x_2 = x_1 + [+l_2*sin(q1+q2)
             -l_2*cos(q1+q2)];

% Get jacobians via partial derivatives:   
Jf_1 = jacobian(x_1, q);
disp('J_1:')
disp(Jf_1);
Jf_2 = jacobian(x_2, q);
disp('J_2:')
disp(Jf_2);

% Get bias acceleration via taking the partial derivative of "J_f*q_dot"
% and multiplying it with q_dot: 
sigma_1 = simplify(expand(jacobian(Jf_1*q_dot, q)*q_dot));
disp('sigma_1:')
disp(sigma_1);
sigma_2 = simplify(expand(jacobian(Jf_2*q_dot, q)*q_dot));
disp('sigma_2:')
disp(sigma_2);

%% Put together the components of the EOM:

M = simplify(expand(  Jf_1'*m_1*Jf_1    +  Jf_2'*m_2*Jf_2    ));
disp('M(q):')
disp(M)

f = simplify(expand( -Jf_1'*m_1*sigma_1 + -Jf_2'*m_2*sigma_2 ));
disp('f(q, q_dot):')
disp(f)

g = simplify(expand(  Jf_1'*Fg_1        +  Jf_2'*Fg_2        ));
disp('g(q):')
disp(g)

%% Show that the constraint forces are
% Since we know the direction of the constraint forces, we can express them
% as a combination of these directions multiplied by a scalar:
syms c_1 c_2 real
Fc_2 = c_2*[-sin(q1+q2);
            +cos(q1+q2)];      
Fc_1 = -Fc_2 + c_1*[-sin(q1);
                    +cos(q1)];
Fc = [Fc_1; Fc_2];        
disp('Fc (scaled):')
disp(Fc);

% Show that constraint forces and velocities are perpendicular to each
% other:
Jf = [Jf_1; Jf_2];
disp('Jf''*Fc:')
disp(simplify(expand(Jf'*Fc)))


