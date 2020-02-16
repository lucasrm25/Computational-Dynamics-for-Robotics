%% Simulate a fully plastic collision with a Passive Dynamic Walker
clear all
close all
clc

%% Compute the mass matrix 
syms m l real;
syms x y phiT phiL real;
q = [x, y, phiT, phiL];

pos1 = [x + l/2*sin(phiT);
        y - l/2*cos(phiT)];
pos2 = [x + l/2*sin(phiL);
        y - l/2*cos(phiL)];
    
J1 = jacobian(pos1, q);  
disp(simplify(J1));

J2 = jacobian(pos2, q);   
disp(simplify(J2));

M = J1'*m*J1 + J2'*m*J2;
disp(simplify(M));

%% Actual collision computation
% Define parameters:
m = 1;    % Point mass at the legs
l = 1;    % Leg length
phiT_dot = 1; % Rotational velocity of the trailing leg
vFact = 1;    % Velocity factor of the leading leg (phiL_dot = vFact*phiT_dot)

% The collision is simulated as a function of the contact angle phi (with
% phiT = phi and phiL = -phi at the instance of the collision), which is
% altered between 1 and 90 degs: 
phi_max = 90;
phi_vec = 0:0.1:phi_max;
% NOTE: We assumed mass, leg length, and the two velocities to be 1.  This
% is arbitrary, but since everything scales linearly with mass, leg length,
% and velocity, this is also very general.

% For all angles phi:
Lambda_1     = zeros(4,length(phi_vec));
c_dot_PLUS_1 = zeros(4,length(phi_vec));
Lambda_2     = zeros(4,length(phi_vec));
c_dot_PLUS_2 = zeros(4,length(phi_vec));
Lambda_3     = zeros(4,length(phi_vec));
c_dot_PLUS_3 = zeros(4,length(phi_vec));
Lambda_4     = zeros(4,length(phi_vec));
c_dot_PLUS_4 = zeros(4,length(phi_vec));
Lambda_5     = zeros(4,length(phi_vec));
c_dot_PLUS_5 = zeros(4,length(phi_vec));
for i = 1:length(phi_vec)
    phiT = phi_vec(i)*pi/180;
    % Compute the generalized coordinates:
    x    = -l*sin(phiT); 
    y    = +l*cos(phiT);
    phiL = -phiT;
    
    % Compute the mass matrix:
    M = [2*m             0               m*l/2*cos(phiT) m*l/2*cos(phiL);
         0               2*m             m*l/2*sin(phiT) m*l/2*sin(phiL);
         m*l/2*cos(phiT) m*l/2*sin(phiT) m*(l/2)^2       0;[1 0 l*cos(phiT) 0;
                0 1 l*sin(phiT) 0;
                1 0 0           l*cos(phiL);
                0 1 0           l*sin(phiL)]; 
         m*l/2*cos(phiL) m*l/2*sin(phiL) 0               m*(l/2)^2];
    % Compute the contact Jacobian:
    J_lambda =    
    % Compute the pre-impact velocity:
    q_dot_MINUS = [-l*cos(phiT) -l*sin(phiT) 1 vFact]'*phiT_dot;
    % Compute the pre-impact velocity in the contact space:
    c_dot_MINUS = J_lambda*q_dot_MINUS;
    
    % Compute the contact space mass matrix:
    M_Lambda = inv(J_lambda*inv(M)*J_lambda');
    
    % Seperate the contact space mass matrix and contact space velocity:
    M_T = M_Lambda(1:2,1:2);
    M_C = M_Lambda(1:2,3:4);
    M_L = M_Lambda(3:4,3:4);
    c_dot_MINUS_T = c_dot_MINUS(1:2);
    c_dot_MINUS_L = c_dot_MINUS(3:4);
    
    % case 1: As the leading foot hits the ground in the collision, the
    % trailing foot comes off the ground. This outcome is expected to
    % happen when the feet are closer together and the momentum of the
    % walker is pointed forward rather than downward. In this case,
    % impulses at the trailing foot are zero
    %
    % Compute impulses and post impact velocities:
    Lambda_T     = [0;0];
    c_dot_PLUS_L = [0;0];
    c_dot_PLUS_T = inv(M_T)*(M_T*c_dot_MINUS_T - M_C*(c_dot_PLUS_L - c_dot_MINUS_L) + Lambda_T);
    Lambda_L = [M_C',M_L]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    % Combine results and store in array:
    Lambda_1(:,i)     = [Lambda_T; Lambda_L];
    c_dot_PLUS_1(:,i) = [c_dot_PLUS_T; c_dot_PLUS_L];
    
    % case 2: After the collision, the trailing foot will remain in place
    % and the system will come to a complete stop.  This outcome is
    % expected to happen when the feet are further apart and the momentum
    % of the walker is pointed downward rather than forward. 
    %
    % Compute impulses and post impact velocities:
    c_dot_PLUS_T = [0;0];
    c_dot_PLUS_L = [0;0];
    Lambda_T = [M_T, M_C]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    Lambda_L = [M_C',M_L]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    % Combine results and store in array:
    Lambda_2(:,i)     = [Lambda_T; Lambda_L];
    c_dot_PLUS_2(:,i) = [c_dot_PLUS_T; c_dot_PLUS_L];
    
    % case 3: The trailing foot slides after the collision. Only the normal
    % component of the trailing foot velocity will remain zero. The
    % horizontal trailing foot velocity is unconstrained. For a Coulomb
    % impact in which the trailing foot slides in the negative x direction
    % (i.e., in the direction of travel), the tangential trailing foot
    % impulse lambdaT,x is given as LambdaT,x = +/-LambdaT,y * mu*    
    %
    mu = +1;
    %
    % Compute impulses and post impact velocities:
    c_dot_PLUS_L = [0;0];
    A = [M_T*[1;0],-[1;1/mu]];
    b = M_T*c_dot_MINUS_T - M_C*(c_dot_PLUS_L - c_dot_MINUS_L);
    x = A\b;
    c_dot_PLUS_T = [1;0]*x(1);
    Lambda_T = [1;1/mu]*x(2);
    Lambda_L = [M_C',M_L]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    % Combine results and store in array:
    Lambda_3(:,i)     = [Lambda_T; Lambda_L];
    c_dot_PLUS_3(:,i) = [c_dot_PLUS_T; c_dot_PLUS_L];
    
    
    mu = -1;
    %
    % Compute impulses and post impact velocities:
    c_dot_PLUS_L = [0;0];
    A = [M_T*[1;0],-[1;1/mu]];
    b = M_T*c_dot_MINUS_T - M_C*(c_dot_PLUS_L - c_dot_MINUS_L);
    x = A\b;
    c_dot_PLUS_T = [1;0]*x(1);
    Lambda_T = [1;1/mu]*x(2);
    Lambda_L = [M_C',M_L]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    % Combine results and store in array:
    Lambda_4(:,i)     = [Lambda_T; Lambda_L];
    c_dot_PLUS_4(:,i) = [c_dot_PLUS_T; c_dot_PLUS_L];
    
    % Limit of mu = infty
    % Compute impulses and post impact velocities:
    c_dot_PLUS_L = [0;0];
    A = [M_T*[1;0],-[1;0]];
    b = M_T*c_dot_MINUS_T - M_C*(c_dot_PLUS_L - c_dot_MINUS_L);
    x = A\b;
    c_dot_PLUS_T = [1;0]*x(1);
    Lambda_T = [1;0]*x(2);
    Lambda_L = [M_C',M_L]*[c_dot_PLUS_T - c_dot_MINUS_T;c_dot_PLUS_L - c_dot_MINUS_L];
    % Combine results and store in array:
    Lambda_5(:,i)     = [Lambda_T; Lambda_L];
    c_dot_PLUS_5(:,i) = [c_dot_PLUS_T; c_dot_PLUS_L];
end

% Show output:
Example_11_CreateVelLambdaFigure(1, 'Case 1: Leading Leg Collision Only', phi_vec, c_dot_PLUS_1, Lambda_1);
Example_11_CreateVelLambdaFigure(2, 'Case 2: Leading and Trailing Leg Collision', phi_vec, c_dot_PLUS_2, Lambda_2);
%
Example_11_CreateVelLambdaFigure(3, 'Case 3: Leading Leg Collision, Trailing Leg Slides mu = +1', phi_vec, c_dot_PLUS_3, Lambda_3);
Example_11_CreateVelLambdaFigure(4, 'Case 4: Leading Leg Collision, Trailing Leg Slides mu = -1', phi_vec, c_dot_PLUS_4, Lambda_4);
%
Example_11_CreateVelLambdaFigure(5, 'Case 5: Leading Leg Collision, Trailing Leg Slides mu = infty', phi_vec, c_dot_PLUS_5, Lambda_5);

        