% Define parameters:
syms m theta grav d real

% Define coords:
syms x y phi real
q = [x, y, phi]';

% Define velocities:
syms dx dy dphi real
dq = [dx, dy, dphi]';

% Define the terms of the unconstrained EOMs (Part a):
% *************************************************************************
% ToDo:                                                                  
% Derive on paper and then implement here the equations of motion by
% defining the terms M, f, g, and tau
% *************************************************************************

A_BI = [ cos(phi) sin(phi) 0;
        -sin(phi) cos(phi) 0
        0         0        1];

% translational jacobian
fc_S = [x;y;0];
J_S = jacobian(fc_S,q);
B_J_S = A_BI * J_S;
% translational bias acceleration
sigma_S = arrayfun(@(x) jacobian(x,q)*dq, J_S) * dq;
B_sigma_S = A_BI * sigma_S;

% rotational jacobian
fc_R = [0;0;phi];
J_R = jacobian(fc_R,q);
B_J_R = A_BI * J_R;
% rotational bias acceleration
sigma_R = arrayfun(@(x) jacobian(x,q)*dq, J_R) * dq;
B_sigma_R = A_BI * sigma_R;

% inertia
B_I_B = diag([0 0 theta]);

% body rotation
B_omega_B = [0;0;dphi];

% graviational force
I_Fgrav = [0;-9.81;0]*m;
B_Fgrav = A_BI * I_Fgrav;
% graviational moment
I_Mgrav = [0;0;0];
B_Mgrav = A_BI * I_Mgrav;

M = B_J_S' * m     * B_J_S + ...
    B_J_R' * B_I_B * B_J_R;
M = simplify(M);
          
f = - ( B_J_S' * m * B_sigma_S + ...
        B_J_R' * ( B_I_B * B_sigma_R + skew(B_omega_B) * B_I_B * B_omega_B) );
f = simplify(f);
    
g = B_J_S' * B_Fgrav + ...
    B_J_R' * B_Mgrav;
g = simplify(g);

tau = [0;0;0];


% Define the constraint violation (Part b):
% *************************************************************************
% ToDo:                                                                  
% Derive on paper and then implement here an equation that describes the
% constraint violation c as a function of q.  Then compute the associated
% Jacobian and bias acceleration.  Note that this is a scleronomic system,
% expresssed in inertial coordinates.  So you can use partial derivatives
% to get J and use J_dot*q_dot for the bias accelerations.
% *************************************************************************

c = [0 1 0] * ( A_BI*[x;y;0] - [0;d;0] );
J_lambda = jacobian(c,q);
sigma_lambda = arrayfun(@(x) jacobian(x,q)*dq, J_lambda) * dq;
sigma_lambda = simplify(expand(sigma_lambda));

% Define the terms for the constrained equations of motion:
A = [M         -J_lambda'; 
     J_lambda          0 ];
b = [f + g + tau; -sigma_lambda];
result = simplify(expand(A\b));
disp('q_ddot:');
disp(result(1:3));
disp('lambda:');
disp(result(4));

% And create a matlab function for this matrix, as well as for the transformation A_IC:
matlabFunction(A,'file','A_fct.m', 'vars', [x, y, phi, dx, dy, dphi, m, theta, grav, d]);
matlabFunction(b,'file','b_fct.m', 'vars', [x, y, phi, dx, dy, dphi, m, theta, grav, d]);

% Evaluate the EOMs with specific values (Part c):
A_num = A_fct(1, 1, pi/4, 0, 0, 0, 0.5, 1, 9.81, 0);
b_num = b_fct(1, 1, pi/4, 0, 0, 0, 0.5, 1, 9.81, 0);
result = A_num\b_num;
disp('q_ddot:');
disp(result(1:3));
disp('lambda:');
disp(result(4));



% % Define parameters:
% syms m theta grav d real
% 
% % Define coords:
% syms x y phi real
% q = [x, y, phi]';
% 
% % Define velocities:
% syms dx dy dphi real
% dq = [dx, dy, dphi]';
% 
% % Define the terms of the unconstrained EOMs (Part a):
% M = diag([m; m; theta]);
% f = zeros(3,1);
% g = [0; -m*grav; 0];
% tau = zeros(3,1);
% 
% % Define the constraint violation (Part b):
% c = y*cos(phi) - x*sin(phi) - d;
% J_lambda = jacobian(c,q);
% sigma_lambda = simplify(expand(dq'*jacobian(J_lambda,q)*dq));
% 
% % Define the terms for the constrained equations of motion:
% A = [M, -J_lambda' ; J_lambda,0];
% b = [f + g + tau; -sigma_lambda];
% result = simplify(expand(A\b));
% disp('q_ddot:');
% disp(result(1:3));
% disp('lambda:');
% disp(result(4));
% 
% % And create a matlab function for this matrix, as well as for the transformation A_IC:
% matlabFunction(A,'file','A_fct.m', 'vars', [x, y, phi, dx, dy, dphi, m, theta, grav, d]);
% matlabFunction(b,'file','b_fct.m', 'vars', [x, y, phi, dx, dy, dphi, m, theta, grav, d]);
% 
% % Evaluate the EOMs with specific values (Part c):
% A_num = A_fct(1, 1, pi/4, 0, 0, 0, 0.5, 1, 9.81, 0);
% b_num = b_fct(1, 1, pi/4, 0, 0, 0, 0.5, 1, 9.81, 0);
% result = A_num\b_num;
% disp('q_ddot:');
% disp(result(1:3));
% disp('lambda:');
% disp(result(4));
