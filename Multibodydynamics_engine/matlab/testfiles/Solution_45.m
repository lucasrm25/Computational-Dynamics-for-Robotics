clear all; clc; close all;

% Define parameters:
syms m theta grav d real

% Define coords:
syms x y phi real
q = [x, y, phi]';

% Define velocities:
syms dx dy dphi real
dq = [dx, dy, dphi]';

%% Define equations of motion

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


%% Define the constraint violation and contact forces:

syms s l real

c = [ [0 1 0] * ( A_BI*[x;y;0] - [0;d;0] );
      [0 1 0] * ( A_BI * ( [x;y;0] - [s;0;0] ) - [0;d;0]) ];

J_lambda = jacobian(c,q);

sigma_lambda = arrayfun(@(x) jacobian(x,q)*dq, J_lambda) * dq;
sigma_lambda = simplify(expand(sigma_lambda));


% Define the terms for the constrained equations of motion:
A = [M         -J_lambda'; 
     J_lambda   0*J_lambda*J_lambda' ];
b = [f + g + tau; -sigma_lambda];
result = simplify(expand(A\b));

% calculate constraint forces
lambda = result(4:5);


%% Calculate Impact hypothesis (bar stops after contact)

% position at impact
q_m = [l;d;0];
% constraint forces at impact
lambda = subs( lambda, {q(1),q(2),q(3)}, {q_m(1), q_m(2), q_m(3)})

% Jacobian before impact
J_lambda_m = subs(J_lambda, {q(1),q(2),q(3)}, {q_m(1), q_m(2), q_m(3)})

% Define velocities before impact:
syms dx_m dy_m dphi_m real
dq_m = [dx_m, dy_m, dphi_m]';
dc_m = J_lambda_m * dq_m;

% since first constraint is supposed to be closed before impact, then:
dc_m(1) = 0;

% hypothesis: bar stops after contact
dc_p = [0;0];

M_lambda = simplify(inv(J_lambda_m / M * J_lambda_m'))

impulse = simplify(expand( M_lambda * ( dc_p - dc_m ) ));


% The first contact will open at the impact moment, if the first impulse is
% negative:

solve( impulse(1)==0, s)    % check impulse(1) < 0

% ANSWER:
% s < l- + theta-/(m*l-)
% 
% since theta- = 0
% 
% s < l





