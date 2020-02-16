clear all; close all; clc;

% Inertia matrix for cuboid at the COG
Icub_G = @(dim,m) [dim(2)^2+dim(3)^2 0 0; 0 dim(1)^2+dim(3)^2 0; 0 0 dim(1)^2+dim(2)^2]*m/12;

% skew symmetric matrix from vector - test: skew(sym('a',[3,1],'real'))
% skew = @(r) [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
% parallel axes theorem
% translate_I = @(I_G,r_BG,m) I_G + m * skew(r_BG)'*skew(r_BG);
% rotate_I    = @(B_I_G,A_BC) A_BC' * B_I_G * A_BC;


syms m real
dim1 = [4,2,4]';
dim2 = [1,1,4]';
r_BG = [-1.5 1.5 0]';


I = Icub_G(dim1,m) + 2 * translate_I(Icub_G(dim2,m),r_BG,m)



function Sr = skew (r)
    % skew symmetric matrix from vector - test: skew(sym('a',[3,1],'real'))
    Sr = [    0 -r(3)  r(2); 
           r(3)    0  -r(1); 
          -r(2)  r(1)    0  ];
end

function C_I_G = rotate_I (B_I_G, A_BC) 
    % change of reference frame of the Inertia matrix
    C_I_G = A_BC' * B_I_G * A_BC;
end

function C_I_B = translate_I (C_I_G, C_r_BG, m) 
    % parallel axes theorem
    C_I_B = C_I_G + m * skew(C_r_BG)'*skew(C_r_BG);
end