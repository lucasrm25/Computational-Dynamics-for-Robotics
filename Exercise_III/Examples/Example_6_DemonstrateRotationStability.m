function Example_6_DemonstrateRotationStability
    close all
    % Some RGB values:
    black  = [0;0;0];
    red    = [1;0;0]; 
    yellow = [1;1;0]; 
    blue   = [0;0;1]; 

    % Create a graphical environment and show a body-fixed coordinate system
    Env = EnvironmentCLASS();
    C = CoSysCLASS(Env, eye(3));
    C.name = 'Body Fixed';
    C.color = black;
    
    % Let's define an object with a given mass and a diagonal inertia
    % matrix (i.e., our body-fixed cosys is aligned with the principal
    % axes):
    m=10;
    I11 = 1.5;
    I22 = 2;
    I33 = 2.5;
    B_I_G = diag([I11,I22,I33]);
    
    % Let's do a quick visualization of this object.  Here, shown as a
    % blue cuboid: 
    % compute the lenghts of the sides of the cuboid
    a = sqrt(6/m*(- I11 + I22 + I33));
    b = sqrt(6/m*(+ I11 - I22 + I33));
    c = sqrt(6/m*(+ I11 + I22 - I33));
    % Define the eight corners of the cuboid
    v = [-a/2,-a/2,-a/2,-a/2,+a/2,+a/2,+a/2,+a/2;
        -b/2,+b/2,+b/2,-b/2,-b/2,+b/2,+b/2,-b/2;
        -c/2,-c/2,+c/2,+c/2,-c/2,-c/2,+c/2,+c/2];
    % Show the cuboid:
    p = ShowCuboid(v, blue);
    disp('wait here');
    p.delete
 
    % Let's spin this object initially around one principal axis with unit
    % velocity: 
    B_omega_B = [1;0;0];
    
    % The resulting kinetic energy is:
    E = 0.5*B_omega_B.'*B_I_G*B_omega_B;
    
    % Both the kinetic energy and the norm of the angular momentum must
    % remain constant for all possible future B_omega_B. For a diagonal
    % inertia matrix, we get for the kinetic energy:
    % 
    % E = 0.5*I11*B_omega_1^2 + 0.5*I22*B_omega_2^2 + 0.5*I33*B_omega_3^2
    % 
    % And since B_L_i = Iii*B_omega_i -> B_omega_i = B_L_i/Iii, we can substitute:
    %
    % 1 = (L_1^2)/(2*I11*E) + (L_2^2)/(2*I22*E) + (L_3^2)/(2*I33*E)
    %
    % We can interprete this as an equation for an ellipsoid, with the
    % squared semi-main axes: 
    % 2*I11*E, 2*I22*E, 2*I33*E
    % The possible tips of all B_L_G must lie on this ellipsoid.
    %
    % Let's draw this ellipsoid in red:
    a = 2*sqrt(2*E*I11);
    b = 2*sqrt(2*E*I22);
    c = 2*sqrt(2*E*I33);
    ShowEllipsoid([a;0;0], [0;b;0], [0;0;c], red);
    Env.resetOutput();
    
    % Yet, since the angular momentum is constant (in the inertial frame),
    % it's magnitude has to be constant, and the tip of all vectors B_L_G
    % must also lie on a sphere with radius norm(L).
    % Let's show this sphere in yellow.
    L_norm = norm(B_I_G*B_omega_B);
    r = 2*L_norm;
    ShowEllipsoid([r;0;0], [0;r;0], [0;0;r], yellow);
    
    disp('done');
    

    % This function draws an ellipsoid with the main axes a1_, a2_, a3.  It
    % is drawn semi-transparent with the collor degfined in color_.
    % It returns a handle to the resulting patch object.
    function p_ = ShowEllipsoid(a1_, a2_, a3_, color_)
        [x_, y_, z_] = sphere(300);
        [f_, v_] = surf2patch(x_, y_, z_, z_);
        R_ = [a1_,a2_,a3_]/2;
        v_ = (R_*v_')';
        % Since in the graphical window the axis are switched, we need an
        % additional rotation:
        ROT_ = [0,0,1;1,0,0;0,1,0];
        v_ = (ROT_*v_')'; 
        p_ = patch('faces', f_, 'vertices', v_, 'FaceColor', color_,'EdgeColor', 'none','FaceAlpha',0.5);    
    end



    % Shows a semi-transparent cuboid with eight corners being defined by
    % the rows of the 3x8 matrix v_ in the color given color_.
    function p_ = ShowCuboid(v_, color_)
        %  Each corner exists three times, such that the normal vectors for
        %  each face are independent: 
         v__ = [v_(:,1),v_(:,1),v_(:,1),...
                v_(:,2),v_(:,2),v_(:,2),...
                v_(:,3),v_(:,3),v_(:,3),...
                v_(:,4),v_(:,4),v_(:,4),...
                v_(:,5),v_(:,5),v_(:,5),...
                v_(:,6),v_(:,6),v_(:,6),...
                v_(:,7),v_(:,7),v_(:,7),...
                v_(:,8),v_(:,8),v_(:,8)];
        % Define faces of a cuboid:
        f_ = [10,7,4,1;
              13,16,19,22;
              5,8,20,17;
              2,14,23,11;
              9,12,24,21;
              3,6,18,15]; 
        % Since in the graphical window the axis are switched, we need an
        % additional rotation:
        ROT_ = [0,0,1;1,0,0;0,1,0];
        v__ = (ROT_*v__)'; 
        p_ = patch('faces', f_, 'vertices', v__, 'FaceColor', color_,'EdgeColor', 'none','FaceAlpha',0.5);    
    end
end