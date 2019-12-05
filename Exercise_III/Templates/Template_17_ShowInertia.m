% This function creates four different representations for the inertia of
% a rigid object. It shows:
%
% 1) The location of 6 points (each of mass m/6) that, in combination,
%    create an inertia of I and a COG at the position [0;0;0]. The points
%    are located in pairs, symmetrical with respect to the COG of the body.
%    Additionally, if lines are drawn between the points of each pair, they
%    form an orthogonal coordinate system.  In other words, the points are
%    centered on the faces of a cuboid.   
% 2) The location of 4 arbitrary points (each of mass m/4) that create an
%    inertia of I and a COG at the position [0;0;0].
% 3) The shape of an ellipsoid with homogenous density that has the inertia
%    I and is centered at the COG position [0;0;0].
% 4) The shape of a rectangular cuboid with homogenous density that has the
%    inertia I and is centered at the COG position [0;0;0]'. 
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   10/12/2018
%   v21
%
% *************************************************************************
% ToDo:                                                                  
% Rename file since for matlab class names and file names must be the same!
% *************************************************************************
function ShowInertia(I, m, Type)
    % Do an eigenvalue decomposition:
    [V,D] = eig(I);
    I1 = D(1,1);
    I2 = D(2,2);
    I3 = D(3,3);

    switch Type
        case 1 
        %  The location of 6 points (each of mass m/6) that, in combination,
        %  create an inertia of I and a COG at the position [0;0;0]. The points
        %  are located in pairs, symmatrical with respect to the COG of the body.
        %  Additionally, if lines are drawn between the points of each pair, they
        %  form an orthogonal coordinate system.  In other words, the points are
        %  centered on the faces of a cuboid. 
            % The 3x6 matrix 'pos' contains the position of the six points
            % in its columns:
% *************************************************************************
% ToDo:                                                                  
% Replace the '1's with the appropriate values:
% *************************************************************************
            pos = [-1,+1, 0, 0, 0, 0;
                    0, 0,-1,+1, 0, 0;
                    0, 0, 0, 0,-1,+1];
            pos = V*pos;
            % Show six points:
            ShowRedPoint(pos(:,1));
            ShowRedPoint(pos(:,2));
            ShowRedPoint(pos(:,3));
            ShowRedPoint(pos(:,4));
            ShowRedPoint(pos(:,5));
            ShowRedPoint(pos(:,6));
        case 2
        % The location of 4 arbitrary points (each of mass m/4) that create
        % an inertia of I and a COG at the position [0;0;0].     
            % The 3x4 matrix 'pos' contains the position of the four points
            % in its columns:
% *************************************************************************
% ToDo:                                                                  
% Replace the '1's with the appropriate values:
% *************************************************************************
            pos = [+1,-1, 0, 0;
                    0, 0,-1,+1;
                   -1,-1,+1,+1];
            pos = V*pos;
            % Show four points:
            ShowGreenPoint(pos(:,1));
            ShowGreenPoint(pos(:,2));
            ShowGreenPoint(pos(:,3));
            ShowGreenPoint(pos(:,4));
        case 3
        % The shape of an ellipsoid with homogenous density that has the
        % inertia I and is centered at the COG position [0;0;0].
            % Define the main axis of the ellipsoid:
% *************************************************************************
% ToDo:                                                                  
% Replace the '1's with the appropriate values:
% *************************************************************************
            a1 = V*[1;0;0];
            a2 = V*[0;1;0];
            a3 = V*[0;0;1];
            % Show an ellipsoid with main-axis a1, a2, and a3:
            ShowEllipsoid(a1, a2, a3);
        case 4
        % The shape of a rectangular cuboid with homogenous density that
        % has the inertia I and is centered at the COG position [0;0;0]. 
            % Define the eight corners of the cuboid
% *************************************************************************
% ToDo:                                                                  
% Replace the '1's with the appropriate values:
% *************************************************************************
            v = [-1,-1,-1,-1,+1,+1,+1,+1;
                 -1,+1,+1,-1,-1,+1,+1,-1;
                 -1,-1,+1,+1,-1,-1,+1,+1];
            v = V*v;
            % Show the cuboid:
            ShowCuboid(v);
    end
    
    
    % ADDITIONAL FUNCTIONS FOR GRAPHICAL DISPLAY
    % (No need to change anything here)
    % Shows a point at the coordinates pos_ = [x;y;z] in red
    function ShowRedPoint(pos_)
        [x_ y_ z_] = sphere;
        [f_, v_] = surf2patch(x_, y_, z_, z_);
        v_ = v_*0.05;
        v_ = v_ +repmat(pos_',size(v_,1),1);
        % Since in the graphical window the axis are switched, we need an
        % additional rotation:
        ROT_ = [0,0,1;1,0,0;0,1,0];
        v_ = (ROT_*v_')'; 
        patch('faces', f_, 'vertices', v_, 'FaceColor', [1;0;0],'EdgeColor', 'none');    
    end

    % Shows a point at the coordinates pos_ = [x;y;z] in green
    function ShowGreenPoint(pos_)
        [x_ y_ z_] = sphere;
        [f_, v_] = surf2patch(x_, y_, z_, z_);
        v_ = v_*0.05;
        v_ = v_ +repmat(pos_',size(v_,1),1);
        % Since in the graphical window the axis are switched, we need an
        % additional rotation:
        ROT_ = [0,0,1;1,0,0;0,1,0];
        v_ = (ROT_*v_')'; 
        patch('faces', f_, 'vertices', v_, 'FaceColor', [0;1;0],'EdgeColor', 'none');    
    end

    % Shows an ellipsoid with the main axis given by a1_, a2_, and a3_ in
    % blue: 
    function ShowEllipsoid(a1_, a2_, a3_)
        [x_ y_ z_] = sphere;
        [f_, v_] = surf2patch(x_, y_, z_, z_);
        R_ = [a1_,a2_,a3_]/2;
        v_ = (R_*v_')'; 
        % Since in the graphical window the axis are switched, we need an
        % additional rotation:
        ROT_ = [0,0,1;1,0,0;0,1,0];
        v_ = (ROT_*v_')'; 
        patch('faces', f_, 'vertices', v_, 'FaceColor', [0;0;1],'EdgeColor', 'none','FaceAlpha',0.5);    
    end

    % Shows a cuboid with eight corners being defined by the rows of the
    % 3x8 matrix v_ in grey.
    function ShowCuboid(v_)
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
        patch('faces', f_, 'vertices', v__, 'FaceColor', [0.5;0.5;0.5],'EdgeColor', 'none','FaceAlpha',0.5);    
    end
end