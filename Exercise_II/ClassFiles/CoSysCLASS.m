% CoSysCLASS < handle
%
% Defines a coordinate system that is shown at the origin of its
% environment graphics axis
% 
% Methods:
%  C = CoSysCLASS(env, A_IC) 
%                          Creates a coordinate system with transformation
%                          matrix 'A_IC' between itself and the inertial
%                          frame of reference in the environment 'env'.   
%  C.delete()              Removes the CoSys from the graphics output and
%                          the memory 
% 
% Properties:
%  A_IC   % A 3x3 matrix defining the rotation between this coordinate
%           system and the inertial frame of reference
%  name   % A string with the name of the coordinate system
%  color  % A 3-vector of RGB values (between 0 and 1) defines the color of
%           this coordinate system in the graphical representation
%
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   10/12/2018
%   v21
%
classdef CoSysCLASS < handle
    % Public Properties
    properties
        A_IC   = eye(3);
        name   = '';
        color  = [0;0;0];
    end
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        patchHandle;
        env;
        xLabelText;
        yLabelText;
        zLabelText;
    end
    % Public Methods
    methods
        % Constructor creates the graphical objects that represent a cosys
        function obj = CoSysCLASS(env, A_IC)
            % Store the graphical environment
            obj.env = env;
            % Store the cosys trafo w.r.t to inertial (env) coordinates
            obj.A_IC   = A_IC;
            % Create a patch object that contains the graphics.
            figure(obj.env.fig);
            [f,v] = obj.createGraphicsData();
            obj.patchHandle = patch('faces', f, 'vertices', v, 'FaceColor', obj.color,'EdgeColor', 'none');
            % Add axis labels:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            A_IC_ = ROT*obj.A_IC;
            pos = [1.15;0;0];
            pos = A_IC_*pos;
            obj.xLabelText = text(pos(1),pos(2),pos(3),['1-axis (', obj.name,')']);
            pos = [0;1.15;0];
            pos = A_IC_*pos;
            obj.yLabelText = text(pos(1),pos(2),pos(3),'2-axis');
            pos = [0;0;1.15];
            pos = A_IC_*pos;
            obj.zLabelText = text(pos(1),pos(2),pos(3),'3-axis');
        end
        % Destructor removes graphics upon deletion
        function delete(obj)
            delete(obj.patchHandle);
            delete(obj.xLabelText);
            delete(obj.yLabelText);
            delete(obj.zLabelText);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'updateGraphics' function afterwards, to
        % make sure that the graphical output is updated accordingly:
        function set.A_IC(obj, A_IC)
            obj.A_IC = A_IC;
            updateGraphics(obj);
        end
        function set.name(obj, name)
            obj.name = name;
            updateGraphics(obj);
        end
        function set.color(obj, color)
            obj.color = color;
            updateGraphics(obj);
        end
    end
    % Private Methods
    methods (Access = private)
        % Update the graphic objects, if a value has changed
        function updateGraphics(obj)
            [~,v] = createGraphicsData(obj);
            set(obj.patchHandle,'vertices',v);
            set(obj.patchHandle,'FaceColor',obj.color');
            % Add axis labels:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            A_IC_ = ROT*obj.A_IC;
            pos = [1.15;0;0];
            pos = A_IC_*pos;
            set(obj.xLabelText,'position',pos,'Color', obj.color, 'String', ['1-axis (', obj.name,')']);
            pos = [0;1.15;0];
            pos = A_IC_*pos;
            set(obj.yLabelText,'position',pos,'Color', obj.color);
            pos = [0;0;1.15];
            pos = A_IC_*pos;
            set(obj.zLabelText,'position',pos,'Color', obj.color);
        end
        % Create the faces and vertices that desctribe a cosys
        function [f,v] = createGraphicsData(obj)
            N = 4;%20;
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            A_IC_   = ROT*obj.A_IC;
            % Basic sphere at origin
            [x, y, z] = sphere;
            [f,v] = surf2patch(x,y,z,z);
            v = v*0.05;
            % X Axis
            [x,y,z] = cylinder([0.02,0.02],N);
            [f1,v1] = surf2patch(x,y,z,z);
            [v,f]=addPatches(v,f,v1,f1);
            [x,y,z] = cylinder([0.4,0],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=v1*0.1;
            v1 = v1 + repmat([0,0,1],size(v1,1),1);
            [v,f]=addPatches(v,f,v1,f1);
            % Y Axis
            [x,y,z] = cylinder([0.02,0.02],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=([1,0,0;0,0,1;0,1,0]*v1')';
            [v,f]=addPatches(v,f,v1,f1);
            [x,y,z] = cylinder([0.4,0],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=v1*0.1;
            v1 = v1 + repmat([0,0,1],size(v1,1),1);
            v1=([1,0,0;0,0,1;0,1,0]*v1')';
            [v,f]=addPatches(v,f,v1,f1);
            % Z Axis
            [x,y,z] = cylinder([0.02,0.02],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=([0,0,1;0,1,0;1,0,0]*v1')';
            [v,f]=addPatches(v,f,v1,f1);
            [x,y,z] = cylinder([0.4,0],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=v1*0.1;
            v1 = v1 + repmat([0,0,1],size(v1,1),1);
            v1=([0,0,1;0,1,0;1,0,0]*v1')';
            [v,f]=addPatches(v,f,v1,f1);
            % Transform to desired position:
            v = transformVertices(v,A_IC_,[0;0;0]);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v_, f_] = addPatches(v1_,f1_,v2_,f2_)
    % function [v, f] = addPatches(v1,f1,v2,f2)
    %
    % Assuming that v1, f1, v2, and f2 define vertices and faces of two patch
    % objects, this function returns the vertices and faces of a new combined
    % patch object.
    f2_ = f2_ + repmat(size(v1_,1),size(f2_));
    v_ = [v1_; v2_];
    f_ = [f1_; f2_];
end
function vTrans_ = transformVertices(v_,dirCosine_,translation_)
    % function vTrans = transformVertices(v,dirCosine,translation)
    %
    % This function transforms the coordinates of the vertices given in 'v'.
    % 'dirCosine' is a rotation 3 x 3 matrix, 'translation' is a translational
    % 3-vector. Both are applied to every element in 'v'.
    % 'v' is a matrix containing vertices, as they are used in patch objects.
    % The reutrn value vTrans contains the coordinates of the transformed
    % vertices.
    if isempty(v_)
        vTrans_ = [];
        return
    end
    % rotation
    vTrans_ = (dirCosine_*v_')';
    % translation
    vTrans_ = vTrans_ + repmat(translation_',size(vTrans_,1),1);
end