% BoundCoSysCLASS < handle
%
% Defines a coordinate system that is bound to a point C, which is given by
% the vector I_r_IC in relation to the origin of its environment graphics
% axis 
% 
% Methods:
%  C = BoundCoSysCLASS(env, A_IC, I_r_IC)) 
%                          Creates a coordinate system with transformation
%                          matrix 'A_IC' between itself and the inertial
%                          frame of reference in the environment 'env',
%                          which is located at a point C, given by the
%                          relative position vector I_r_IC    
%  C.delete()              Removes the BoundCoSys from the graphics output 
%                          and the memory 
% 
% Properties:
%  A_IC   % A 3x3 matrix defining the rotation between this coordinate
%           system and the inertial frame of reference
%  I_r_IC % A 3x1 column-vector, defining the relative position of inertial
%           coordinate system and this coordinate system.
%  name   % A string with the name of the coordinate system
%  color  % A 3-vector of RGB values (between 0 and 1) defines the color of
%           this coordinate system in the graphical representation
%  scale  % A scalar value that allows to scale the coordinate system by a
%           given factor for graphical purposes.
%
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
classdef BoundCoSysCLASS < handle
    % Public properties
    properties
        A_IC   = eye(3);
        I_r_IC = [0;0;0];
        name   = '';
        color  = [0;0;0];
        scale  = 1;
    end
    % Private properties
    properties (SetAccess = private, GetAccess = private)
        patchHandle;
        env;
        xLabelText;
        yLabelText;
        zLabelText;
    end
    % Public methods
    methods
        % Constructor creates the graphical objects that represent a cosys
        function obj = BoundCoSysCLASS(env, A_IC, I_r_IC)
            % Store the graphical environment
            obj.env = env;
            % Store the cosys trafo w.r.t to inertial (env) coordinates
            obj.A_IC   = A_IC;
            obj.I_r_IC = I_r_IC;
            % Create a patch object that contains the graphics.
            [f,v] = obj.createGraphicsData();
            obj.patchHandle = patch('faces', f, 'vertices', v, 'FaceColor', obj.color,'EdgeColor', 'none');
            % Add axis labels:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            pos = [1.15;0;0] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
            obj.xLabelText = text(pos(1),pos(2),pos(3),['1-axis (', obj.name,')']);
            pos = [0;1.15;0] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
            obj.yLabelText = text(pos(1),pos(2),pos(3),'2-axis');
            pos = [0;0;1.15] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
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
% *************************************************************************
% ToDo:                                                                  
% Complete the code below to learn how set functions work in Matlab:
% *************************************************************************
        function set.A_IC(obj, A_IC)
            obj.A_IC =  ;
            updateGraphics(obj);
        end
        function set.I_r_IC(obj, I_r_IC)
            obj.I_r_IC =  ;
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
        function set.scale(obj, scale)
            obj.scale = scale;
            updateGraphics(obj);
        end
    end
    % Private methods
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
            pos = [1.15;0;0] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
            set(obj.xLabelText,'position',pos,'Color', obj.color, 'String', ['1-axis (', obj.name,')']);
            pos = [0;1.15;0] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
            set(obj.yLabelText,'position',pos,'Color', obj.color);
            pos = [0;0;1.15] * obj.scale;
            pos = ROT*(obj.I_r_IC + obj.A_IC*pos);
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
            I_r_IC_ = ROT*obj.I_r_IC;
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
            v = transformVertices(v*obj.scale, A_IC_, I_r_IC_);
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