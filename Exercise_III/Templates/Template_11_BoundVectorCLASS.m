% BoundVectorCLASS < handle
%
% Defines a vector that is bound to a point O, which is given by the vector
% C_r_CO in relation to a given coordinate system C
% 
% Methods:
%  v = BoundVectorCLASS(env, C, C_v, C_r_CO)
%                          Creates a vector with the components 'C_v' (a
%                          numerical 3x1 column-vector) and the offset of
%                          origin 'C_r_CO', both given in the coordinate
%                          system 'C' (of the type BoundCoSysClass).
%  v.delete()              Removes the vector from the graphics output
%                          and the memory 
%  v.setCoords(C, C_v, C_r_CO)
%                          Sets the coordinates to new values defined in
%                          components 'C_v' and 'C_r_CO' of the coordinate
%                          system 'C' (of the type BoundCoSysClass).
%  C_v = v.getCoords(C)    Gets the components of this vector in
%                          coordinates of the CoSys 'C'
% 
% Properties:
%  name   % A string with the name of the vector
%  color  % A 3-vector of RGB values (between 0 and 1) defines the color of
%           this coordinate system in the graphical representation
%  scale  % A scalar value that allows to scale the vector-thickness by a
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
classdef BoundVectorCLASS < handle
    % Public Properties
    properties
        name  = '';
        color = [0;0;0];
        scale = 1.0;
    end
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        ENV_v;  % Vector coordinates in the environment-frame
        ENV_r;  % Coordinates of the origin in the environment-frame
        patchHandle;
        env;
        labelText;
    end
    % Public Methods
    methods
        % Constructor creates the graphical objects that represent a vector
        function obj = BoundVectorCLASS(env, C, C_v, C_r_CO)
            % Store the graphical environment
            obj.env = env;
            % Store the vector in inertial (env) coordinates
% *************************************************************************
% ToDo:                                                                  
% Complete constructor to learn how objects are created and initialized in
% Matlab. Implement the necessary coordinate transformations to store the
% vector and the position of its starting point with respect to the
% inertial (env) frame:
% *************************************************************************
            obj.ENV_v = ;
            obj.ENV_r = ;
            
            % Create a patch object that contains the graphics.
            [f,v] = createGraphicsData(obj);
            obj.patchHandle = patch('faces', f, 'vertices', v, 'FaceColor', obj.color,'EdgeColor', 'none');
            % Add name label:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            pos = ROT*obj.ENV_v;
            pos = ROT*obj.ENV_r + pos + pos/norm(pos)*0.15;
            obj.labelText = text(pos(1),pos(2),pos(3),obj.name);
        end
        % Destructor removes graphics upon deletion
        function delete(obj)
            delete(obj.patchHandle);
            delete(obj.labelText);
        end
        % Update the coordinates of this vector in the provided cosys
        function setCoords(obj, C, C_v, C_r_CO)
% *************************************************************************
% ToDo:                                                                  
% Complete the code for the set function below to learn how to access
% properties within an object. This should be similar to code in
% constructor:
% *************************************************************************
            obj.ENV_v = ;
            obj.ENV_r = ;
            updateGraphics(obj);
        end
        % Get the coordinates of this vector in the provided cosys
        function C_v = getCoords(obj, C)
            C_v = C.A_IC'*obj.ENV_v;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'updateGraphics' function afterwards, to
        % make sure that the graphical output is updated accordingly:
        function set.color(obj, color)
            obj.color = color;
            updateGraphics(obj);
        end
        function set.name(obj, name)
            obj.name = name;
            updateGraphics(obj);
        end
        function set.scale(obj, scale)
            obj.scale = scale;
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
            % Add name label:
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            pos = ROT*obj.ENV_v;
            pos = ROT*obj.ENV_r + pos + pos/norm(pos)*0.15;
            set(obj.labelText,'position',pos,'String',obj.name,'Color', obj.color);
        end
        % Create the faces and vertices that desctribe a vector
        function [f,v] = createGraphicsData(obj)
            N = 4;%20;
            vec = obj.ENV_v;
            % Create a unit vector in z-direction:
            [x,y,z] = cylinder([0.02*obj.scale,0.02*obj.scale],N);
            % scale:
            abs_vec = norm(vec);
            z = z*abs_vec;
            [f,v] = surf2patch(x,y,z,z);
            [x,y,z] = cylinder([0.4*obj.scale,0],N);
            [f1,v1] = surf2patch(x,y,z,z);
            v1=v1*0.1;
            v1 = v1 + repmat([0,0,abs_vec],size(v1,1),1);
            [v,f]=addPatches(v,f,v1,f1);
            
            % figure out which transformation to apply to rotate this vector into
            % v:
            vec1 = vec/abs_vec;
            k = cross([0;0;1],vec1);
            if norm(k)~=0
                % Rodrigues's formula:
                costheta = dot([0;0;1],vec1);
                R =[ 0    -k(3)  k(2);
                    k(3)  0    -k(1);
                    -k(2)  k(1)  0];
                R = costheta*eye(3) + R + k*k'*(1-costheta)/sum(k.^2);
            else
                if vec1(3)>0
                    R = eye(3);
                else
                    R = -eye(3);
                end
            end
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            v = transformVertices(v,ROT*R,ROT*obj.ENV_r);
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