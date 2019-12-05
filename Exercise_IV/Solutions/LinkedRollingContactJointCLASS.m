% classdef LinkedRollingContactJointCLASS < handle
%
% Defines a rotational joint in a kinematic tree that is implemented as a
% set of linked objects.  
% 
% Public Methods:
%  joint = LinkedRollingContactJointCLASS(env, predBody, sucBody) 
%            Creates a rolling contact joint object that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'LinkedRigidBodyDynamicsCLASS', 
%            and function as predecessor and successor in a kinematic tree.
%            The joint is shown in the graphical environment 'env'.
%  joint.delete()         
%            Removes the joint from the graphics output and memory   
%  joint.recursiveOutput(spaceString)
%            This function recursively displays the object properties in
%            the matlab command window.  Then calls the 'recursiveOutput'
%            routine of the successor body. Display starts with an
%            identation of 'spaceString'.
%  sucBody = joint.getSuccessorBody()
%            Returns the successor of this joint. Can be used to write
%            recursive algorithms that simply jump over joints.
%  joint.updateGraphics() 
%            This function forces and update of the graphical  output. If
%            the property 'autoUpdate' is set to 'false', changes in the
%            properties are not directly reflected in the graphical output.
%            So calling this function every couple of integration steps
%            makes sure that graphics and numerical values are the same.
%  joint.recursiveForwardKinematics(B_r_IB, A_IB) 
%            Recursivley computes the position and orientation of the
%            successor body from the internally stored joint angle and the
%            provided values of predecessor body position and orientation.
%            Recursively calls the successor body to store this information
%            and pass it on.  
%  joint.recursiveGraphicsUpdate(obj)
%            Calls the internal 'updateGraphics()' function.  Then
%            recursively calls the child joints. 
%
% Public Properties:
%   P_r_PDp    % Position of the joint in the predecessor body
%   S_r_SDs    % Position of the joint in the successor body
%   A_PDp      % Orientation of the joint w.r.t the predecessor body
%   A_SDs      % Orientation of the joint w.r.t the successor body
%   q          % The joint angle.  
%   jointName  % A string with the name of the joint
%   autoUpdate % If this property is set to 'true', the graphical output
%                will be updated every time another public  property
%                changes.  If it is set to false, the user has to force the
%                graphical update, by calling the function
%                'joint.updateGraphics()'  
%   scale      % This value can be used to scale the size of the coordinate
%                systems for Dp and Ds. 
%   description % A string describing the joint
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v23
%
classdef LinkedRollingContactJointCLASS < handle
    % Public Properties
    properties
        P_r_PDp = [0;0;0];% Position of the joint in the predecessor body
        S_r_SDs = [0;0;0];% Position of the joint in the successor body
        A_PDp = eye(3);   % Orientation of the joint w.r.t the predecessor 
                          % body
        A_SDs = eye(3);   % Orientation of the joint w.r.t the successor 
                          % body
        q;                % The joint angle
        r;                % The radius of the rolling contact
        jointName  = '';  % Defaults to blank
        autoUpdate = 'true'  % In the default configuration, the 
                          % graphical output is updated every time a
                          % variable changes.  This is convenient, but can
                          % really slow down everything.
        scale = 1;        % To draw the coordinate systems of the joint smaller
        description = ''; % A string describing the joint
    end
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        % Objects for graphical visualization:
        env;       % Graphical environment
        Dp;        % A bound coordinate system, attached to the joint at 
                   % the predecessor
        Ds;        % A bound coordinate system, attached to the joint at
                   % the sucessor
        predBody;  % This is the predecessor of the joint in the kinematic
                   % tree
        sucBody;   % This is the successor of the joint in the kinematic
                   % tree
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedRollingContactJointCLASS(env, predBody, sucBody)
            % Store the information about the predecessor and successor of
            % this joint:
            obj.predBody = predBody;
            obj.sucBody = sucBody;
            % Now link this body outward and inward, by registering it
            % with the parent and child bodies:
            sucBody.setParentJoint(obj);
            predBody.addChildJoint(obj);
            % Create a graphical representation:
            obj.env = env;
            obj.Dp        = BoundCoSysCLASS(env, predBody.A_IB*obj.A_PDp,  predBody.A_IB*(predBody.B_r_IB + obj.P_r_PDp));
            obj.Dp.color  = [1 0 0];
            obj.Dp.name   = [obj.jointName,' DP'];
            obj.Dp.scale  = obj.scale;
            obj.Ds        = BoundCoSysCLASS(env, sucBody.A_IB*obj.A_SDs, sucBody.A_IB*(sucBody.B_r_IB + obj.S_r_SDs));
            obj.Ds.color  = [0 1 0];
            obj.Ds.name   = [obj.jointName,' DS'];
            obj.Ds.scale  = obj.scale;
        end
        % Remove everything from the graphics window upon deletion
        function delete(obj)
            delete(obj.Dp);
            delete(obj.Ds);
        end
        % This function forces an update of the graphical output. If the
        % property 'autoUpdate' is set to 'false', changes in the
        % properties are not directly reflected in the graphical output.
        % So calling this function every couple of integration steps makes
        % sure that graphics and numerical values are the same. 
        function updateGraphics(obj)
            update(obj)
        end
        function recursiveOutput(obj, spaceString)
            disp([spaceString,'###### BEGIN JOINT #####']);
            % Display some information about this joint:
            disp([spaceString, '" ',obj.description,' "']);
            % recursively call the successor body and add three spaces so
            % we get a nice indentation: 
            obj.sucBody.recursiveOutput([spaceString,'   ']);
            disp([spaceString,'###### END JOINT #######']);
        end
        % This function allows accessing the private property sucBody:
        function sucBody = getSuccessorBody(obj)
            sucBody = obj.sucBody;
        end
        function recursiveForwardKinematics(obj, P_r_IP, A_IP)
            % Rotation and displacement about the joint:
            [Dp_r_DpDs, A_DpDs] = obj.JointFunction(obj.q);
            
            % Compute the position, velocity, and acceleration of each
            % successing coordinate system:
            A_IDp           = A_IP * obj.A_PDp;
            Dp_r_IDp        = obj.A_PDp' * (P_r_IP + obj.P_r_PDp);
            
            A_IDs           = A_IDp * A_DpDs;
            Ds_r_IDs        = A_DpDs' * (Dp_r_IDp + Dp_r_DpDs);
            
            A_IS            = A_IDs * obj.A_SDs';
            S_r_IS          = obj.A_SDs * Ds_r_IDs - obj.S_r_SDs;
            
            % Pass this information on to the successor body:
            obj.sucBody.recursiveForwardKinematics(S_r_IS, A_IS);
        end
        function recursiveGraphicsUpdate(obj)
            % calls the 'updateGraphics()' function to force an update of
            % all graphics objects.  Then recursively calls the successor
            % body. 
            obj.updateGraphics();
            obj.sucBody.recursiveGraphicsUpdate();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
        function set.P_r_PDp(obj, P_r_PDp)
            obj.P_r_PDp = P_r_PDp;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.S_r_SDs(obj, S_r_SDs)
            obj.S_r_SDs = S_r_SDs;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.A_PDp(obj, A_PDp)
            obj.A_PDp = A_PDp;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.A_SDs(obj, A_SDs)
            obj.A_SDs = A_SDs;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.q(obj, q)
            obj.q = q;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.jointName(obj, jointName)
            obj.jointName = jointName;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.scale(obj, scale)
            obj.scale = scale;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
    end
    % Private Methods
    methods (Access = private)
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj, q)
            gamma = q;
            Dp_r_DpDs = [-obj.r*gamma;0;0];
            A_DpDs    = [+cos(gamma), -sin(gamma), 0;
                         +sin(gamma), +cos(gamma), 0;
                         +0         , +0         , 1];
        end
        % Update the graphic objects, if a value has changed
        function update(obj)
            obj.Dp.A_IC   = obj.predBody.A_IB*obj.A_PDp;
            obj.Dp.I_r_IC = obj.predBody.A_IB*(obj.predBody.B_r_IB + obj.P_r_PDp);
            obj.Dp.scale  = obj.scale;
            obj.Ds.A_IC   = obj.sucBody.A_IB*obj.A_SDs;
            obj.Ds.I_r_IC = obj.sucBody.A_IB*(obj.sucBody.B_r_IB + obj.S_r_SDs);
            obj.Ds.scale  = obj.scale;
        end
    end  
end

