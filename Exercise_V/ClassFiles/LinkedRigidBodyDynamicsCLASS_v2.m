% LinkedRigidBodyDynamicsCLASS_v2 < RigidBodyDynamicsCLASS_v2 & LinkedBodyObjectCLASS_v2 
%
% Defines a rigid body that is represented by all its
% kinematic and dynamic properties and that is part of a linked list.
% 
% Public Methods:
%  B = LinkedRigidBodyDynamicsCLASS_v2(env) 
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  It is not connected to any joints.  
%  B.delete()         
%            Removes the body from the graphics output and memory   
%  B.recursiveForwardKinematics(obj, B_r_IB, A_IB)
%            This function recieves the bodies position and orientation
%            from the parent joint, saves it, and passes it on to the child
%            joints. 
%  B.recursiveGraphicsUpdate(obj)
%            Calls the internal 'update' function to force an update of all
%            graphics objects.  Then recursively calls the child joints.
%  + All methods inherited from RigidBodyDynamicsCLASS_v2 &
%        LinkedBodyObjectCLASS_v2 
% 
% Public Properties:
%  All inherited from RigidBodyDynamicsCLASS_v2 & LinkedBodyObjectCLASS_v2
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef LinkedRigidBodyDynamicsCLASS_v2 < RigidBodyDynamicsCLASS_v2 & LinkedBodyObjectCLASS_v2
    % Public Properties
    properties
        % All inherited
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor (calls superclass constructors)
        function obj = LinkedRigidBodyDynamicsCLASS_v2(env)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Call superclass constructors to create a rigid dynamic body
            % and a body object that is part of a linked list: 
            obj@RigidBodyDynamicsCLASS_v2(env);
            obj@LinkedBodyObjectCLASS_v2();
        end
        % Destructor (empty)
        function delete(obj)
            % Superclass desctructor is called automatically
        end
        function recursiveForwardKinematics(obj, B_r_IB, A_IB)
            % Position and orientation, as well as velocities and
            % accelerations are given by the parent joint and passed in its
            % call of 'recursiveForwardKinematics' 
            if obj.isRoot == false
                obj.A_IB          = A_IB;
                obj.B_r_IB        = B_r_IB;
            else
                obj.A_IB          = eye(3);
                obj.B_r_IB        = zeros(3,1);
            end
            
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveForwardKinematics(obj.B_r_IB, obj.A_IB);
                end
            end
        end
        function recursiveGraphicsUpdate(obj)
            % Calls the 'updateGraphics()' function to force an update of
            % all graphics objects.  Then recursively calls the child
            % joints. 
            obj.updateGraphics();
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveGraphicsUpdate();
                end
            end
        end
    end
    % Protected Methods
    methods (Access = protected)
        % -none
    end
end

