% LinkedRigidBodyDynamicsVeAcJaCLASS < LinkedRigidBodyDynamicsCLASS_v2
%
% Defines a rigid body that is represented by all its
% kinematic and dynamic properties and that is part of a linked list.
% This class allows the computation of Positions/Orientations, Velocities,
% Accelerations, and Jacobians.
% 
% Public Methods:
%  B = LinkedRigidBodyDynamicsVeAcJaCLASS(env) 
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  It is not connected to any joints.  
%  B.delete()         
%            Removes the body from the graphics output and memory   
%  B.recursiveForwardKinematicsVeAcJa(obj, B_r_IB, A_IB, B_omega_B, B_v_B,
%      B_omegaDot_B, B_a_B,  B_J_S, B_J_R)
%            This function recieves the bodies position, orientation,
%            velocities, accelerations, and Jacobians from the parent
%            joint, saves them, and passes them on to the child joints. 
%  + All methods inherited from LinkedRigidBodyDynamicsCLASS_v2
%
% Public Properties:
%  
%  Constraint Jacobians:
%     B_J_S;         % Translational Jacobian of the body
%     B_J_R;         % Rotational Jacobian of the body
%     nq;            % Number of generalized coordinates in the system
%                      (used only in ground (root of kinematic tree) to
%                       determine the dimensions of the Jacobians) 
%  + All properties inherited from LinkedRigidBodyDynamicsCLASS_v2
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
% *************************************************************************
% ToDo:                                                                  
% Rename file since for matlab class names and file names must be the same!
% *************************************************************************
classdef LinkedRigidBodyDynamicsVeAcJaCLASS < LinkedRigidBodyDynamicsCLASS_v2
    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        % Kinematic values:
        % All properties are given in body-fixed coordinates B and
        % intialized to standard values (as they're not set in the
        % constructor):
        B_J_S         = [];      % Translational Jacobian of the body
        B_J_R         = [];      % Rotational Jacobian of the body
        nq            = 0;       % Dimensions of the q-vector
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedRigidBodyDynamicsVeAcJaCLASS(env)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to create linked dynamic body:
            obj = obj@LinkedRigidBodyDynamicsCLASS_v2(env);
        end
        % Destructor (empty)
        function delete(obj)
            % Superclass desctructor is called automatically
        end
        function recursiveForwardKinematicsVeAcJa(obj, B_r_IB, A_IB, B_omega_B, B_v_B, B_omegaDot_B, B_a_B,  B_J_S, B_J_R)
            % Position and orientation, as well as velocities and
            % accelerations are given by the parent joint and passed in its
            % call of 'recursiveForwardKinematics' 
            if obj.isRoot
                % If this body is the root of the kinematic tree, all
                % values are set to zero (this allows us to call the
                % function for root without any arguments): 
                obj.A_IB          = eye(3);
                obj.B_omega_B     = zeros(3,1);
                obj.B_omegaDot_B  = zeros(3,1);
                obj.B_r_IB        = zeros(3,1);
                obj.B_v_B         = zeros(3,1);
                obj.B_a_B         = zeros(3,1);
                obj.B_J_S         = zeros(3,obj.nq);
                obj.B_J_R         = zeros(3,obj.nq);
            else
                % otherwise the provided values are stored in the variables
                % of this object.
                obj.A_IB          = A_IB;
                obj.B_omega_B     = B_omega_B;
                obj.B_omegaDot_B  = B_omegaDot_B;
                obj.B_r_IB        = B_r_IB;
                obj.B_v_B         = B_v_B;
                obj.B_a_B         = B_a_B;
                obj.B_J_S         = B_J_S;
                obj.B_J_R         = B_J_R;
            end
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    % *************************************************************************
                    % ToDo:                                                                  
                    % Complete the recursive call to the corresponding function in
                    % LinkedGenericJointVeAcJaCLASS to learn how this recursion is implemented
                    % in Matlab:
                    % *************************************************************************
                    obj.childJoints{i}.recursiveForwardKinematicsVeAcJa(B_r_IB, A_IB, B_omega_B, B_v_B, B_omegaDot_B, B_a_B,  B_J_S, B_J_R);
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
        function set.B_J_S(obj, B_J_S)
            obj.B_J_S = B_J_S;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_J_R(obj, B_J_R)
            obj.B_J_R = B_J_R;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.nq(obj, nq)
            obj.nq = nq;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
    end
    % Protected Methods
    methods (Access = protected)
        % -none
    end  
end

