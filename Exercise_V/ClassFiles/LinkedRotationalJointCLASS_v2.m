% classdef LinkedRotationalJointCLASS_v2 < LinkedGenericJointCLASS_v2
%
% Defines a rotational joint in a kinematic tree that is implemented as a
% set of linked objects.  
% 
% Public Methods:
%  joint = LinkedRotationalJointCLASS_v2(env, predBody, sucBody) 
%            Creates a rotational joint object that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'LinkedRigidBodyDynamicsCLASS_v2', 
%            and function as predecessor and successor in a kinematic tree.
%            The joint is shown in the graphical environment 'env'.
%  joint.delete()         
%            Removes the joint from the graphics output and memory   
%  + All methods inherited from LinkedGenericJointCLASS_v2
%
% Public Properties:
%  All inherited from LinkedGenericJointCLASS_v2
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef LinkedRotationalJointCLASS_v2 < LinkedGenericJointCLASS_v2
    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        % All inherited
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedRotationalJointCLASS_v2(env, predBody, sucBody)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to create generic joint:
            obj = obj@LinkedGenericJointCLASS_v2(env, predBody, sucBody);
        end
        % Destructor (empty)
        function delete(obj)
            % Superclass desctructor is called automatically
        end
    end
    % Protected Methods
    methods (Access = protected)
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj, q)
            % Overwrite generic JointFunction:
            gamma = q;
            Dp_r_DpDs = [0;0;0];
            A_DpDs    = [+cos(gamma), -sin(gamma), 0;
                         +sin(gamma), +cos(gamma), 0;
                         +0         , +0         , 1];
        end
    end  
end

