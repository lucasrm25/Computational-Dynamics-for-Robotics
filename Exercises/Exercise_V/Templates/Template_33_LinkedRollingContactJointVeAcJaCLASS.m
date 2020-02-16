% classdef LinkedRollingContactJointVeAcJaCLASS < LinkedGenericJointVeAcJaCLASS
%
% Defines a rolling contact joint in a kinematic tree
% that is implemented as a set of linked objects.  
% This class allows the computation of Positions/Orientations, Velocities,
% Accelerations, and Jacobians.
% 
% Public Methods:
%  joint = LinkedRollingContactJointVeAcJaCLASS(env, predBody, sucBody) 
%            Creates a rotational joint object that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'LinkedRigidBodyDynamicsVeAcJaCLASS', 
%            and function as predecessor and successor in a kinematic tree.
%            The joint is shown in the graphical environment 'env'.
%  joint.delete()         
%            Removes the joint from the graphics output and memory   
%  + All methods inherited from LinkedGenericJointVeAcJaCLASS
%
% Public Properties:
%  All inherited from LinkedGenericJointVeAcJaCLASS
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
classdef LinkedRollingContactJointVeAcJaCLASS < LinkedGenericJointVeAcJaCLASS
    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        r = 0; % Rolling contact radius 
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedRollingContactJointVeAcJaCLASS(env, predBody, sucBody)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to create generic joint:
            obj = obj@LinkedGenericJointVeAcJaCLASS(env, predBody, sucBody);
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
            Dp_r_DpDs = [-obj.r*gamma;0;0];
            A_DpDs    = [+cos(gamma), -sin(gamma), 0;
                         +sin(gamma), +cos(gamma), 0;
                         +0         , +0         , 1];
        end
% *************************************************************************
% ToDo:                                                                  
% In the three functions below, complete the specific functions to compute
% velocities, accelerations, and Jacobians accross the joint, to learn how
% these concepts are implemented in Matlab for a rolling contact joint with
% radius 'r'. 
% *************************************************************************
        function [Dp_rDot_DpDs, Dp_omega_DpDs] = JointVelocity(obj, q, qDot)
            % Overwrite generic JointVelocity:
            Dp_rDot_DpDs  = ;
            Dp_omega_DpDs = ;
        end
        function [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = JointAcceleration(obj, q, qDot, qDDot)
            % Overwrite generic JointAcceleration:
            Dp_rDDot_DpDs    = ;
            Dp_omegaDot_DpDs = ;
        end
        function [S, R] = JointJacobian(obj, q, qIndex, nq)
            % Overwrite generic JointJacobian:
            S = zeros(3,nq);
            R = zeros(3,nq);
            S(:,qIndex) = ;
            R(:,qIndex) = ;
        end
    end  
end

