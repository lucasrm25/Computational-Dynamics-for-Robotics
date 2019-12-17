% classdef LinkedGenericJointVeAcJaCLASS < LinkedGenericJointCLASS_v2
%
% Defines a generic joint (without any specific motion) in a kinematic tree
% that is implemented as a set of linked objects.  
% This class allows the computation of Positions/Orientations, Velocities,
% Accelerations, and Jacobians.
% 
% Public Methods:
%  joint = LinkedGenericJointVeAcJaCLASS(env, predBody, sucBody) 
%            Creates a generic joint object that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'LinkedRigidBodyDynamicsVeAcJaCLASS', 
%            and function as predecessor and successor in a kinematic tree.
%            The joint is shown in the graphical environment 'env'.
%  joint.delete()         
%            Removes the joint from the graphics output and memory   
%  joint.recursiveForwardKinematicsVeAcJa(obj, P_r_IP, A_IP, P_omega_P,
%      P_v_P, P_omega_dot_P, P_a_P, P_J_S, P_J_R) 
%            This function recieves the bodies position, orientation,
%            velocities, accelerations, and Jacobians from the parent
%            joint, saves them, and passes them on to the child joints. 
%  + All methods inherited from LinkedGenericJointCLASS_v2
%
% Public Properties:
%   qDot      % Vector of joint velocities.  
%   qDDot     % Vector of joint accelerations.
%   qIndex    % This vector of indices states where the joint variables are
%               located in the complete vector of generalized coordinates q
%               (So that the Jacobians are extended in the right position)
%  + All properties inherited from LinkedGenericJointCLASS_v2
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef LinkedGenericJointVeAcJaCLASS < LinkedGenericJointCLASS_v2
    % Public Properties
    properties
        qDot      % Vector of joint velocities.  
        qDDot     % Vector of joint accelerations.
        qIndex    % Indices of the joint variables in the vector q
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedGenericJointVeAcJaCLASS(env, predBody, sucBody)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to create linked generic joint:
            obj = obj@LinkedGenericJointCLASS_v2(env, predBody, sucBody);
        end
        % Destructor (empty)
        function delete(obj)
            % Superclass desctructor is called automatically
        end
        function recursiveForwardKinematicsVeAcJa(obj, P_r_IP, A_IP, P_omega_P, P_v_P, P_omega_dot_P, P_a_P, P_J_S, P_J_R)
            % Rotation and displacement about the joint:
            [Dp_r_DpDs, A_DpDs] = obj.JointFunction(obj.q);
             % Angular rate and translational velocity accross the joint:
            [Dp_rDot_DpDs, Dp_omega_DpDs] = obj.JointVelocity(obj.q, obj.qDot);
            % Angular andtranslational acceleration accross the joint:
            [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = obj.JointAcceleration(obj.q, obj.qDot, obj.qDDot);
            % Compute the matrices that define the velocity accross the
            % joint as a linear function of q_dot:  
            [S, R] = obj.JointJacobian(obj.q, obj.qIndex, size(P_J_S,2));
            
            % Compute the position, velocity, and acceleration of each
            % successing coordinate system:
            A_IDp           = A_IP * obj.A_PDp;
            Dp_r_IDp        = obj.A_PDp' * (P_r_IP + obj.P_r_PDp);
            Dp_omega_Dp     = obj.A_PDp' * (P_omega_P + 0);
            Dp_v_Dp         = obj.A_PDp' * (P_v_P + 0 + skew(P_omega_P)*obj.P_r_PDp);
            Dp_omegaDot_Dp  = obj.A_PDp' * (P_omega_dot_P + 0 + 0);
            Dp_a_Dp         = obj.A_PDp' * (P_a_P + 0 + 0 + (skew(P_omega_dot_P) + skew(P_omega_P)^2)*obj.P_r_PDp);
            
            A_IDs           = A_IDp * A_DpDs;
            Ds_r_IDs        = A_DpDs' * (Dp_r_IDp + Dp_r_DpDs);
            Ds_omega_Ds     = A_DpDs' * (Dp_omega_Dp + Dp_omega_DpDs);
            Ds_v_Ds         = A_DpDs' * (Dp_v_Dp + Dp_rDot_DpDs + skew(Dp_omega_Dp)*Dp_r_DpDs);
            Ds_omegaDot_Ds  = A_DpDs' * (Dp_omegaDot_Dp + Dp_omegaDot_DpDs + skew(Dp_omega_Dp)*Dp_omega_DpDs);
            Ds_a_Ds         = A_DpDs' * (Dp_a_Dp + Dp_rDDot_DpDs + 2*skew(Dp_omega_Dp)*Dp_rDot_DpDs + (skew(Dp_omegaDot_Dp) + skew(Dp_omega_Dp)^2)*Dp_r_DpDs);
            
            A_IS            = A_IDs * obj.A_SDs';
            S_r_IS          = obj.A_SDs * Ds_r_IDs - obj.S_r_SDs;
            S_omega_S       = obj.A_SDs * (Ds_omega_Ds + 0);
            S_v_S           = obj.A_SDs * (Ds_v_Ds + 0) - skew(S_omega_S)*obj.S_r_SDs;
            S_omegaDot_S    = obj.A_SDs * (Ds_omegaDot_Ds + 0 + 0);
            S_a_S           = obj.A_SDs * (Ds_a_Ds + 0 + 0) - (skew(S_omegaDot_S) + skew(S_omega_S)^2)*obj.S_r_SDs;
            
            % Compute the displacement and orientation of the successor body:
            % Compute the overall rotation first:
            A_PS = obj.A_PDp * A_DpDs * obj.A_SDs';
            A_SP = A_PS';
            % Compute the rotational Jacobian of the successor body:
            S_J_R = A_SP*(P_J_R + obj.A_PDp*R);
            % Compute the translational Jacobian of the successor body:
            S_J_S = A_SP*(P_J_S + obj.A_PDp*S + skew(obj.P_r_PDp + obj.A_PDp*Dp_r_DpDs)'*P_J_R)-skew(obj.S_r_SDs)'*S_J_R;

            % Pass this information on to the successor body:
            obj.sucBody.recursiveForwardKinematicsVeAcJa(S_r_IS, A_IS, S_omega_S, S_v_S, S_omegaDot_S, S_a_S, S_J_S, S_J_R);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
        function set.qDot(obj, qDot)
            obj.qDot = qDot;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.qDDot(obj, qDDot)
            obj.qDDot = qDDot;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.qIndex(obj, qIndex)
            obj.qIndex = qIndex;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
    end
    % Protected Methods
    methods (Access = protected)
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj, q)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from this CLASS.
            % So the functionality in this SuperCLASS is only a constant
            % function. 
            Dp_r_DpDs = [0;0;0];
            A_DpDs    = eye(3);
        end
        function [Dp_rDot_DpDs, Dp_omega_DpDs] = JointVelocity(obj, q, qDot)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from this CLASS.
            % So the functionality in this SuperCLASS is only a constant
            % (zero) function. 
            Dp_rDot_DpDs  = zeros(3,1);
            Dp_omega_DpDs = zeros(3,1);
        end
        function [Dp_rDDot_DpDs, Dp_omegaDot_DpDs] = JointAcceleration(obj, q, qDot, qDDot)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from this CLASS.
            % So the functionality in this SuperCLASS is only a constant
            % (zero) function. 
            Dp_rDDot_DpDs    = zeros(3,1);
            Dp_omegaDot_DpDs = zeros(3,1);
        end
        function [S, R] = JointJacobian(obj, q, qIndex, nq)
            % Since this is a generic joint, it wouldn't make sense to
            % implement a specific type of joint (e.g., rotational). This
            % will happen in all the classes that inherit from this CLASS.
            % So the functionality in this SuperCLASS is only a constant
            % (zero) function. 
            S = zeros(3,nq);
            R = zeros(3,nq);
        end
    end  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function M = skew(w)
    % Generates a skew-symmetric matrix given a vector w
    % The initialization with zeros(3,3)*w, is done, such that this matrix
    % is symbolic if w is symbolic.
    M = zeros(3,3)*w;
    
    M(1,2) = -w(3);
    M(1,3) =  w(2);
    M(2,3) = -w(1);
    
    M(2,1) =  w(3);
    M(3,1) = -w(2);
    M(3,2) =  w(1);
end 