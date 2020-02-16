% LinkedRigidBodyDynamicsMfgCLASS < LinkedRigidBodyDynamicsVeAcJaCLASS
%
% Defines a rigid body that is represented by all its
% kinematic and dynamic properties and that is part of a linked list.
% This class allows the computation of Positions/Orientations, Velocities,
% Accelerations, and Jacobians, as well as the recursive computation of the
% terms in the equations of motion. 
% 
% Public Methods:
%  B = LinkedRigidBodyDynamicsMfgCLASS(env) 
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  It is not connected to any joints.  
%  B.delete()         
%            Removes the body from the graphics output and memory   
%  [M, f, g] = B.recursiveComputationOfMfg(obj)
%            This function computes the components of the equations of
%            motion by recursively summing over all bodies and applying the
%            projected Newton Euler equations.
%  + All methods inherited from LinkedRigidBodyDynamicsVeAcJaCLASS
%
% Public Properties:
%  
%     I_grav;        % Vector of gravitational acceleration, given in 
%                      inertial frame
%  + All properties inherited from LinkedRigidBodyDynamicsVeAcJaCLASS
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef LinkedRigidBodyDynamicsMfgCLASS < LinkedRigidBodyDynamicsVeAcJaCLASS
    % Public Properties
    properties
        I_grav        = [0;0;0]; % Vector of gravitational forces
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % All inherited
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedRigidBodyDynamicsMfgCLASS(env)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to create linked dynamic body:
            obj = obj@LinkedRigidBodyDynamicsVeAcJaCLASS(env);
        end
        % Destructor (empty)
        function delete(obj)
            % Superclass desctructor is called automatically
        end
        function [M, f, g] = recursiveComputationOfMfg(obj)
            % Compute the components for this body:
            M = obj.B_J_S' * obj.m_B   * obj.B_J_S + ...
                obj.B_J_R' * obj.B_I_B * obj.B_J_R;  
            f = - obj.B_J_S' * obj.m_B * obj.B_a_B - ...
                obj.B_J_R' * (obj.B_I_B * obj.B_omegaDot_B + skew(obj.B_omega_B) * obj.B_I_B * obj.B_omega_B);
            g = obj.B_J_S' * obj.A_IB' * obj.I_grav * obj.m_B + ...
                obj.B_J_R' * obj.A_IB' * [0; 0; 0] ;
            if obj.isLeaf == false
            % If this is no leaf, recursively call the successor bodies of
            % the child joints, and sum over the components provided by
            % them:   
                for i = 1:obj.nChildren
                    [M_part, f_part, g_part] = obj.childJoints{i}.getSuccessorBody().recursiveComputationOfMfg();
                    M = M + M_part;
                    f = f + f_part;
                    g = g + g_part;
                end
            end
            % The values of M, f, and g that are returned include the
            % contribution from the entire sub-tree
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
        function set.I_grav(obj, I_grav)
            obj.I_grav = I_grav;
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
