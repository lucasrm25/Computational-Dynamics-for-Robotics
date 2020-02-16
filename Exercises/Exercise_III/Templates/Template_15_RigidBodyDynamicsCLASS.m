% RigidBodyDynamicsCLASS < handle
%
% Defines a rigid body that is represented by all its
% kinematic and dynamic properties.
% 
% Public Methods:
%  B = RigidBodyDynamicsCLASS(env) 
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  
%  B.delete()         
%            Removes the body from the graphics output and memory   
%  B.integrationStep(delta_t) 
%            This function performs an internal integration step, i.e., all
%            velocities and positions are updated via Euler integration,
%            while the rotation matrix is updated via a matrix exponential.    
%  I_r_IQ = B.positionOfPoint(B_r_BQ)  
%            Returns the position of a body fixed point Q in inertial
%            coordinates.  The point is provided in B-coordinates
%  I_v_Q = B.velocityOfPoint(B_r_BQ)   
%            Returns the absolute velocity of a body fixed point Q in
%            inertial coordinates.  The point is provided in B-coordinates
%  I_a_Q = B.accelerationOfPoint(B_r_BQ) 
%            Returns the absolute acceleration of a body fixed point Q in
%            inertial coordinates.  The point is provided in B-coordinates
%  B.computeNaturalDynamics() 
%            This function computes the accelerations within the rigid body
%            that result without any external forces or moments acting on
%            the body. 
%  B.updateGraphics() 
%            This function forces an update of the graphical output. If
%            the property 'autoUpdate' is set to 'false', changes in the
%            properties are not directly reflected in the graphical output.
%            So calling this function every couple of integration steps
%            makes sure that graphics and numerical values are the same.
% 
% Public Properties:
%  Kinematic properties:
%  (All properties are given in body-fixed coordinates B)
%     A_IB           % The rotational orientation of the body B with
%                      respect to the inertial frame 
%     B_omega_B      % The absolute angular velocity [rad/s]
%     B_omegaDot_B   % The absolute angular acceleration [rad/s^2]
%     B_r_IB         % The displacement of the body's COG [m]
%     B_v_B          % The absolute velocity of the body's COG [m/s]
%     B_a_B          % The absolute acceleration of the body's COG [m/s^2]
%  Mass and inertia:
%     m_B            % The mass of the body [Kg]
%     B_I_B          % The inertia of the body [Kg*m^2]
%  Other properties:
%     color          % A 3-vector of RGB values (between 0 and 1) defines
%                      the color of the body in the graphical
%                      representation  
%     bodyName       % A string with the name of the body
%     scale          % This value can be used to scale the size of the
%                      coordinate system that represents the body.
%     autoUpdate     % If this property is set to 'true', the graphical
%                      output will be updated every time another public
%                      property changes.  If it is set to false, the user
%                      has to force the graphical update, by calling the
%                      function 'B.updateGraphics()'
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
classdef RigidBodyDynamicsCLASS < handle
    % Public Properties
    properties
        % Kinematic values:
        % All properties are given in body-fixed coordinates B and
        % intialized to standard values (as they're not set in the
        % constructor):
        A_IB          = eye(3);  % The rotational orientation of the body B  
                                 % with respect to the inertial frame
        B_omega_B     = [0;0;0]; % The absolute angular velocity [rad/s]
        B_omegaDot_B  = [0;0;0]; % The absolute angular acceleration 
                                 % [rad/s^2]
        B_r_IB        = [0;0;0]; % The displacement of the body's COG [m]
        B_v_B         = [0;0;0]; % The absolute velocity of the body's COG
                                 % [m/s]
        B_a_B         = [0;0;0]; % The absolute acceleration of the body's 
                                 % COG [m/s^2]
        % Mass and inertia:
        m_B           = 1;       % The mass of the body [Kg]
        B_I_B         = eye(3);  % The inertia of the body [Kg*m^2]
        % Other properties:
        color         = [0;0;0]; % Defaults to black
        bodyName      = '';      % Defaults to blank
        scale = 1;               % To draw the coordinate system of the 
                                 % body at a smaller scale.
        autoUpdate    = true     % In the default configuration, the 
                                 % graphical output is updated every time a
                                 % variable changes.  This is convenient,
                                 % but can really slow down everything.
    end
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        % Objects for graphical visualization:
        env;         % Graphical environment
        B;           % A bound coordinate system, attached to the body at 
                     % its COG
        v_omega;     % Bound vector of angular velocity
        v_omegaDot;  % Bound vector of angular acceleration 
        v_v;         % Bound vector of linear velocity
        v_a;         % Bound vector of linear acceleration
        v_screw;     % Bound vector of the screw axis
        v_L;         % Bound vector of angular momentum
        ellipsoidPatch; % To visualize the inertia
    end
    % Public Methods
    methods
        % Constructor creates the graphical objects
        function obj = RigidBodyDynamicsCLASS(env)
            obj.env               = env;
            obj.B                 = BoundCoSysCLASS(env, obj.A_IB, obj.A_IB*obj.B_r_IB);
            obj.B.color           = obj.color;
            obj.B.scale           = obj.scale;
            obj.B.name            = obj.bodyName;
            obj.v_omega           = BoundVectorCLASS(env, obj.B, obj.B_omega_B, [0;0;0]);
            obj.v_omega.color     = [1;1;0];
            obj.v_omega.name      = '\Omega_B';
            obj.v_omegaDot        = BoundVectorCLASS(env, obj.B, obj.B_omegaDot_B, [0;0;0]);
            obj.v_omegaDot.color  = [1;0;1];
            obj.v_omegaDot.name   = '\Omega_B-dot';
            obj.v_v               = BoundVectorCLASS(env, obj.B, obj.B_v_B, [0;0;0]);
            obj.v_v.color         = [1;0;0];
            obj.v_v.name          = 'v_B';
            obj.v_a               = BoundVectorCLASS(env, obj.B, obj.B_a_B, [0;0;0]);
            obj.v_a.color         = [0;0;1];
            obj.v_a.name          = 'a_B';
            obj.v_screw           = BoundVectorCLASS(env, obj.B, [0;0;0], [0;0;0]);
            obj.v_screw.color     = [0;1;1];
            obj.v_screw.name      = 'Screw-Axis';
            obj.v_L               = BoundVectorCLASS(env, obj.B, [0;0;0], [0;0;0]);
            obj.v_L.color         = [0.2;0.2;0.2];
            obj.v_L.name          = 'L_B';
            [v,f] = getEllipsoid([1;0;0],[0;1;0],[0;0;1]);
            obj.ellipsoidPatch = patch('faces', f, 'vertices', v, 'FaceColor', [0;0;1],'EdgeColor', 'none','FaceAlpha',0.2);
% *************************************************************************
% ToDo:                                                                  
% Add the code to initialize the vector of angular momentum obj.v_L with
% color grey ([0.2;0.2;0.2]) and name 'L_B' to learn about setting up
% objects in a constructor.
% *************************************************************************
        end
        % Destructor removes all graphical objects upon deletion
        function delete(obj)
            delete(obj.B);
            delete(obj.v_omega);
            delete(obj.v_omegaDot);
            delete(obj.v_v);
            delete(obj.v_a);
            delete(obj.v_L);
            delete(obj.v_screw);
            delete(obj.ellipsoidPatch);
        end
        % This function performs an internal integration step, i.e., all
        % velocities and positions are updated via Euler integration, while
        % the rotation matrix is updated via a matrix exponential.
        function integrationStep(obj, delta_t)
            % Using the M = skew(w) function which is
            % defined below, we compute the skew symmetric matrices of
            % omega_B in I and in B-coordinates: 
            B_omega_IB = skew(obj.B_omega_B);
            I_omega_IB = skew(obj.A_IB*obj.B_omega_B);
            
            % Doing one-step Euler forward integration for linear motion
            % while taking into account that we do so in a moving
            % coordinate system:  
            obj.B_r_IB     = obj.B_r_IB    + delta_t * (obj.B_v_B         - B_omega_IB*obj.B_r_IB);
            obj.B_v_B      = obj.B_v_B     + delta_t * (obj.B_a_B         - B_omega_IB*obj.B_v_B);
            % Using the matrix-exponential to compute A_IB exactly over the
            % course of one integration time-step.
            obj.A_IB        = expm(delta_t*I_omega_IB)*obj.A_IB;
            % Doing one-step Euler forward integration for angular
            % velocity:
            obj.B_omega_B  = obj.B_omega_B + delta_t * (obj.B_omegaDot_B - 0);
        end
        % The following functions compute the position, velocity, and
        % acceleration of an arbitrary point on the body, which is given by
        % the displacement 'B_r_BQ' with respect to the body fixed point
        % 'B'. All values are returned in inertial coordinates:
        function I_r_IQ = positionOfPoint(obj, B_r_BQ)
            I_r_IQ = obj.A_IB*(obj.B_r_IB + B_r_BQ);
        end
        function I_v_Q = velocityOfPoint(obj, B_r_BQ)
            B_omega_IB = skew(obj.B_omega_B);
            I_v_Q = obj.A_IB*(obj.B_v_B + B_omega_IB*B_r_BQ);
        end
        function I_a_Q = accelerationOfPoint(obj, B_r_BQ)
            B_omega_IB = skew(obj.B_omega_B);
            B_omegaDot_IB = skew(obj.B_omegaDot_B);
            I_a_Q = obj.A_IB*(obj.B_a_B + (B_omegaDot_IB + B_omega_IB^2)*B_r_BQ);
        end
        function computeNaturalDynamics(obj)
            % Since no external forces or moments are acting, the change of
            % angular momentum and linear moment is zero:
            B_pDot   = zeros(3,1);
            B_LDot_B = zeros(3,1);
            % Compute the current angular momentum and the skew symmetric
            % matrix of B_omega_B
            B_L_B = obj.B_I_B * obj.B_omega_B;
            B_omega_IB = skew(obj.B_omega_B);
            % Compute accelerations from the equations of motion of a rigid
            % body.  Note that instead of using inv(B_I_B), we're using the
            % matrix 'devision' '\' that Matlab implements ("...X = A\B is
            % the solution to the equation A*X = B..."):   
% *************************************************************************
% ToDo:                                                                  
% Complete the code to compute the linear and rotational acceleration of
% this body if it is not subject to external forces and moments to learn
% how to implement the Newton-Euler Equations of motion:  
% *************************************************************************
            obj.B_a_B         = ;
            obj.B_omegaDot_B  = ;
        end
        % This function forces an update of the graphical output. If the
        % property 'autoUpdate' is set to 'false', changes in the
        % properties are not directly reflected in the graphical output.
        % So calling this function every couple of integration steps makes
        % sure that graphics and numerical values are the same. 
        function updateGraphics(obj)
            update(obj)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
        function set.A_IB(obj, A_IB)
            obj.A_IB = A_IB;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_omega_B(obj, B_omega_B)
            obj.B_omega_B = B_omega_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_omegaDot_B(obj, B_omegaDot_B)
            obj.B_omegaDot_B = B_omegaDot_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_r_IB(obj, B_r_IB)
            obj.B_r_IB = B_r_IB;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_v_B(obj, B_v_B)
            obj.B_v_B = B_v_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_a_B(obj, B_a_B)
            obj.B_a_B = B_a_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.m_B(obj, m_B)
            obj.m_B = m_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.B_I_B(obj, B_I_B)
            obj.B_I_B = B_I_B;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.color(obj, color)
            obj.color = color;
            if (obj.autoUpdate == true)
                update(obj);
            end
        end
        function set.bodyName(obj, bodyName)
            obj.bodyName = bodyName;
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
        % Update the graphic objects, if a value has changed
        function update(obj)
            obj.B.color  = obj.color;
            obj.B.name   = obj.bodyName;
            obj.B.scale  = obj.scale;
            obj.B.A_IC   = obj.A_IB;
            obj.B.I_r_IC = obj.A_IB*obj.B_r_IB;
            obj.v_omega.setCoords(obj.B, obj.B_omega_B, [0;0;0]);
            obj.v_omegaDot.setCoords(obj.B, obj.B_omegaDot_B, [0;0;0]);
            obj.v_v.setCoords(obj.B, obj.B_v_B, [0;0;0]);
            obj.v_a.setCoords(obj.B, obj.B_a_B, [0;0;0]);
            % Compute the shortest possible vector to the screw axis:
            % Compute the skew-symmetric matrix:
            B_omega_IB = skew(obj.B_omega_B);
            % Compute a point on the axis of rotation:
            if norm(obj.B_omega_B)~=0
                B_r_BT = B_omega_IB * obj.B_v_B/(norm(obj.B_omega_B)^2);
                B_v_T  = obj.B_v_B + B_omega_IB * B_r_BT;
            else % no rotation -> no screw axis
                B_r_BT = [0;0;0];
                B_v_T  = [0;0;0];
            end
            obj.v_screw.setCoords(obj.B, B_v_T, B_r_BT);
            % Compute the angular momentum in body coordinates:
% *************************************************************************
% ToDo:                                                                  
% Add code to compute the angular momentum B_L_B.  Then call
% obj.v_L.setCoords(obj.B, B_L_B, [0;0;0]) to update the graphics.
% *************************************************************************
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function M = skew(w)
    % Generates a skew-symmetric matrix given a vector w
      M = zeros(3,3);
    
      M(1,2) = -w(3);
      M(1,3) =  w(2);
      M(2,3) = -w(1);

      M(2,1) =  w(3);
      M(3,1) = -w(2);
      M(3,2) =  w(1);
end 