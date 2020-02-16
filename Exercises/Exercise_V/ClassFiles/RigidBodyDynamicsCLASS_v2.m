% RigidBodyDynamicsCLASS_v2 < RigidBodyKinematicsCLASS_v2
%
% Defines a rigid body that is represented by all its
% kinematic and dynamic properties.
% 
% Public Methods:
%  B = RigidBodyDynamicsCLASS_v2(env) 
%            Creates a body in the graphical environment 'env'. The body's
%            initial position, velocity, and acceleration are set to
%            standard values.  
%  B.delete()         
%            Removes the body from the graphics output and memory   
%  B.computeNaturalDynamics() 
%            This function computes the accelerations within the rigid body
%            that result without any external forces or moments acting on
%            the body. 
%  + All methods inherited from RigidBodyKinematicsCLASS_v2
% 
% Public Properties:
%  Mass and inertia:
%     m_B            % The mass of the body [Kg]
%     B_I_B          % The inertia of the body [Kg*m^2]
%  Kinematic properties:
%     - inherited from RigidBodyKinematicsCLASS_v2
%  Other properties:
%     - inherited from RigidBodyKinematicsCLASS_v2
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef RigidBodyDynamicsCLASS_v2 < RigidBodyKinematicsCLASS_v2
    % Public Properties
    properties
        % Mass and inertia:
        m_B           = 1;       % The mass of the body [Kg]
        B_I_B         = eye(3);  % The inertia of the body [Kg*m^2]
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % Additional objects for graphical visualization:
        v_L;         % Bound vector of angular momentum
        ellipsoidPatch; % To visualize the inertia
    end
    % Public Methods
    methods
        % Constructor creates the graphical objects
        function obj = RigidBodyDynamicsCLASS_v2(env)
            % Superclass constructors must be called explicitly, as we need
            % to decide which arguments we pass to each constructor.
            %
            % Invoke superclass constructor to establish kinematic
            % properties: 
            obj = obj@RigidBodyKinematicsCLASS_v2(env);
            obj.v_L               = BoundVectorCLASS(env, obj.B, [0;0;0], [0;0;0]);
            obj.v_L.color         = [0.2;0.2;0.2];
            obj.v_L.name          = 'L_B';
            [v,f] = getEllipsoid([1;0;0],[0;1;0],[0;0;1]);
            obj.ellipsoidPatch = patch('faces', f, 'vertices', v, 'FaceColor', [0;0;1],'EdgeColor', 'none','FaceAlpha',0.2);
        end
        % Destructor removes all graphical objects upon deletion
        function delete(obj)
            % Superclass desctructor is called automatically
            delete(obj.v_L);
            delete(obj.ellipsoidPatch);
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
            obj.B_a_B         = B_pDot./obj.m_B;
            obj.B_omegaDot_B  = obj.B_I_B \ (B_LDot_B - B_omega_IB*B_L_B);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%% SET FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set functions are called whenever a public parameter is changed.
        % All of them call the 'update' function afterwards, to make sure
        % that the graphical output is updated accordingly:
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
	end
    % Protected Methods
    methods (Access = protected)
        % Update the graphic objects, if a value has changed
        function update(obj)
            update@RigidBodyKinematicsCLASS_v2(obj);
            % Compute the angular momentum in body coordinates:
            B_L_B = obj.B_I_B * obj.B_omega_B;
            obj.v_L.setCoords(obj.B, B_L_B, [0;0;0]);
            % Compute the inertia axis:
            [V,D] = eig(obj.B_I_B);
            I1 = D(1,1);
            I2 = D(2,2);
            I3 = D(3,3);
            % Define the main axis of the ellipsoid:
            a = sqrt(2.5/obj.m_B*(- I1 + I2 + I3));
            b = sqrt(2.5/obj.m_B*(+ I1 - I2 + I3));
            c = sqrt(2.5/obj.m_B*(+ I1 + I2 - I3));
            a1 = obj.A_IB*V*[a;0;0];
            a2 = obj.A_IB*V*[0;b;0];
            a3 = obj.A_IB*V*[0;0;c];
            [v, ~] = getEllipsoid(a1,a2,a3);
            % Transform into graphical CoSys, since Matlab uses a convention in which Z points up:
            % Z -> 2-axis
            % Y -> 1-axis
            % X -> 3-axis
            ROT = [0,0,1;1,0,0;0,1,0];
            v = transformVertices(v, eye(3), ROT*obj.A_IB*obj.B_r_IB);
            set(obj.ellipsoidPatch,'vertices',v);
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
function [v_, f_] = getEllipsoid(a1_, a2_, a3_)
    [x_, y_, z_] = sphere;
    [f_, v_] = surf2patch(x_, y_, z_, z_);
    R_ = [a1_,a2_,a3_];
    v_ = (R_*v_')';
    % Since in the graphical window the axis are switched, we need an
    % additional rotation:
    ROT_ = [0,0,1;1,0,0;0,1,0];
    v_ = (ROT_*v_')';
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
