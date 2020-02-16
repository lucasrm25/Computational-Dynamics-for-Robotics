% *************************************************************************
% 
% classdef PassiveWalker3DCLASS(p) < OutputCLASS
%
% Three dimensional graphics of a passive dynamic biped.
%
% The system is initialized with a vector of system parameters p.
%
%
% Properties: - 'offset'      Zero position of the stance leg
%             - 'stepLength'  Length of the ground patches used for
%                             stepping 
%             - 'stanceOuter' Flag that indicates that the outer leg pair
%                             is used as stance legs
% Methods:    - NONE
%
%
% Created by C. David Remy on 03/14/2011
% MATLAB 2010a
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
%   See also OUTPUTCLASS.
%
classdef PassiveWalker3DCLASS < OutputCLASS % Inhereting from abstract classes only works from MATLAB 2008 upward
    properties (SetAccess = 'private', GetAccess = 'private')
        fig; % The output window
        % Patch objects used in the 3D representation
        outerLegPatch;
        innerLegPatch;
        mainAxisPatch;
        weightPatchs;
        floorPatch;
        % Vertices to which the transformation will be applied
        outerLegVertices;
        innerLegVertices;
        mainAxisVertices;
        weightVertices;
        floorVertices;
        % System parameters:
        p
    end
	properties 
        offset; 
        stanceOuter;
        stepLength;
    end
    methods
        function obj = PassiveWalker3DCLASS(p)
            obj.p = p;
            obj.slowDown    = 1;      % Run this in realtime.
            obj.rate        = 0.04;   % with 25 fps
            obj.offset      = 0.5; 
            obj.stepLength  = 0.5;
            obj.stanceOuter = true;
            % Initialize the 3D graphics
            obj.fig = figure();
            % Set some window properties
            set(obj.fig,'Name','3D-Output of a passive dynamic biped');  % Window title
            set(obj.fig,'Color','k');         % Background color
            
            %% Create the 3D Objects:  
            
            % Outer Leg:
            % Load CAD data from STL-file
            [f, v] = rndread('CADOuterLegs.txt');
            v = v/160;         % Normalize to leg length
            v(:,3) = v(:,3)-1; % Move axis to origin
            p = patch('faces', f, 'vertices' ,v);
            set(p, 'EdgeColor','none');
            set(p, 'FaceLighting','phong');
            % Set material properties that resemble aluminum:
            set(p, 'FaceColor', [0.8 0.8 0.8]);
            set(p, 'AmbientStrength',0.5);
            set(p, 'DiffuseStrength',0.8);
            set(p, 'SpecularStrength',0.5);
            set(p, 'SpecularExponent',25);
            set(p, 'SpecularColorReflectance',.25);
            obj.outerLegPatch    = p;
            obj.outerLegVertices = v;
            % Inner Leg:
            % Load CAD data from STL-file
            [f, v] = rndread('CADInnerLegs.txt');
            v = v/160;         % Normalize to leg length
            v(:,3) = v(:,3)-1; % Move axis to origin
            p = patch('faces', f, 'vertices' ,v);
            set(p, 'EdgeColor','none');
            set(p, 'FaceLighting','phong');
            % Set material properties that resemble aluminum:
            set(p, 'FaceColor', [0.8 0.8 0.8]);
            set(p, 'AmbientStrength',0.5);
            set(p, 'DiffuseStrength',0.8);
            set(p, 'SpecularStrength',0.5);
            set(p, 'SpecularExponent',25);
            set(p, 'SpecularColorReflectance',.25);
            obj.innerLegPatch    = p;
            obj.innerLegVertices = v;
            % Main Axis:
            % Load CAD data from STL-file
            [f, v] = rndread('CADMainAxis.txt');
            v = v/160;         % Normalize to leg length
            v(:,3) = v(:,3)-1; % Move axis to origin
            p = patch('faces', f, 'vertices' ,v);
            set(p, 'EdgeColor','none');
            set(p, 'FaceLighting','phong');
            % Set material properties that resemble steel:
            set(p, 'FaceColor', [0.5 0.5 0.5]);
            set(p, 'AmbientStrength',0.3);
            set(p, 'DiffuseStrength',0.3);
            set(p, 'SpecularStrength',1);
            set(p, 'SpecularExponent',25);
            set(p, 'SpecularColorReflectance',.5);
            obj.mainAxisPatch    = p;
            obj.mainAxisVertices = v;
            % Weights:
            % Load CAD data from STL-file
            [f, v] = rndread('CADWeight.txt');
            v = v/160; %  Normalize to leg length
            for i=1:4
                p = patch('faces', f, 'vertices' ,v);
                set(p, 'EdgeColor','none');
                set(p, 'FaceLighting','phong');
                % Set material properties that resemble lead:
                set(p, 'FaceColor', [0.25 0.25 0.25]);
                set(p, 'AmbientStrength',0.8);
                set(p, 'DiffuseStrength',0.8);
                set(p, 'SpecularStrength',0);
                obj.weightPatchs(i) = p;
            end
            obj.weightVertices    = v;
            % Create the ground:
            % Define a basic ground patch with a normalized size of 2x4x1,
            % that contains 15 individual rectangles forming a step up, and
            % a step down.  
            xPosGround = [0,0,0,0, 0,0,1,1, 0,0,1,1, 0,0,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 2,2,2,2, 2,2,2,2];
            yPosGround = [1,3,3,1, 1,1,1,1, 1,3,3,1, 3,3,3,3, 0,1,1,0, 3,1,1,3, 3,4,4,3, 0,0,0,0, 0,1,1,0, 1,1,1,1, 3,3,3,3, 3,4,4,3, 4,4,4,4, 1,0,0,1, 4,3,3,4];
            zPosGround = [0,0,1,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,0,1,1, 0,0,1,1, 0,0,1,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,0,1,1, 0,0,1,1];
            % Repeat this pattern N times:
            N = 10;
            vX = repmat(xPosGround,1,N);
            vY = repmat(yPosGround,1,N);
            vZ = repmat(zPosGround,1,N);
            % And shift the x-component for every step:
            xOffsets = reshape(repmat(0:2:2*N-2,size(xPosGround,2),1),1,N*size(xPosGround,2));
            vX = vX + xOffsets;
            % The last four vertices form the ground plane:
            vX = [vX,min(vX),min(vX),max(vX),max(vX)];
            vY = [vY,min(vY),max(vY),max(vY),min(vY)];
            vZ = [vZ, 0,0,0,0];
            % Scale everything:
            % The steps are 0.02 leg lengths heigh, 1.5 leg lengths wide,
            % and spaced according to the variable stepLength
            v = [vX',vY',vZ']*diag([obj.stepLength,260/160/4,0.02]);
            % Define faces (each containing 4 successing vertices)
            f = reshape(1:size(v,1),4,size(v,1)/4)';
            % Define color
            c = repmat([0,0.25,1],size(v,1)/4,1);
            % The last row is the ground plance, color differently:
            c(end,:) = [1,1,1];
            p = patch('faces', f, 'vertices' ,v, 'FaceVertexCData', c);
            set(p, 'EdgeColor','none');
            set(p, 'FaceColor','flat');
            set(p, 'FaceLighting','phong');
            set(p, 'AmbientStrength',0.6);
            set(p, 'DiffuseStrength',0.6);
            set(p, 'SpecularStrength',1);
            set(p, 'SpecularExponent',25);
            set(p, 'SpecularColorReflectance',.5);
            obj.floorPatch    = p;
            obj.floorVertices = v;
            
            %% Set up view:
            axis off
            box off
            axis equal
            camproj('perspective');
            camtarget([obj.offset,0.5,0.5]);
            campos ([10,-3,4]);
            [~, ~, systParamIndices] = SystParamDefinition();
            camup (-[obj.p(systParamIndices.gx),0,obj.p(systParamIndices.gy)]);
            camva(15)
            %% Create illumination:
            for i = 2:3:7
                light('Position',[i 3 2 ],'Style','local','Color',[1,1,0.8]./3);   % local light at the position given in 'Position'
            end
            for i = 4:4:8
                light('Position',[i -2 0 ],'Style','local','Color',[1,1,0.8]./2);   % local light at the position given in 'Position'
            end
        end
        
        
        function obj = update(obj, y, ~, ~, ~)
            persistent contStateIndices
            if isempty(contStateIndices)
                [~, ~, contStateIndices] = ContStateDefinition();
            end
            [CoGs, ~, ~] = GraphicalKinematicsWrapper(y, obj.p);
            if obj.stanceOuter
                angleOuter = y(contStateIndices.gamma);
                angleInner = y(contStateIndices.gamma) + y(contStateIndices.alpha);
                nCOGouter = 2;
                nCOGinner = 3;
            else
                angleInner = y(contStateIndices.gamma);
                angleOuter = y(contStateIndices.gamma) + y(contStateIndices.alpha);
                nCOGinner = 2;
                nCOGouter = 3;
            end
            % Outer leg:
            v = TransformVertices(obj.outerLegVertices, [cos(angleOuter),0,-sin(angleOuter);0,1,0;sin(angleOuter),0,cos(angleOuter)], [obj.offset + CoGs(1,1),55/160,0 + CoGs(2,1)+0.02]);
            set(obj.outerLegPatch,'Vertices',v);
            % Inner leg:
            v = TransformVertices(obj.innerLegVertices, [cos(angleInner),0,-sin(angleInner);0,1,0;sin(angleInner),0,cos(angleInner)], [obj.offset + CoGs(1,1),55/160,0 + CoGs(2,1)+0.02]);
            set(obj.innerLegPatch,'Vertices',v);
            % Main axis:
            v = TransformVertices(obj.mainAxisVertices, eye(3), [obj.offset + CoGs(1,1),55/160,0 + CoGs(2,1)+0.02]);
            set(obj.mainAxisPatch,'Vertices',v);
            % Weights:
            v = TransformVertices(obj.weightVertices, [1,0,0;0,0,-1;0,1,0], [obj.offset + CoGs(1,nCOGouter),55/160,0 + CoGs(2,nCOGouter)+0.02]);
            set(obj.weightPatchs(1),'Vertices',v);
            v = TransformVertices(obj.weightVertices, [1,0,0;0,0,+1;0,1,0], [obj.offset + CoGs(1,nCOGouter),55/160+150/160,0 + CoGs(2,nCOGouter)+0.02]);
            set(obj.weightPatchs(2),'Vertices',v);
            v = TransformVertices(obj.weightVertices, [1,0,0;0,0,+1;0,1,0], [obj.offset + CoGs(1,nCOGinner),55/160+23/160,0 + CoGs(2,nCOGinner)+0.02]);
            set(obj.weightPatchs(3),'Vertices',v);
            v = TransformVertices(obj.weightVertices, [1,0,0;0,0,-1;0,1,0], [obj.offset + CoGs(1,nCOGinner),55/160+127/160,0 + CoGs(2,nCOGinner)+0.02]);
            set(obj.weightPatchs(4),'Vertices',v);
            % Ground:
            xPosGround = [0,0,0,0, 0,0,1,1, 0,0,1,1, 0,0,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 1,1,2,2, 2,2,2,2, 2,2,2,2];
            yPosGround = [1,3,3,1, 1,1,1,1, 1,3,3,1, 3,3,3,3, 0,1,1,0, 3,1,1,3, 3,4,4,3, 0,0,0,0, 0,1,1,0, 1,1,1,1, 3,3,3,3, 3,4,4,3, 4,4,4,4, 1,0,0,1, 4,3,3,4];
            zPosGround = [0,0,1,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,0,1,1, 0,0,1,1, 0,0,1,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,1,1,0, 1,1,1,1, 1,0,0,1, 0,0,1,1, 0,0,1,1];
            % Repeat this pattern N times:
            N = 10;
            vX = repmat(xPosGround,1,N);
            vY = repmat(yPosGround,1,N);
            vZ = repmat(zPosGround,1,N);
            % And shift the x-component for every step:
            xOffsets = reshape(repmat(0:2:2*N-2,size(xPosGround,2),1),1,N*size(xPosGround,2));
            vX = vX + xOffsets;
            % The last four vertices form the ground plane:
            vX = [vX,min(vX),min(vX),max(vX),max(vX)];
            vY = [vY,min(vY),max(vY),max(vY),min(vY)];
            vZ = [vZ, 0,0,0,0];
            % Scale everything:
            % The steps are 0.02 leg lengths heigh, 1.5 leg lengths wide,
            % and spaced according to the variable stepLength
            v = [vX',vY',vZ']*diag([obj.stepLength,260/160/4,0.02]);
            % Define faces (each containing 4 successing vertices)
            f = reshape(1:size(v,1),4,size(v,1)/4)';
            % Define color
            c = repmat([0,0.25,1],size(v,1)/4,1);
            % The last row is the ground plance, color differently:
            c(end,:) = [1,1,1];
            set(obj.floorPatch,'Faces',f);
            set(obj.floorPatch,'Vertices',v);
            set(obj.floorPatch,'FaceVertexCData',c);
            % Set camera:
            camtarget([CoGs(1,1)+obj.offset,0.5,0.5]);
            %campos ([CoGs(1,1)+obj.offset+5,-5,3]);
            campos ([5,-5,3]);
            [~, ~, systParamIndices] = SystParamDefinition();
            camup (-[obj.p(systParamIndices.gx),0,obj.p(systParamIndices.gy)]);
            camva(20)
            drawnow();
        end
    end
end