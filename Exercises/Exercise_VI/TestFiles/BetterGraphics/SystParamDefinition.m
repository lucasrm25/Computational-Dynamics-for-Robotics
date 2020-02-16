% *************************************************************************
% 
% function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
% function p = SystParamDefinition()
%
% This MATLAB function defines the physical system parameter vector 'p' for
% a passive dynamic biped in 2D.  Besides serving as initial configuration
% of the model, this file provides a definition of the individual
% components of the system parameter vector and an index struct that allows
% name-based access to its values. 
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial system parameters as the vector 'systParamVec' (or 'p')
%         - The corresponding parameter names in the cell array 'systParamNames' 
%         - The struct 'systParamIndices' that maps these names into indices  
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, JUMPSET, 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    % Physics:
    % Gravity is pointing to the right (simulating an inclination of 1 deg):
    delta = 1*pi/180;
    systParam.gx        = +1*sin(delta); % [g] gravity in horizontal direction
    systParam.gy        = -1*cos(delta); % [g] gravity in vertical direction
    % Parameter of the model
    systParam.l_0       = 1;     % [l_0] leg length
    systParam.m1        = 0.20;  % [m_0] mass of the main body
    systParam.m2        = 0.40;  % [m_0] mass of the legs
    systParam.l2x       = 0.5;   % [l_0] distance between hip joint and CoG of the legs (along the legs)
    systParam.l2y       = 0;     % [l_0] distance between hip joint and CoG of the legs (perpendicular to the legs)
    systParam.rFoot     = 0.5;   % [l_0] foot radius
    systParam.j2        = 0.002;     % [m_0*l_0^2] inertia of the legs
    
    [systParamVec, systParamNames] = Struct2Vec(systParam);
    systParamIndices = Vec2Struct(1:1:length(systParamVec),systParamNames);
end
