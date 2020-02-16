% *************************************************************************
%
% function struct = Vec2Struct(vec, names)
%
% Creates a MATLAB struct from a vector and a cell array of variable names.
%
% Input:  - 'vec', a one dimensional column vector that contains the
%           numeric information that will be placed in the struct.
%         - 'names', a string cell array that contains the names (and if
%           necessary) indexes to all elements in 'vec'
% Output: - A struct that contains doubles, or multidimensional (max number
%           of dimensions = 2) arrays of doubles. 
%
%  Example:
%    names = {'a'    'b(1,1)'    'b(1,2)'};
%    vec = [1; 2; 3];
%    struct = Vec2Struct(vec,names)
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
%   See also STRUCT2VEC. 
%
function struct = Vec2Struct(vec,names)
    
    % Check input values:
    if (length(vec) ~= length(names))
        error('GaitCreation:Vec2Struct:LengthMissmatch','Vector and name array must be of the same length');
    end
    % Start with an empty struct...
    struct = [];
    % ... and process items one by one:
    for i = 1:length(vec)
        % Use the eval command to map the struct back:
        eval(['struct.',names{i},' = vec(i);'])
    end
end
% *************************************************************************
% *************************************************************************