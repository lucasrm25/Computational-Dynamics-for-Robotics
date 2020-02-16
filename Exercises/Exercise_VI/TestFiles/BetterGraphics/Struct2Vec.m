% *************************************************************************
%
% function [vec,names] = Struct2Vec(struct)
% function vec = Struct2Vec(struct, names)
%
% Converts the struct variable 'struct' into a one dimensional array
%
% Input:  - A struct that contains doubles, or multidimensional (max number
%           of dimensions = 2) arrays of doubles. 
%         OPTIONAL:
%         - a string cell array that contains the names (and if
%           necessary indexes) to all elements that should be included in
%           'vec'. This also ensures a proper ordering of the elements.  If
%           a name is given and the corresponding variable is not found in
%           the struct, an entry is created and set to 0. A warning is
%           issued.
%
% Output: - 'vec', a one dimensional column vector that contains the
%           numerical information
%         - 'names', a string cell array that contains the names (and if
%           necessary indexes) to all elements in 'vec'
%
%  Example:
%    struct.a = 1;
%    struct.b = [2, 3];
%    [vec, names] = Struct2Vec(struct)
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
%   See also VEC2STRUCT. 
%
function [vec,names] = Struct2Vec(struct, varargin)

    
    if nargin > 1 % a cell array of index names was provided
        names = varargin{1};
        % Keep track of index-names that were not found in the struct:
        notFoundIndex = [];
        vec = zeros(length(names),1);
        for i = 1:length(names)
            try % not sure if the value was set
                vec(i) = eval(['struct.',names{i},';']);
            catch ME % if not, set the corresponding entry to 0;
                if strcmp(ME.identifier,'MATLAB:nonExistentField');
                    notFoundIndex = [notFoundIndex, i];
                    vec(i) = 0;
                end
            end
        end
        % Issue a warning about the index-names that were not found in the
        % struct:
        if ~isempty(notFoundIndex)
            warning('GaitCreation:Struct2Vec:NameNotFound','No entries for the following names were found and set to 0:');
            a = warning('query','SharedFunctionsHybridDynamics:Struct2Vec:NameNotFound');
            if ~strcmp(a.state,'off')
                for i = 1: length(notFoundIndex)
                    disp(['  -> ''',char(names{notFoundIndex(i)}),'''']);
                end
            end
        end
    else % No index-names provided, use all variables from the struct: 
        fNames = fieldnames(struct);
        c = 0;
        vec = [];
        % cycle through all names:
        for i = 1:length(fNames)
            newEl = struct.(fNames{i});
            [m, n] = size(newEl);
            for j = 1:m
                for k = 1:n
                    c = c + 1;
                    vec (c) = newEl(j,k);
                    if m*n == 1 % scalar:
                        names{c} = fNames{i};
                    else % Vector/Matrix:
                        names{c} = [fNames{i},'(',num2str(j),',',num2str(k),')'];
                    end
                end
            end
        end
        vec = vec';
    end
end
% *************************************************************************
% *************************************************************************
