% *************************************************************************
%
% function vTrans = TransformVertices(v, dirCosine, translation)
% 
% This function transforms the coordinates of the vertices given in 'v'.
% 'dirCosine' is a rotation 3 x 3 matrix about the origin of the coordinate
% system, 'translation' is a translational 3-vector which is applied
% subsequently. Both are applied to every element in 'v'.  'v' is a matrix
% containing vertices, as they are used in patch objects. The return value
% 'vTrans' contains the coordinates of the transformed vertices.
% 
%
% Input:  - A nx3 matrix of vertices 'v' stored in rows
%         - A 3x3 rotation/scale matrix 'dirCosine'
%         - A 1x3 translation vector
% 
% Output: - A nx3 matrix of transformed vertices.
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
%   See also PATCH. 
%
function vTrans = TransformVertices(v, dirCosine, translation)
    if isempty(v)
        vTrans = [];
        return
    end
    % rotation
    vTrans = (dirCosine*v')';
    % translation
    vTrans = vTrans + repmat(translation(:)', size(vTrans,1),1);
end
% *************************************************************************
% *************************************************************************
