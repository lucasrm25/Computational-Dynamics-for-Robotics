% classdef LinkedJointObjectCLASS_v2 < handle
%
% Defines an abstract joint object that is part of a linked list of bodies
% and joints.    
% 
% Public Methods:
%  joint = LinkedJointObjectCLASS_v2(predBody, sucBody) 
%            Creates an abstract joint object that links the body
%            specified in 'predBody' with the body specified in 'sucBody'.
%            Both are objects of the type 'LinkedBodyObjectCLASS', 
%            and function as predecessor and successor in a kinematic tree.
%  joint.delete()         
%            Removes the joint from the graphics output and memory   
%  joint.recursiveOutput(spaceString)
%            This function recursively displays the object properties in
%            the matlab command window.  Then calls the 'recursiveOutput'
%            routine of the successor body. Display starts with an
%            identation of 'spaceString'.
%  sucBody = joint.getSuccessorBody()
%            Returns the successor of this joint. Can be used to write
%            recursive algorithms that simply jump over joints.
%
% Public Properties:
%   description % A string describing the joint
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v23
%
classdef LinkedJointObjectCLASS_v2 < handle
    % Public Properties
    properties
        description = ''; % A string describing the joint
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        predBody;  % This is the predecessor of the joint in the kinematic
                   % tree
        sucBody;   % This is the successor of the joint in the kinematic
                   % tree
    end
    % Public Methods
    methods
        % Constructor
        function obj = LinkedJointObjectCLASS_v2(predBody, sucBody)
            % Store the information about the predecessor and successor of
            % this joint:
            obj.predBody = predBody;
            obj.sucBody = sucBody;
            % Now link this body outward and inward, by registering it
            % with the parent and child bodies:
            sucBody.setParentJoint(obj);
            predBody.addChildJoint(obj);
        end
        % Destructor (empty)
        function delete(obj)
        end
        function recursiveOutput(obj, spaceString)
            disp([spaceString,'###### BEGIN JOINT #####']);
            % Display some information about this joint:
            disp([spaceString, '" ',obj.description,' "']);
            % recursively call the successor body and add three spaces so
            % we get a nice indentation: 
            obj.sucBody.recursiveOutput([spaceString,'   ']);
            disp([spaceString,'###### END JOINT #######']);
        end
        % This function allows accessing the private property sucBody:
        function sucBody = getSuccessorBody(obj)
            sucBody = obj.sucBody;
        end
    end
    % Protected Methods
    methods (Access = protected)
        % - none
    end  
end

