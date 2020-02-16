% LinkedBodyObjectCLASS_v2 < handle
%
% Defines an abstract body object that is part of a linked list of bodies
% and joints.  
% 
% Public Methods:
%  B = LinkedBodyObjectCLASS_v2() 
%            Creates a new body in a linked list. It is not connected to
%            any joints.   
%  B.delete()         
%            Removes the body from memory   
%  B.setParentJoint(parentJoint) 
%            This function sets the parent joint to the provided object
%            'parentJoint'. It also flags that the body is not the root
%            of a kinematic tree.
%  B.addChildJoint(childJoint) 
%            This function adds a new child joint which is given by the
%            provided object 'childJoint'.  If the first child joint is
%            added, the function also flags that the body is not a leaf in
%            a kinematic tree.
%  B.recursiveOutput(spaceString)
%            This function recursively displays the object properties in
%            the matlab command window.  Then calls the 'recursiveOutput'
%            routine of all child joints. Display starts with an
%            identation of 'spaceString'. 
% 
% Public Properties:
%     description    % A string describing the body
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   12/21/2018
%   v22
%
classdef LinkedBodyObjectCLASS_v2 < handle
    % Public Properties
    properties
        description = '';        % A string describing the body
    end
    % Protected Properties
    properties (SetAccess = protected, GetAccess = protected)
        % Objects for setting up the linked list:
        parentJoint;    % This joint connects the body with its predecessor
                        % in the kinematic tree (a JointCLASS object)
        childJoints;    % These joints connect the body with its successors 
                        % in the kinematic tree (a cell array of JointCLASS
                        % objects) 
        nChildren = 0;  % Number of child joints that connect with further 
                        % bodies in the tree
        isRoot = true;  % Is true if this body has no parent joint.
        isLeaf = true;  % Is true if this body has no child joints.
    end
    % Public Methods
    methods
        % Constructor (empty)
        function obj = LinkedBodyObjectCLASS()
        end
        % Destructor (empty)
        function delete(obj)
        end
        % Set the parents joint.  This function is called to link this body
        % with its predecessor in the kinematic tree via a parents joint.
        function setParentJoint(obj, parentJoint)
            obj.parentJoint = parentJoint;
            % If the body has a parent joint, it cannot be the root of the
            % tree:
            obj.isRoot = false;
        end
        % Add a child joint
        function addChildJoint(obj, childJoint)
            % Increase the number of child joints by one...
            obj.nChildren = obj.nChildren + 1;
            % ... and fill in the new place in the cell array
            obj.childJoints{obj.nChildren} = childJoint;
            % If the body has a child joint, it cannot be a leave of the
            % tree:
            obj.isLeaf = false;
        end
        function recursiveOutput(obj, spaceString)
            disp([spaceString,'****** BEGIN BODY ******']);
            % Display some information about this body:
            disp([spaceString, '" ',obj.description,' "']);
            if obj.isLeaf == false
                disp([spaceString, 'This body has ', num2str(obj.nChildren),' child joints.']);
            else
                disp ([spaceString, 'This body is a leaf']);
            end
            % If this is no leaf, recursively call the child joints and add
            % three spaces so we get a nice indentation: :
            if obj.isLeaf == false
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveOutput([spaceString,'   ']);
                end
            end
            disp([spaceString,'****** END BODY   ******']);
        end
	end
    % Protected Methods
    methods (Access = protected)
        % - none
    end
end

