%% INITIALIZE:
clear all
close all
clc

% Define four body-objects:
ground = LinkedBodyObjectCLASS();
bodyA  = LinkedBodyObjectCLASS();
bodyB  = LinkedBodyObjectCLASS();
bodyC  = LinkedBodyObjectCLASS();

% Define three joint-object that connect these bodies:
joint1 = LinkedJointObjectCLASS(ground,bodyA);
joint2 = LinkedJointObjectCLASS(bodyA,bodyB);
joint3 = LinkedJointObjectCLASS(ground,bodyC);

% Set the date stored in each:
ground.description = 'This is ground.';
bodyA.description = 'This is Body A.';
bodyB.description = 'This is Body B.';
bodyC.description = 'This is Body C.';
joint1.description = 'This is Joint 1.';
joint2.description = 'This is Joint 2.';
joint3.description = 'This is Joint 3.';

% Recursively call the output routine of each body:
disp('Information about all the objects in the tree:')
disp(' ');
ground.recursiveOutput('');
