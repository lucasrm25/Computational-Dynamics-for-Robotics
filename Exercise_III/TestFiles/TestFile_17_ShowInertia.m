clear all
close all
clc
Env = EnvironmentCLASS();
CoSysCLASS(Env,eye(3)); 
m = 10;
% I = [ 1.0, 0.0, 0.0;
%       0.0, 1.0, 0.0;
%       0.0, 0.0, 1.0];
% I = [ 1.5, 0.0, 0.0;
%       0.0, 2.0, 0.0;
%       0.0, 0.0, 2.5];
 I = [ 1.56,-0.16, 0.23;
     -0.16, 1.95,-0.50;
      0.23,-0.50, 2.50];
% I = [ 1.0, 0.0, 0.0;
%       0.0, 1.0, 0.0;
%       0.0, 0.0, 3.0];
%   
 
ShowInertia(I,m,1); % Six points
ShowInertia(I,m,2); % Four points
ShowInertia(I,m,3); % Ellipsoid
ShowInertia(I,m,4); % Cuboid

print(gcf,'-r600','-djpeg','Problem_17_Output.jpg','-opengl');