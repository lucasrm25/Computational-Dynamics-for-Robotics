
q = [0.1, -pi/4, 0.2, pi/4];    % joint coordinates
q = [0.1, -pi/4, 0.2, -pi/4, 0.2, -pi/4, 0.2, pi/4, 0.2, pi/4, 0.2, pi/4];    % joint coordinates
d = 0.15;   % length of the final link

% number of rotational joints (points P_k)
n = length(q)/2;

% Initialize matrix P containing position of and orientation at each of the
% points P_k and the origin. The first row P(1,:)=[0,0,0] is the x
% position, y position, and orientation of the origin. The (k+1)-th row
% P(k+1,:) contains the x position, y position, and orientation at the k-th
% point P_k
P = zeros(n+1,3);

% iteratively find position and orientation at each point P_k based on
% the position and orientation at point P_k-1, the kth joint angle q(2*k), and the kth translation q(2*k-1)
for k = 1:n
% *************************************************************************
% ToDo:                                                                  
% Complete the code to iteratively compute the position/orientation of
% element k+1 to write a first recurive kinematics function: 
% *************************************************************************
    P(k+1,:) = ;
end

% position and orientation at the end-effector
% *************************************************************************
% ToDo:                                                                  
% Complete the code to update the position and orientation of the
% end-effector to better understand that there might be different types of
% joints/links in a kinematic tree: 
% *************************************************************************
Pe = ;
disp(Pe);

% plot the resulting robotic arm
figure; axis equal;
line([P(:,1);Pe(1)], [P(:,2);Pe(2)]);

