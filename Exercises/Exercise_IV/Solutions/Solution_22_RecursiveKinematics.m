
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
    P(k+1,:) = [P(k,1)-q(2*k-1)*sin(P(k,3)), ...    % x-position of P_k
                P(k,2)+q(2*k-1)*cos(P(k,3)), ...    % y-position of P_k
                P(k,3)+q(2*k)];                     % orientation at P_k
end

% position and orientation at the end-effector
Pe = [P(end,1)-d*sin(P(end,3)), ...
      P(end,2)+d*cos(P(end,3)), ...
      P(end,3)];
disp(Pe);
  

% plot the resulting robotic arm
figure; axis equal;
line([P(:,1);Pe(1)], [P(:,2);Pe(2)]);

