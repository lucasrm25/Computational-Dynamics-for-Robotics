function Solution_22_RecursiveKinematics_RecursiveFunctionCall

    q = [0.1, -pi/4, 0.2, pi/4];    % joint coordinates
    q = [0.1, -pi/4, 0.2, -pi/4, 0.2, -pi/4, 0.2, pi/4, 0.2, pi/4, 0.2, pi/4];    % joint coordinates
    d = 0.15;   % length of the final link

    % number of rotational joints (points P_k)
    n = length(q)/2;

    % extend the coordinates vector q with the length (d) and relative
    % orientation (0) of the end-effector
    q = [q, d, 0];

    % position and orientation at the end-effector
    Pe = RecursiveJoint(q, n+1);
    disp(Pe);
end


function P = RecursiveJoint(q, k)
    if k == 0
        P = [0, 0, 0];  % origin
    else
        P_prev = RecursiveJoint(q, k-1);    % position and orientation at the previous joint
        P = [P_prev(1)-q(2*k-1)*sin(P_prev(3)), ...    % x-position of P
             P_prev(2)+q(2*k-1)*cos(P_prev(3)), ...    % y-position of P
             P_prev(3)+q(2*k)];                        % orientation at P
    end
end
