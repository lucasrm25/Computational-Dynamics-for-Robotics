function M = skew(w)
    % Generates a skew-symmetric matrix given a vector w
    % The initialization with zeros(3,3)*w, is done, such that this matrix
    % is symbolic if w is symbolic.
    M = zeros(3,3)*w;

    M(1,2) = -w(3);
    M(1,3) =  w(2);
    M(2,3) = -w(1);

    M(2,1) =  w(3);
    M(3,1) = -w(2);
    M(3,2) =  w(1);
end