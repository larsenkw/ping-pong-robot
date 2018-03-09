% twist2Transform:  Returns the homogeneous transformation matrix
% corresponding to a 6 element twist vector.
%
%   [H] = twist2Transform(t)  This function takes a set of twist parameters
%   in a single vector [v;w] (6x1) and calculates the homogeneous
%   tranformation matrix using the Angle Axis formulas.
%
%   H = The homogeneous transformation matrix (4x4)
%
%   t = The twist parameters [v;w] (6x1), v = (3x1) vector, w = Omega =
%   theta * k_hat (the Angle Axis parameters)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [H] = twist2Transform(t)
    % Extract twist parameters as vectors v and w
    v = t(1:3,1);
    w = t(4:6,1);
    
    % Calculate Angle Axis parameters
    theta = norm(w);
    if (theta ~= 0)
        k_hat = w/norm(w);
    end
    
    
    % Find rotation matrix
    if (theta ~= 0)
        R = angleAxis2Rot(k_hat, theta);
    else
        R = eye(3);
    end
    
    % Find displacement
    if (theta ~= 0)
        d = ((eye(3) - R)*cpMap(k_hat) + theta*(k_hat*k_hat'))*v;
    else
        d = v;
    end
    
    % Create full transformation matrix
    H = zeros(4,4);
    H(1:3,1:3) = R;
    H(1:3,4) = d;
    H(4,:) = [0 0 0 1];

end