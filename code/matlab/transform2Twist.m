% transform2Twist:  Returns the 6 element twist vector corresponding to the
% provided homogeneous transformation matrix.
%
%   [t] = transform2Twist(H)  This function takes a homogeneous 
%   transformation matrix and calculates a set of twist parameters and 
%   returns them in a single vector [v;w] (6x1).  The calculations use the
%   Angle Axis functions to find the rotation parameters.
%
%   t = The twist parameters [v;w] (6x1), v = (3x1) vector, w = Omega =
%   theta * k_hat (the Angle Axis parameters)
%
%   H = The homogeneous transformation matrix (4x4)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [t] = transform2Twist(H)

    % Extract rotation matrix and find k_hat and theta
    R = H(1:3,1:3);
    [k_hat, theta] = rot2AngleAxis(R);
    
    % Extract displacement vector and find v
    d = H(1:3,4);
    if (theta == 0)
        v = d;
    else
        v = (((sin(theta))/(2*(1-cos(theta))))*eye(3) + (((2*(1-cos(theta)) - ...
        theta*sin(theta))/(2*theta*(1-cos(theta))))*(k_hat*k_hat')) - ...
        ((1/2)*(cpMap(k_hat))))*d;
    end
    
    % Calculate w
    w = theta*k_hat;
    
    % Return t
    t = [v;w];

end