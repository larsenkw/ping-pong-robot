% rot2AngleAxis:  Returns the unit vector k and rotation angle theta about 
% that vector given a rotation matrix.
%
%   [k, theta] = rot2AngleAxis(R)  This function takes a rotation matrix
%   and converts it into the corresponding Angle-Axis parameters consisting
%   of the axis of rotation, k, and the angle of rotation about that axis,
%   theta.
%
%   k     = the 3x1 unit vector that is the axis of rotation
%   theta = the angle in radians to rotate about axis k, following
%   the right-hand rule
%
%   R = the rotation matrix (3x3)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [k, theta] = rot2AngleAxis(R)

    cos_theta = (trace(R) - 1)/2;
    sin_theta = 0.5*sqrt((R(3,2) - R(2,3))^2 + (R(1,3) - R(3,1))^2 + (R(2,1) - R(1,2))^2);
    theta = atan2(sin_theta, cos_theta);
    
    if ((theta <= (0 + 0.00000000001) && theta >= (0 - 0.00000000001)) || ...
            (theta <= (pi + .00000000001) && theta >= (pi - .00000000001)))
        k1 = sqrt((R(1,1) + 1)/2);
        k2 = sign(R(1,2))*sqrt((R(2,2) + 1)/2);
        k3 = sign(R(1,3))*sqrt((R(3,3) + 1)/2);
        k = [k1;k2;k3];
        k = k/norm(k);
    else
        k = (1/(2*sin_theta))*[R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
        k = k/norm(k);
    end

end