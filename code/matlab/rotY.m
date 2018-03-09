% rotY: Returns a rotation matrix describing a rotation about the Y axis
% (theta in radians).
% 
%   [R] = rotY(theta)  This function takes a value theta in radians and
%   calculates the corresponding three-dimensional rotation matrix.
%
%   R = the rotation matrix (3x3)
%
%   theta = the angle of rotation in radians (follow the right-hand rule)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [R] = rotY(theta)

    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

end