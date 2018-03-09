% rotZ: Returns a rotation matrix describing a rotation about the Z axis
% (theta in radians).
% 
%   [R] = rotZ(theta)  This function takes a value theta in radians and
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

function [R] = rotZ(theta)

    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

end