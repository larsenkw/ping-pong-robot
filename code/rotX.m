% rotX: Returns a rotation matrix describing a rotation about the X axis
% (theta in radians).
% 
%   [R] = rotX(theta)  This function takes a value theta in radians and
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

function [R] = rotX(theta)

    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];

end