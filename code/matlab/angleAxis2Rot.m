% angleAxis2Rot:  Returns the rotation matrix encoded by a rotation of
% theta radians about the unit vector k axis.
%
%   [R] = angleAxis2Rot(k, theta)  This function takes a 3x1 unit vector k
%   and generates a rotation matrix for rotating theta radians about k
%   using the Angle-Axis formula
%
%   R = the rotation matrix (3x3)
%
%   k     = the 3x1 unit vector that is the axis of rotation
%   theta = the angle in radians to rotate about axis k, following
%   the right-hand rule
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [R] = angleAxis2Rot(k, theta)

    k = (1/norm(k))*k;

    R = cos(theta)*eye(3) + sin(theta)*cpMap(k) + (1 - cos(theta))*(k*k');
    
end