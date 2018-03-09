% dhTransform: Returns the homogeneous transformation corresponding to the
% provided DH parameters for a link.
%
%   [H] = dhTransform(a, d, alpha, theta)   This function takes the DH
%   parameters for a single link and calculates the corresponding
%   homogeneous transformation matrix.  This assumes the standard DH
%   parameter convention where the joint origin is at the distal end of the
%   link.
%
%   H = The homogeneous transformation matrix (4x4)
%
%   a     = Translation from z_(i-1) to z_(i) along the x_(i) axis
%   d     = Translation from x_(i-1) to x_(i) along the z_(i-1) axis
%   alpha = Angle of rotation from z_(i-1) to z_(i) about the x_(i) axis
%   theta = Angle of rotation from x_(i-1) to x_(i) about the z_(i-1) axis
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [H] = dhTransform(a, d, alpha, theta)

    Transz = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
    Rotz = [rotZ(theta), [0;0;0]; 0, 0, 0, 1];
    Transx = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    Rotx = [rotX(alpha), [0;0;0]; 0, 0, 0, 1];
    
    H = Transz*Rotz*Transx*Rotx;

end