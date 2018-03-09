% cpMap:  Returns the skew-symmetric matrix associated with the first
% element of a cross product
%
%   [X] = cpMap(w)  This function takes the vector 'w' and converts it into
%   a matrix [w]_x such that when right-multiplied by another vector it
%   produces a vector equivalent with the cross product of the two.
%   Example: w x v = [w]_x*v = cross(w)*v
%
%   X = The skew-symmetric matrix [w]_x (3x3)
%
%   w = The 3x1 column vector to be turned into skew-symmetric form (3x1)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [X] = cpMap(w)

    X = [0 -w(3,1) w(2,1);
         w(3,1) 0 -w(1,1);
         -w(2,1) w(1,1) 0];

end