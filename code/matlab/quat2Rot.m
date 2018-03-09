% rot2Quad:  Returns the rotation matrix that corresponds to the
% quaternion.
%
%   [R] = rot2Quad(Q)  This function takes a set of quaternion parameters
%   as a single (4x1) vector and calculate the rotation matrix from those
%   parameters.
%
%   R = The rotation matrix (3x3)
%
%   Q = The quaternion parameters [q0;q_vec] (4x1)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [R] = quat2Rot(Q)
    
    R = [2*(Q(1,1)^2+Q(2,1)^2)-1 2*(Q(2,1)*Q(3,1)-Q(1,1)*Q(4,1)) 2*(Q(2,1)*Q(4,1)+Q(1,1)*Q(3,1)); ...
        2*(Q(2,1)*Q(3,1)+Q(1,1)*Q(4,1)) 2*(Q(1,1)^2+Q(3,1)^2)-1 2*(Q(3,1)*Q(4,1)-Q(1,1)*Q(2,1)); ...
        2*(Q(2,1)*Q(4,1)-Q(1,1)*Q(3,1)) 2*(Q(3,1)*Q(4,1)+Q(1,1)*Q(2,1)) 2*(Q(1,1)^2+Q(4,1)^2)-1];

end