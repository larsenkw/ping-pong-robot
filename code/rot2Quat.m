% rot2Quad:  Returns the quaternion [q0;q_vec] that corresponds to the
% rotation matrix.
%
%   [Q] = rot2Quad(R)  This function takes a rotation matrix R and calculates
%   the quaternion parameters q0 and q_vec and returns them in a single
%   vector Q.
%
%   Q = The quaternion parameters [q0;q_vec] (4x1)
%
%   R = The rotation matrix (3x3)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [Q] = rot2Quat(R)
    % Scalar value
    q0 = (1/2)*sqrt(R(1,1)+R(2,2)+R(3,3)+1);
    
    % Vector values
    q_vec = zeros(3,1);
    if (q0 ~= 0)
        q_vec(1,1) = (1/2)*sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
        q_vec(2,1) = (1/2)*sign(R(1,3)-R(3,1))*sqrt(R(2,2)-R(3,3)-R(1,1)+1);
        q_vec(3,1) = (1/2)*sign(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1);
    else
        q1_sq = (1/4)*(R(1,1) - R(2,2) - R(3,3) + 1);
        q2_sq = (1/4)*(-R(1,1) + R(2,2) - R(3,3) + 1);
        q3_sq = (1/4)*(-R(1,1) - R(2,2) + R(3,3) + 1);
        
        % Find largest value, then solve for other terms using product
        % relationship
        if (q1_sq == max([q1_sq, q2_sq, q3_sq]))
            q1 = sqrt(q1_sq);
            q2 = (R(1,2) + R(2,1))/(4*q1);
            q3 = (R(1,3) + R(3,1))/(4*q1);
            q_vec = [q1;q2;q3];
        elseif (q2_sq == max([q1_sq, q2_sq, q3_sq]))
            q2 = sqrt(q2_sq);
            q1 = (R(1,2) + R(2,1))/(4*q2);
            q3 = (R(2,3) + R(3,2))/(4*q2);
            q_vec = [q1;q2;q3];
        else
            q3 = sqrt(q3_sq);
            q1 = (R(1,3) + R(3,1))/(4*q3);
            q2 = (R(2,3) + R(3,2))/(4*q3);
            q_vec = [q1;q2;q3];
        end
        
    end
    
    Q = [q0;q_vec];

end