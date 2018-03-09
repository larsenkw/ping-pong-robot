% rot2RPY: Returns the roll, pitch, and yaw angles given a rotation matrix.
% 
%   [roll, pitch, yaw] = rot2RPY(R)  This function takes the roll, pitch,
%   and yaw angles and calculates a rotation matrix.  The process uses the
%   Euler Angles ZYX and the given angles as follows: R =
%   Rotz(roll)*Roty(pitch)*Rotx(yaw)
%
%   roll  = the angle of rotation about the current Z axis (in radians)
%   pitch = the angle of rotation about the current Y axis (in radians)
%   yaw   = the angle of rotation about the current X axis (in radians)
%
%   R = the provided rotation matrix (3x3)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [roll, pitch, yaw] = rot2RPY(R)

    % Case where pitch is within the range (-pi/2,pi/2)
    roll1   = atan2(R(2,1), R(1,1));
    pitch1  = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    yaw1    = atan2(R(3,2), R(3,3));
    
    % Case where pitch is within the range (pi/2, 3pi/2)
    roll2   = atan2(-R(2,1), -R(1,1));
    pitch2  = atan2(-R(3,1), -sqrt(R(3,2)^2 + R(3,3)^2));
    yaw2    = atan2(-R(3,2), -R(3,3));
    
    if (pitch1 == pi/2)
        roll1 = atan2(R(2,3), R(1,3));
        roll2 = atan2(R(2,3), R(1,3));
        yaw1 = 0;
        yaw2 = 0;
    end
    
    % He may want Roll to be X-axis rotation of the current frame, Pitch as
    % Y-axis rotation of the current frame and yaw as Z-axis rotation of
    % the current frame, thereby switching the 'roll' and 'yaw' angles
    %----- Test Switch -----%
%     roll1_original = roll1;
%     roll1 = yaw1;
%     yaw1 = roll1_original;
%     roll2_original = roll2;
%     roll2 = yaw2;
%     yaw2 = roll2_original;
    %----- Test Switch -----%
    
    roll = [roll1;roll2];
    pitch = [pitch1;pitch2];
    yaw = [yaw1;yaw2];
    
end