% rpy2Rot: Returns the rotation matrix corresponding to the roll, pitch,
% and yaw angles provided.
% 
%   [R] = rpy2Rot(roll, pitch, yaw)  This function takes the roll, pitch,
%   and yaw angles and calculates a rotation matrix.  The process uses the
%   Euler Angles ZYX and the given angles as follows: R =
%   Rotz(roll)*Roty(pitch)*Rotx(yaw)
%
%   R = the rotation matrix (3x3)
%
%   roll  = the angle of rotation about the current Z axis (in radians)
%   pitch = the angle of rotation about the current Y axis (in radians)
%   yaw   = the angle of rotation about the current X axis (in radians)
%   
%   Kyle Larsen
%   10832395
%   MEGN544
%   1 Oct 2017

function [R] = rpy2Rot(roll, pitch, yaw)

    % He may want Roll to be X-axis rotation of the current frame, Pitch as
    % Y-axis rotation of the current frame and yaw as Z-axis rotation of
    % the current frame, thereby switching the 'roll' and 'yaw' angles
    %----- Test Switch -----%
%     roll_original = roll;
%     roll = yaw;
%     yaw = roll_original;
    %----- Test Switch -----%

    R = [cos(roll)*cos(pitch), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);...
        sin(roll)*cos(pitch), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);...
        -sin(pitch), cos(pitch)*sin(yaw), cos(pitch)*cos(yaw)];

end