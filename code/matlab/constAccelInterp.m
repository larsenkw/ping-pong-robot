% constAccelInterp:  Computes the position, velocity, and acceleration
% along a set of trajectory points using a constant acceleration method.
% 
%   [p,v,a] = constAccelInterp(t, trajectory, transPercent)  This function
%   finds the position, velocity, and acceleration of a trajectory
%   specified by the points in the input 'trajectory' at the specified time
%   't'.  The values are computed using the constant acceleration method as
%   outined in Hollerbach section 6.2.3.  The 'transPercent' input is the
%   variable that changes the transition time for the constant acceleration
%   portion.  This value can be from 0 to 1 which generates a tau from 0 to
%   one-half of the smallest time length for the points in consideration.
%
%   p = position (corresponds with the interpolated value) (N x M)
%   v = velocity (the first time derivative of the interpolated value)
%   (N x M)
%   a = acceleration (the second time derivative of the interpolated value)
%   (N x M)
%
%   t = the time value where the position, velocity, and acceleration are
%   desired
%   trajectory = A matrix that describes the trajectory, of size N x (M +
%   1).  There are N points described by values in M dimensions.  The first
%   column is the time of each point and the remaining columns are the
%   point values in each of the M dimensions.
%   transPercent = fraction corresponding to the amount of time used for
%   the constant acceleration transition phase.
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [p,v,a] = constAccelInterp(t, trajectory, transPercent)

    %---------- Find Closest Point ----------%
    % Special cases: value is closest to the first point or last point in
    % the trajectory, in these cases the path is simply linear
    % Anything else is in the middle and can be solved with the book
    % equations
    
    % Check that transPercent is <= 1, >= 0;
    if (exist('transPercent','var'))
        if (transPercent > 1)
            fprintf('Variable "transPercent" > 1, reassigned to 1');
            transPercent = 1;
        elseif (transPercent < 0)
            fprintf('Variable "transPercent" < 0, reassigned to 0');
            transPercent = 0;
        end
    else
        transPercent = 0;
    end
    
    time = trajectory(:,1);
    nPoints = size(time,1);
    t_error = (t - time).^2;
    % Find point with minimum error, this is point 1
    [~, i_1] = min(t_error);
    
    %----- First Point: Special Case where p1 is the first point
    if (i_1 == 1)
        % Get Position, Velocity, Acceleration
        %   p = p1 + v1*t
        p1 = trajectory(i_1,2:end);
        p2 = trajectory(i_1+1,2:end);
        t1 = time(i_1+1) - time(i_1);
        v1 = (p2 - p1)/t1;
        dt = t-time(i_1);
        p = p1 + v1*dt;
        v = v1;
        a = zeros(size(p));
        
    %----- Last Point: Special Case where p1 is the last point
    elseif (i_1 == nPoints)
        % Get Position, Velocity, Acceleration
        %   p = p2 - v2*t
        p2 = trajectory(i_1,2:end);
        p1 = trajectory(i_1-1,2:end);
        t2 = time(i_1) - time(i_1-1);
        v2 = (p2 - p1)/t2;
        dt = time(i_1) - t;
        p = p2 - v2*dt;
        v = v2;
        a = zeros(size(p));
        
    %----- Middle Point: p1 is between p0 and p2
    else
        % Find Tau
        tau = min(time(i_1+1) - time(i_1), time(i_1) - time(i_1-1));
        tau = transPercent*tau;
        % Get Position, Velocity, Acceleration
        p0 = trajectory(i_1-1,2:end);
        p1 = trajectory(i_1,2:end);
        p2 = trajectory(i_1+1,2:end);
        t1 = time(i_1) - time(i_1-1);
        t2 = time(i_1+1) - time(i_1);
        v1 = (p1 - p0)/t1;
        v2 = (p2 - p1)/t2;
        % Linear Portion 1
        %   p = p0 + v1*t
        if (t < (time(i_1) - tau))
            dt = time(i_1) - t;
            p = p1 - v1*dt;
            v = v1;
            a = zeros(size(p));
        % Linear Portion 2
        %   p = p2 - v2*t
        elseif (t > time(i_1) + tau)
            dt = time(i_1+1) - t;
            p = p2 - v2*dt;
            v = v2;
            a = zeros(size(p));
        % Constant Acceleration
        %   p = p0 + v*t + 0.5*a*t^2
        else
            a = (v2 - v1)/(2*tau);
            dt = t - (time(i_1)-tau);
            p = (p1 - v1*tau) + v1*dt + 0.5*a*dt^2;
            v = v1 + a*dt;
        end
    end
end