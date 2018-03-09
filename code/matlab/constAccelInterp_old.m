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

    %---------- Determine Time Bounds for t ----------%
    time = trajectory(:,1);
    p_t = trajectory(:,[2:end]);
    i_t_lb = find(time <= t, 1, 'last');
    i_t_ub = find(time >= t, 1, 'first');
    if (isempty(i_t_lb) || isempty(i_t_ub))
        fprintf('Your specified time is outside the provided range\n');
    else
        if (i_t_lb == 1)
            segment = 'first';
        else
            if ((i_t_lb > 1) && (i_t_ub < length(trajectory(:,1))))
                segment = 'middle';
            else
                if (i_t_ub == length(trajectory(:,1)))
                    segment = 'end';
                else
                    segment = 'unknown';
                end
            end
        end
        
        switch segment
        %---------- Handle t Along First Segment ----------%
            case 'first'
            if (i_t_ub == i_t_lb)
                i = i_t_ub + 1; % time lies right on the first index
            else
                i = i_t_ub;
            end
            tau_i = 0.5*transPercent*min(time(i + 1) - time(i), ...
                time(i) - time(i-1));
            v_i = (p_t(i,:) - p_t(i-1,:))/(time(i) - time(i-1));
            v_i_1 = (p_t(i + 1,:) - p_t(i,:))/(time(i + 1) - time(i));
            a_i = (v_i_1 - v_i)/(2*tau_i);
            
            if (t < (time(i) - tau_i))
                p = p_t(i - 1,:) + v_i*(t - time(i - 1));
                v = v_i;
                a = zeros(size(p));
            else
                p = p_t(i - 1,:) + v_i*(t - time(i - 1)) + 0.5*a_i*(t - time(i) + tau_i)^2;
                v = v_i + a_i*(t - time(i) + tau_i);
                a = a_i;
            end
        %---------- Handle t Along First Segment ----------%
        
        %---------- Handle t Along Middle Segment ----------%
            case 'middle'
            i = i_t_lb;
            tau_i = 0.5*transPercent*min(time(i + 1) - time(i), ...
                time(i) - time(i-1));
            tau_i_1 = 0.5*transPercent*min(time(i + 2) - time(i + 1), ...
                time(i + 1) - time(i));
            v_i = (p_t(i,:) - p_t(i-1,:))/(time(i) - time(i-1));
            v_i_1 = (p_t(i + 1,:) - p_t(i,:))/(time(i + 1) - time(i));
            v_i_2 = (p_t(i + 2,:) - p_t(i + 1,:))/(time(i + 2) - time(i + 1));
            a_i = (v_i_1 - v_i)/(2*tau_i);
            a_i_1 = (v_i_2 - v_i_1)/(2*tau_i_1);
            
            if (t < (time(i) + tau_i))
                p = p_t(i - 1,:) + v_i*(t - time(i-1)) + 0.5*a_i*(t - time(i) + tau_i)^2;
                v = v_i + a_i*(t - time(i) + tau_i);
                a = a_i;
            else
                if (t < (time(i + 1) - tau_i_1))
                    p = p_t(i,:) + v_i_1*(t - time(i));
                    v = v_i_1;
                    a = zeros(size(p));
                else
                    p = p_t(i,:) + v_i_1*(t - time(i)) + 0.5*a_i_1*(t - time(i + 1) + tau_i_1)^2;
                    v = v_i_1 + a_i_1*(t - time(i + 1) + tau_i_1);
                    a = a_i_1;
                end
            end
        %---------- Handle t Along Middle Segment ----------%
        
        %---------- Handle t Along Last Segment ----------%
            case 'end'
            if (i_t_lb == i_t_ub)
                i = i_t_lb - 1; % time lies right on the last index
            else
                i = i_t_lb;
            end
            tau_i = 0.5*transPercent*min(time(i + 1) - time(i), ...
                time(i) - time(i-1));
            v_i = (p_t(i,:) - p_t(i-1,:))/(time(i) - time(i-1));
            v_i_1 = (p_t(i + 1,:) - p_t(i,:))/(time(i + 1) - time(i));
            a_i = (v_i_1 - v_i)/(2*tau_i);
            
            if (t < (time(i) + tau_i))
                p = p_t(i - 1,:) + v_i*(t - time(i-1)) + 0.5*a_i*(t - time(i) + tau_i)^2;
                v = v_i + a_i*(t - time(i) + tau_i);
                a = a_i;
            else
                p = p_t(i,:) + v_i_1*(t - time(i));
                v = v_i_1;
                a = zeros(size(p));
            end
        %---------- Handle t Along Last Segment ----------%
        
        %---------- Handle Error ----------%
            otherwise
                fprintf('Something went wrong, because your t did not fit within the indices of trajectory\n');
        end
        %---------- Handle Error ----------%
                
    end
    %---------- Determine Time Bounds for t ----------%

end