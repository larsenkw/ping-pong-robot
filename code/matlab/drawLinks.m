% drawLinks: Rgenerates a 3D plot of the link structure given a list of
% links created by createLink() and the joint parameters.
% 
%   [] = drawLinks(linkList, paramList, figureNum)  This function takes a list of
%   links created by createLink() and draws the structure corresponding to
%   the provided joint parameters in paramList on a 3D plot.  The X and Z
%   axis for the DH parameters are shown at the distal end of each link.
%   If a list of joint parameters are not provided the zero-angle
%   configuration is shown.
%
%   [] = outpus is simply a plot
%
%   linkList = array of structures containing the link parameters, each
%   created by createLink() (nx1)
%   paramList = array of current action variables for each link (nx1)
%   paramRateList = derivative of each action variable with respect to time
%   (nx1, optional)
%   figureNum = number of the figure to plot on (optional)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [] = drawLinks(linkList, paramList, figureNum)

    nLinks = length(linkList);
    linkOrigin = zeros(3,nLinks);
    % X axis is first column, Z is second
    linkAxes = zeros(3,2,nLinks);
    
    if ~exist('paramList', 'var')
        paramList = zeros(nLinks,1);
    end

    if ~exist('figureNum', 'var')
        figureNum = gcf;
    end
    
    % Generate transform for each link in relation to base frame
    for i = 1:nLinks
        T = dhFwdKine(linkList(1:i), paramList(1:i,1));
        % Extract displacement and rotation
        linkOrigin(1:3,i) = T(1:3,4);
        linkAxes(:,:,i) = [T(1:3,1),T(1:3,3)];
    end
    
    % Plot
    figure(figureNum);
    for i = 1:nLinks
        % Plot segment
        if (i == 1)
            plot3([0;linkOrigin(1,i)],[0;linkOrigin(2,i)],[0;linkOrigin(3,i)], 'k-', 'LineWidth', 5);
            hold on;
        else
            plot3([linkOrigin(1,i-1);linkOrigin(1,i)],[linkOrigin(2,i-1);linkOrigin(2,i)],[linkOrigin(3,i-1);linkOrigin(3,i)], 'k-', 'LineWidth', 5);
        end
        % Plot axes
        quiver3(linkOrigin(1,i), linkOrigin(2,i), linkOrigin(3,i), linkAxes(1,1,i), linkAxes(2,1,i), linkAxes(3,1,i), 0.1, 'g');
        quiver3(linkOrigin(1,i), linkOrigin(2,i), linkOrigin(3,i), linkAxes(1,2,i), linkAxes(2,2,i), linkAxes(3,2,i), 0.1, 'b');
    end
    axis equal;
    hold off;

end