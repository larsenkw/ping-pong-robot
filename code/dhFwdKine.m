% dhFwdKine: creates a structure for a link with the provided DH
% parameters, mass, center of mass location, and moment of inertia and
% rotary/prismatic type.
%
%   [H] = dhFwdKine(linkList, paramList)  This function calculates the
%   homogeneous transformation matrix of the end effector using forward
%   kinematics provided a list of links (n x 1) and the corresponding 
%   action parameters in an array (n x 1).  The DH parameters for the links
%   are assumed to have been generated using the standard convention (NOT
%   craig's convention).
%
%   H = the homogeneous transformation matrix for the end effector
%   corresponding to the provided joint parameters
%
%   linkList = array of link structures, these should have been created by
%   using the createLink() function, the dimensions are (n x 1)
%   paramList = array of action parameters for the corresponding links
%   provided in linkList (n x 1)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [H] = dhFwdKine(linkList, paramList)
    
    % Number of links
    nLinks = length(linkList);
    
    % Preallocate for transformation matrices going from link i to link i+1
    T = zeros(4,4,nLinks);
    
    % Transformation matrix from the origin to the end effector.  It is
    % calculated using a series of products with each consective link's DH
    % transformation matrix.
    H = eye(4);
    
    for i = 1:nLinks
        
        % Generate transformation matrices for each link using the DH
        % parameters in the link structure.  If the joint is rotational,
        % replace linkList(i).theta with paramList(i).  If the joint is
        % prismatic replate linkList(i).d with paramList(i).
        if (linkList(i).isRotary)
            T(:,:,i) = dhTransform(linkList(i).a, linkList(i).d, ...
                       linkList(i).alpha, paramList(i));
        else
            T(:,:,i) = dhTransform(linkList(i).a, paramList(i), ...
                       linkList(i).alpha, linkList(i).theta);
        end
        
        H = H*T(:,:,i);
        
    end
        

end