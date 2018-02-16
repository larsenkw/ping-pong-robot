% createLink: creates a structure for a link with the provided DH
% parameters, mass, center of mass location, and moment of inertia and
% rotary/prismatic type.
%
%   [L] = createLink(a, d alpha, theta, com, mass, inertia,
%   isRotary)  This function creates a structure for a manipulator link
%   given the necessary parameters to describe it.  These parameters
%   include the DH parameters, the center of mass location, the mass, the
%   moment of inertia, and whether the joint is a rotary joint or prismatic
%   joint.  All the vectors for this function should be in the link's
%   coordinate frame.  For the action parameter (theta for rotary joint, d
%   for prismatic) pass in an empty array (e.g. createLink(1.5, 0.5, pi/2,
%   [], ...).  The function assumes the DH parameters are provided using
%   the standard convention (NOT craig's convention)
%
%   L = the structure represent a manipulator link with the give parameters
%   stored.
%
%   a = translation from z_(i-1) to z_(i) along the x_(i) axis (m)
%   d = translation from x_(i-1) to x_(i) along the z_(i-1) axis (m)
%   alpha = angle of rotation from z_(i-1) to z_(i) about the x_(i) axis
%   (m)
%   theta = angle of rotation from x_(i-1) to x_(i) about the z_(i-1) axis
%   (m)
%   com = the position of the link's center of mass (m)
%   mass = the link's mass (kg)
%   inertia = the link mass moment of inertia (kg m^2)
%   
%   Assigned within:
%   isRotary = true if the link has a rotary joint, false if the link has
%   a prismatic joint, determined by whether d or theta is empty.
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [L] = createLink(a, d, alpha, theta, com, mass, inertia)
    if (isempty(theta))
        isRotary = true;
    else
        isRotary = false;
    end
    
    L = struct('a', a, 'd', d, 'alpha', alpha, 'theta', theta, 'com', ...
        com, 'mass', mass, 'inertia', inertia, 'isRotary', isRotary);
end