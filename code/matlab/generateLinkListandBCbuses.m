%==========================================================================
% This code generates the structures for the parameters 'linkList' and
% 'boundary_conditions' and they will be saved to the workspace.  Type the
% command 'buseditor' and rename them as desired.  You can then export them
% to a '.mat' file which will be loaded at initialization using the
% function 'proj3LoadBuses()'.  The values are passed to functions by
% creating a constant and setting its type to 'Bus: proj3linkList' then
% naming the constant 'linkListP3'.  For the boundary conditions the type
% is 'Bus: proj3boundary_conditions' and the name is 'boundary_conditions'
%==========================================================================

%----- Robot link specs -----%
% Density of aluminum
rho = 2700;     % kg/m^3

% Link dimensions
radius = .05;       % m
L = [0.25, 1, 0.5]; % m

% Calculate volume and mass
V = pi*radius^2*L;  % m^3
mass = rho*V;       % kg

% Center of Mass (w.r.t. link's frame)
% rows = [x;y;z], cols = [link1, link2, link3]
com = [      0, -L(2)/2, -L(3)/2;
       -L(1)/2,       0,       0;
             0,       0,       0];    % m

% Inertial Tensors
inertia = zeros(3,3,3);
inertia(:,:,1) = [(mass(1)/12)*(3*radius^2 + L(1)^2),0,0;
                  0,(mass(1)*radius^2)/2,0;
                  0,0,(mass(1)/12)*(3*radius^2 + L(1)^2)];
inertia(:,:,2) = [(mass(2)*radius^2)/2,0,0;
                  0,(mass(2)/12)*(3*radius^2 + L(2)^2),0;
                  0,0,(mass(2)/12)*(3*radius^2 + L(2)^2)];
inertia(:,:,3) = [(mass(3)*radius^2)/2,0,0;
                  0,(mass(3)/12)*(3*radius^2 + L(3)^2),0;
                  0,0,(mass(3)/12)*(3*radius^2 + L(3)^2)];

%----- linkList parameter -----%
% Here the action parameters 'theta' and 'd' values are replaced with 0
% when empty.  Simulink does not like empty vectors.  They will not be
% accessed anyway as long as isRotary is correct.
% DH Parameters
a = [0, L(2), L(3)];
d = [L(1), 0, 0];
alpha = [pi/2, 0, 0];
%*** action parameter ***%
theta = [0, 0, 0];
%*** action parameter ***%
linkListP3_1 = struct('a',a(1),'d',d(1),'alpha',alpha(1),'theta',theta(1),'mass',mass(1),'inertia',inertia(:,:,1),'com',com(:,1),'isRotary',1);
linkListP3_2 = struct('a',a(2),'d',d(2),'alpha',alpha(2),'theta',theta(2),'mass',mass(2),'inertia',inertia(:,:,2),'com',com(:,2),'isRotary',1);
linkListP3_3 = struct('a',a(3),'d',d(3),'alpha',alpha(3),'theta',theta(3),'mass',mass(3),'inertia',inertia(:,:,3),'com',com(:,3),'isRotary',1);
linkListP3 = [linkListP3_1,linkListP3_2,linkListP3_3];

%----- boundary_conditions parameter -----%
% Gravity
g = 9.81;  % m/s^2
boundary_conditions = struct('base_angular_velocity',[0;0;0], ...
    'base_angular_acceleration',[0;0;0], ...
    'base_linear_acceleration',[0;0;-g], ...
    'distal_force',[0;0;0], ...
    'distal_torque',[0;0;0]);

%----- Generate Bus Objects -----%
busInfo1 = Simulink.Bus.createObject(linkListP3);
busInfo2 = Simulink.Bus.createObject(boundary_conditions);

fprintf('Now run "buseditor" and save the two buses as "proj3linkListandBCbuses.mat"\n');
