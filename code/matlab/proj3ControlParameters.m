% This function generates the control parameters and saves them to the
% workspace

Kp = eye(3)*1000;
Kd = eye(3)*450;

eps = 10^-2;
a_slope = 100;
Tmax = 1;