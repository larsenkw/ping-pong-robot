% abbInvKine: Returns the joint angles required to reach the desired
% transformation for the ABB arm we solved in Homework 5.
% 
%   [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
%   This function takes a desired transformation matrix (which contains
%   position and orientation information) and calculates the inverse
%   kinematics of the ABB robot arm, with the parameters as provided in
%   Homework 5.  The second argument, if provided, is the last set of theta
%   values used (the current joint angles), which provides a current
%   position.  Of the 8 possible solutions generated, the returned solution
%   will be the one closest to the th_last input.  If a second argument is
%   not provided, all 8 solutions will be returned.
%
%   th1 -> th6 = the joint angles for joints 1 through 6 (scalars if
%   th_last is not provided, 8x1 vectors otherwise)
%   reachable = boolean indicating whether the provided transformation
%   matrix is achievable (true if it can be reached, false if not)
%
%   T_des = the desired homogeneous transformation matrix (4x4)
%   th_last = set of theta parameters last provided to the robot (6x1
%   vector of the current joint angles)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
    %reachable = ones(8,1);
    reachable = true;
    tol = 1*10^-10;

    %----- ABB Robot Parameters -----%
    %--- DH Parameters
    d1 = 0.290;
    a2 = 0.270;
    a3 = 0.070;
    d4 = 0.302;
    d6 = 0.072;
    a_vec = [0; a2; a3; 0; 0; 0];
    alpha_vec = [-pi/2; 0; -pi/2; pi/2; -pi/2; 0];
    d_vec = [d1; 0; 0; d4; 0; d6];
    
    %--- Specs
    com = [0, 0, 0, 0, 0, 0];
    mass = [0; 0; 0; 0; 0; 0];
    inertia = [0; 0; 0; 0; 0; 0];
    
    %--- Create Links
    linkList(1) = createLink(a_vec(1,1), d_vec(1,1), alpha_vec(1,1), [], ...
                             com(:,1), mass(1,1), inertia(1,1));
    linkList(2) = createLink(a_vec(2,1), d_vec(2,1), alpha_vec(2,1), [], ...
                             com(:,2), mass(2,1), inertia(2,1));
    linkList(3) = createLink(a_vec(3,1), d_vec(3,1), alpha_vec(3,1), [], ...
                             com(:,3), mass(3,1), inertia(3,1));
    linkList(4) = createLink(a_vec(4,1), d_vec(4,1), alpha_vec(4,1), [], ...
                             com(:,4), mass(4,1), inertia(4,1));
    linkList(5) = createLink(a_vec(5,1), d_vec(5,1), alpha_vec(5,1), [], ...
                             com(:,5), mass(5,1), inertia(5,1));
    linkList(6) = createLink(a_vec(6,1), d_vec(6,1), alpha_vec(6,1), [], ...
                             com(:,6), mass(6,1), inertia(6,1));                     
    %----- ABB Robot Parameters -----%

    %----- Solve for thetas -----%
    % There are 8 possible solution sets for this problem.  The solutions
    % will be returned in this format, the header shows the joint angle and
    % the columns indicate the solution number for that angle:
    %    theta1  theta2  theta3  theta4  theta5  theta6
    %  [   1   |   1   |   1   |   1   |   1   |   1   ;
    %      1   |   1   |   1   |   2   |   2   |   2   ;
    %      1   |   2   |   2   |   3   |   3   |   3   ;
    %      1   |   2   |   2   |   4   |   4   |   4   ;
    %      2   |   3   |   3   |   5   |   5   |   5   ;
    %      2   |   3   |   3   |   6   |   6   |   6   ;
    %      2   |   4   |   4   |   7   |   7   |   7   ;
    %      2   |   4   |   4   |   8   |   8   |   8    ]

    %--- Theta 1
    theta1_tmp = zeros(2,1);
    theta1_real = zeros(2,1);
    d0_06 = T_des(1:3,4);
    z0_6 = T_des(1:3,3);
    d0_05 = d0_06 - z0_6*linkList(6).d;
    theta1_tmp(1,1) = atan2(d0_05(2,1), d0_05(1,1));
    theta1_tmp(2,1) = theta1_tmp(1,1) + pi;
    theta1_real = real(theta1_tmp);
    
    %--- Theta 2 & 3
    theta2_tmp = zeros(4,1);
    theta2_real = zeros(4,1);
    theta3_tmp = zeros(4,1);
    theta3_real = zeros(4,1);
    
    % Theta1_1
    T0_1 = dhTransform(linkList(1).a, linkList(1).d, linkList(1).alpha, ...
                       theta1_real(1,1));
    R0_1 = T0_1(1:3,1:3);
    d1_15 = (R0_1')*(d0_06 - z0_6*linkList(6).d - [0;0;1]*linkList(1).d);
    % Theta3_1, Theta2_1
    theta3_tmp(1,1) = pi + 2*atan(sqrt((2*linkList(2).a*sqrt(linkList(4).d^2 + ...
        linkList(3).a^2) - linkList(2).a^2 - linkList(4).d^2 - linkList(3).a^2 + ...
        norm(d1_15)^2)/(2*linkList(2).a*sqrt(linkList(4).d^2 + linkList(3).a^2) + ...
        linkList(2).a^2 + linkList(4).d^2 + linkList(3).a^2 - norm(d1_15)^2))) - ...
        atan2(linkList(4).d, linkList(3).a);
    theta3_real(1,1) = real(theta3_tmp(1,1));
    zeta = (linkList(2).a + linkList(3).a*cos(theta3_real(1,1)) + linkList(4).d* ...
        cos(theta3_real(1,1) + pi/2));
    beta = (linkList(3).a*sin(theta3_real(1,1)) + linkList(4).d*sin(theta3_real(1,1) + ...
        pi/2));
    theta2_tmp(1,1) = atan2((zeta*d1_15(2,1) - beta*d1_15(1,1))/(zeta^2 + beta^2), ...
        (zeta*d1_15(1,1) + beta*d1_15(2,1))/(zeta^2 + beta^2));
    theta2_real(1,1) = real(theta2_tmp(1,1));
    % Theta3_2, Theta2_2
    theta3_tmp(2,1) = pi - 2*atan(sqrt((2*linkList(2).a*sqrt(linkList(4).d^2 + ...
        linkList(3).a^2) - linkList(2).a^2 - linkList(4).d^2 - linkList(3).a^2 + ...
        norm(d1_15)^2)/(2*linkList(2).a*sqrt(linkList(4).d^2 + linkList(3).a^2) + ...
        linkList(2).a^2 + linkList(4).d^2 + linkList(3).a^2 - norm(d1_15)^2))) - ...
        atan2(linkList(4).d, linkList(3).a);
    theta3_real(2,1) = real(theta3_tmp(2,1));
    zeta = (linkList(2).a + linkList(3).a*cos(theta3_real(2,1)) + linkList(4).d* ...
        cos(theta3_real(2,1) + pi/2));
    beta = (linkList(3).a*sin(theta3_real(2,1)) + linkList(4).d*sin(theta3_real(2,1) + ...
        pi/2));
    theta2_tmp(2,1) = atan2((zeta*d1_15(2,1) - beta*d1_15(1,1))/(zeta^2 + beta^2), ...
        (zeta*d1_15(1,1) + beta*d1_15(2,1))/(zeta^2 + beta^2));
    theta2_real(2,1) = real(theta2_tmp(2,1));
    
    % Theta1_2
    T0_1 = dhTransform(linkList(1).a, linkList(1).d, linkList(1).alpha, ...
                       theta1_real(2,1));
    R0_1 = T0_1(1:3,1:3);
    d1_15 = (R0_1')*(d0_06 - z0_6*linkList(6).d - [0;0;1]*linkList(1).d);
    % Theta3_3, Theta2_3
    theta3_tmp(3,1) = pi + 2*atan(sqrt((2*linkList(2).a*sqrt(linkList(4).d^2 + ...
        linkList(3).a^2) - linkList(2).a^2 - linkList(4).d^2 - linkList(3).a^2 + ...
        norm(d1_15)^2)/(2*linkList(2).a*sqrt(linkList(4).d^2 + linkList(3).a^2) + ...
        linkList(2).a^2 + linkList(4).d^2 + linkList(3).a^2 - norm(d1_15)^2))) - ...
        atan2(linkList(4).d, linkList(3).a);
    theta3_real(3,1) = real(theta3_tmp(3,1));
    zeta = (linkList(2).a + linkList(3).a*cos(theta3_real(3,1)) + linkList(4).d* ...
        cos(theta3_real(3,1) + pi/2));
    beta = (linkList(3).a*sin(theta3_real(3,1)) + linkList(4).d*sin(theta3_real(3,1) + ...
        pi/2));
    theta2_tmp(3,1) = atan2((zeta*d1_15(3,1) - beta*d1_15(1,1))/(zeta^2 + beta^2), ...
        (zeta*d1_15(1,1) + beta*d1_15(3,1))/(zeta^2 + beta^2));
    theta2_real(3,1) = real(theta2_tmp(3,1));
    % Theta3_4, Theta2_4
    theta3_tmp(4,1) = pi - 2*atan(sqrt((2*linkList(2).a*sqrt(linkList(4).d^2 + ...
        linkList(3).a^2) - linkList(2).a^2 - linkList(4).d^2 - linkList(3).a^2 + ...
        norm(d1_15)^2)/(2*linkList(2).a*sqrt(linkList(4).d^2 + linkList(3).a^2) + ...
        linkList(2).a^2 + linkList(4).d^2 + linkList(3).a^2 - norm(d1_15)^2))) - ...
        atan2(linkList(4).d, linkList(3).a);
    theta3_real(4,1) = real(theta3_tmp(4,1));
    zeta = (linkList(2).a + linkList(3).a*cos(theta3_real(4,1)) + linkList(4).d* ...
        cos(theta3_real(4,1) + pi/2));
    beta = (linkList(3).a*sin(theta3_real(4,1)) + linkList(4).d*sin(theta3_real(4,1) + ...
        pi/2));
    theta2_tmp(4,1) = atan2((zeta*d1_15(3,1) - beta*d1_15(1,1))/(zeta^2 + beta^2), ...
        (zeta*d1_15(1,1) + beta*d1_15(3,1))/(zeta^2 + beta^2));
    theta2_real(4,1) = real(theta2_tmp(4,1));

    %--- Theta 4, 5, & 6
    theta5_tmp = zeros(8,1);
    theta5_real = zeros(8,1);
    theta4_tmp = zeros(8,1);
    theta4_real = zeros(8,1);
    theta6_tmp = zeros(8,1);
    theta6_real = zeros(8,1);
    % Theta1_1, Theta2_1, Theta3_1
    T0_3 = dhFwdKine(linkList(1:3), [theta1_real(1,1);theta2_real(1,1);theta3_real(1,1)]);
    R0_3 = T0_3(1:3,1:3);
    R3_6 = (R0_3')*T_des(1:3,1:3);
    theta5_tmp(1,1) = atan2(sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_tmp(2,1) = atan2(-sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_real(1,1) = real(theta5_tmp(1,1));
    theta5_real(2,1) = real(theta5_tmp(2,1));
    if ((sin(theta5_real(1,1)) < tol) && (sin(theta5_real(1,1)) > -tol))
        theta4_tmp(1:2,1) = zeros(2,1);
        theta6_tmp(1:2,1) = ones(2,1)*atan2(R3_6(2,1), R3_6(1,1));
    else
        theta4_tmp(1,1) = atan2(-R3_6(2,3)/sin(theta5_real(1,1)), -R3_6(1,3)/sin(theta5_real(1,1)));
        theta4_real(1,1) = real(theta4_tmp(1,1));
        theta6_tmp(1,1) = atan2(-R3_6(3,2)/sin(theta5_real(1,1)), R3_6(3,1)/sin(theta5_real(1,1)));
        theta6_real(1,1) = real(theta6_tmp(1,1));
        theta4_tmp(2,1) = atan2(-R3_6(2,3)/sin(theta5_real(2,1)), -R3_6(1,3)/sin(theta5_real(2,1)));
        theta4_real(2,1) = real(theta4_tmp(2,1));
        theta6_tmp(2,1) = atan2(-R3_6(3,2)/sin(theta5_real(2,1)), R3_6(3,1)/sin(theta5_real(2,1)));
        theta6_real(2,1) = real(theta6_tmp(2,1));
    end
    % Theta1_1, Theta2_2, Theta3_2
    T0_3 = dhFwdKine(linkList(1:3), [theta1_real(1,1);theta2_real(2,1);theta3_real(2,1)]);
    R0_3 = T0_3(1:3,1:3);
    R3_6 = (R0_3')*T_des(1:3,1:3);
    theta5_tmp(3,1) = atan2(sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_tmp(4,1) = atan2(-sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_real(3,1) = real(theta5_tmp(3,1));
    theta5_real(4,1) = real(theta5_tmp(4,1));
    if ((sin(theta5_real(3,1)) < tol) && (sin(theta5_real(3,1)) > -tol))
        theta4_tmp(3:4,1) = zeros(2,1);
        theta6_tmp(3:4,1) = ones(2,1)*atan2(R3_6(2,1), R3_6(1,1));
    else
        theta4_tmp(3,1) = atan2(-R3_6(2,3)/sin(theta5_real(3,1)), -R3_6(1,3)/sin(theta5_real(3,1)));
        theta4_real(3,1) = real(theta4_tmp(3,1));
        theta6_tmp(3,1) = atan2(-R3_6(3,2)/sin(theta5_real(3,1)), R3_6(3,1)/sin(theta5_real(3,1)));
        theta6_real(3,1) = real(theta6_tmp(3,1));
        theta4_tmp(4,1) = atan2(-R3_6(2,3)/sin(theta5_real(4,1)), -R3_6(1,3)/sin(theta5_real(4,1)));
        theta4_real(4,1) = real(theta4_tmp(4,1));
        theta6_tmp(4,1) = atan2(-R3_6(3,2)/sin(theta5_real(4,1)), R3_6(3,1)/sin(theta5_real(4,1)));
        theta6_real(4,1) = real(theta6_tmp(4,1));
    end
    % Theta1_2, Theta2_3, Theta3_3
    T0_3 = dhFwdKine(linkList(1:3), [theta1_real(2,1);theta2_real(3,1);theta3_real(3,1)]);
    R0_3 = T0_3(1:3,1:3);
    R3_6 = R0_3'*T_des(1:3,1:3);
    theta5_tmp(5,1) = atan2(sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_tmp(6,1) = atan2(-sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_real(5,1) = real(theta5_tmp(5,1));
    theta5_real(6,1) = real(theta5_tmp(6,1));
    if ((sin(theta5_real(5,1)) < tol) && (sin(theta5_real(5,1)) > -tol))
        theta4_tmp(5:6,1) = zeros(2,1);
        theta6_tmp(5:6,1) = ones(2,1)*atan2(R3_6(2,1), R3_6(1,1));
    else
        theta4_tmp(5,1) = atan2(-R3_6(2,3)/sin(theta5_real(5,1)), -R3_6(1,3)/sin(theta5_real(5,1)));
        theta4_real(5,1) = real(theta4_tmp(5,1));
        theta6_tmp(5,1) = atan2(-R3_6(3,2)/sin(theta5_real(5,1)), R3_6(3,1)/sin(theta5_real(5,1)));
        theta6_real(5,1) = real(theta6_tmp(5,1));
        theta4_tmp(6,1) = atan2(-R3_6(2,3)/sin(theta5_real(6,1)), -R3_6(1,3)/sin(theta5_real(6,1)));
        theta4_real(6,1) = real(theta4_tmp(6,1));
        theta6_tmp(6,1) = atan2(-R3_6(3,2)/sin(theta5_real(6,1)), R3_6(3,1)/sin(theta5_real(6,1)));
        theta6_real(6,1) = real(theta6_tmp(6,1));
    end
    % Theta1_2, Theta2_4, Theta3_4
    T0_3 = dhFwdKine(linkList(1:3), [theta1_real(2,1);theta2_real(4,1);theta3_real(4,1)]);
    R0_3 = T0_3(1:3,1:3);
    R3_6 = R0_3'*T_des(1:3,1:3);
    theta5_tmp(7,1) = atan2(sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_tmp(8,1) = atan2(-sqrt(R3_6(3,1)^2 + R3_6(3,2)^2), R3_6(3,3));
    theta5_real(7,1) = real(theta5_tmp(7,1));
    theta5_real(8,1) = real(theta5_tmp(8,1));
    if ((sin(theta5_real(7,1)) < tol) && (sin(theta5_real(7,1)) > -tol))
        theta4_tmp(7:8,1) = zeros(2,1);
        theta6_tmp(7:8,1) = ones(2,1)*atan2(R3_6(2,1), R3_6(1,1));
    else
        theta4_tmp(7,1) = atan2(-R3_6(2,3)/sin(theta5_real(7,1)), -R3_6(1,3)/sin(theta5_real(7,1)));
        theta4_real(7,1) = real(theta4_tmp(7,1));
        theta6_tmp(7,1) = atan2(-R3_6(3,2)/sin(theta5_real(7,1)), R3_6(3,1)/sin(theta5_real(7,1)));
        theta6_real(7,1) = real(theta6_tmp(7,1));
        theta4_tmp(8,1) = atan2(-R3_6(2,3)/sin(theta5_real(8,1)), -R3_6(1,3)/sin(theta5_real(8,1)));
        theta4_real(8,1) = real(theta4_tmp(8,1));
        theta6_tmp(8,1) = atan2(-R3_6(3,2)/sin(theta5_real(8,1)), R3_6(3,1)/sin(theta5_real(8,1)));
        theta6_real(8,1) = real(theta6_tmp(8,1));
    end
    
    % Generate 8 solution matrix
    th1_tmp = [ones(4,1)*theta1_tmp(1,1); ones(4,1)*theta1_tmp(2,1)];
    th2_tmp = [ones(2,1)*theta2_tmp(1,1); ones(2,1)*theta2_tmp(2,1); ones(2,1)*theta2_tmp(3,1); ...
           ones(2,1)*theta2_tmp(4,1)];
    th3_tmp = [ones(2,1)*theta3_tmp(1,1); ones(2,1)*theta3_tmp(2,1); ones(2,1)*theta3_tmp(3,1); ...
           ones(2,1)*theta3_tmp(4,1)];
    th4_tmp = theta4_tmp;
    th5_tmp = theta5_tmp;
    th6_tmp = theta6_tmp;
    solutions_tmp = [th1_tmp,th2_tmp,th3_tmp,th4_tmp,th5_tmp,th6_tmp];
    %----- Solve for thetas -----%
    
    %----- Check for real solutions (reachable) -----%
    for i = 1:8
        if (~isreal(solutions_tmp(i,:)))
            %reachable(i,1) = false;
            reachable = false;
            solutions_tmp(i,:) = real(solutions_tmp(i,:));
        end
    end
    solutions = wrapToPi(solutions_tmp);
    th1 = solutions(:,1);
    % Add offset of pi/2, so the 0 angle position is not going through the
    % table
    th2 = solutions(:,2) + pi/2;
    th3 = solutions(:,3);
    th4 = solutions(:,4);
    th5 = solutions(:,5);
    th6 = solutions(:,6);
    %----- Check for real solutions (reachable) -----%
    
    %----- Find closest solution to th_last -----%
    if (exist('th_last', 'var'))
        % Calculate error between th_last and each solution
        
        % Convert th_last (theta_ABB) to theta_DH 
        th_last(2) = th_last(2) - pi/2;
        
        % Performs least squares solution for th4 and th6 when th5 is
        % degenerate
        for i = 1:8
            if ((sin(th5(i,1)) < tol) && (sin(th5(i,1)) > -tol))
                solutions(i,4) =  th_last(4)/2 - th_last(6)/2 + solutions(i,6)/2;
                solutions(i,6) = -th_last(4)/2 + th_last(6)/2 + solutions(i,6)/2;
            end
        end
        
        error = zeros(8,6);
        error_sq = zeros(8,1);
        for i = 1:8
            error(i,:) = th_last - solutions(i,:)';
            error_sq(i,1) = dot(error(i,:), error(i,:));
        end
        min_index = find(min(error_sq)==error_sq);
        th1 = solutions(min_index, 1);
        % Add offset of pi/2, so the 0 angle position is not going through the
        % table
        th2 = solutions(min_index, 2) + pi/2;
        th3 = solutions(min_index, 3);
        th4 = solutions(min_index, 4);
        th5 = solutions(min_index, 5);
        th6 = solutions(min_index, 6);
    end
    %----- Find closest solution to th_last -----%
end