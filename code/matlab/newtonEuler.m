% newtonEuler:  Performs the Newton-Euler recursion method to calculate
% joint torques provided the base conditions and the distal end wrench.
% 
%   [jointTorques,Jv,JvDot] = newtonEuler(linkList,paramList,paramListDot, 
%   paramListDDot,boundary_conditions)  This function takes in a list of
%   links created using createLink(), the current parameters and their
%   rates for those links, and the boundary conditions consisting of the
%   base motion and the wrench on the distal end, and then calculates the
%   velocity Jacobian, its time derivative, and the joint torques required
%   to produce the input state.
%
%   jointTorques = The torque required for each joint to produce the
%   current state specified by the joint parameters and their rates (n x 1)
%   Jv = velocity Jacobian (6 x n)
%   JvDot = time derivative of velocity Jacobian (6 x n)
%
%   linkList = list of links created by createLink() (n x 1)
%   paramList = list of joint parameters (n x 1)
%   paramListDot = list of time derivative of joint parameters (n x 1)
%   paramListDDot = list of second time derivative of joint parameters (n x
%   1)
%   boundary_conditions = a structure containing the following parameters:
%       base_angular_velocity
%       base_angular_acceleration 
%       base_linear_acceleration (add gravity in here) 
%       distal_force
%       distal_torque
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [jointTorques,Jv,JvDot] = newtonEuler(linkList,paramList,paramListDot,paramListDDot,boundary_conditions)

    %========== Forward Kinemtaics ==========%
    %----- Solve for Pose -----%
    %----- Save: R0_i, d0_i, z_i
    nLinks = length(linkList);
    R0_i = zeros(3,3,nLinks);
    d0_i = zeros(3,nLinks);
    z_0 = [0;0;1];
    z_i = zeros(3,nLinks);
    
    T = eye(4);
    for i = 1:nLinks
        if (linkList(i).isRotary)
            Ti_n = dhTransform(linkList(i).a, linkList(i).d, ...
                              linkList(i).alpha, paramList(i));
        else
            Ti_n = dhTransform(linkList(i).a, paramList(i), ...
                              linkList(i).alpha, linkList(i).theta);
        end
        T = T*Ti_n;
        R0_i(:,:,i) = T(1:3,1:3);
        d0_i(:,i) = T(1:3,4);
        z_i(:,i) = R0_i(1:3,3,i);
    end
    
    %----- Solve for Velocity and Acceleration -----%
    %----- Save: v_i, a_i, w_i, wdot_i
    %*** Velocities ***
    % Get linear and angular velocities by following the recursive
    % function in section 8.4 of the course notes (Hollerbach).
    v_0 = [0;0;0];
    %w_0 = boundary_conditions.base_angular_velocity;
    w_0 = [0;0;0];
    v_i = zeros(3,nLinks);
    w_i = zeros(3,nLinks);    
    for i = 1:nLinks
        if (i == 1)
            if (linkList(i).isRotary)
                w_i(:,i) = w_0 + paramListDot(i)*z_0;
                v_i(:,i) = v_0 + cross(w_i(:,i), d0_i(:,i));
            else
                w_i(:,i) = w_0 + zeros(3,1);
                v_i(:,i) = v_0 + cross(w_i(:,i), d0_i(:,i)) + paramListDot(i);
            end
        else
            if (linkList(i).isRotary)
                w_i(:,i) = w_i(:,i-1) + paramListDot(i)*z_i(:,i-1);
                v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                    d0_i(:,i-1)));
            else
                w_i(:,i) = w_i(:,i-1) + zeros(3,1);
                v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                    d0_i(:,i-1))) + paramListDot(i)*z_i(:,i-1);
            end
        end
    end
    
    %----- Generate Jacobian and its Derivative -----%
    Jv = zeros(6,nLinks);
    JvDot = zeros(6,nLinks);
    % Iterate over links to generate columns of Jacobian
    for i = 1:nLinks
        if (i == 1)
            if (linkList(i).isRotary)
                Jv(:,1) = [cpMap(z_0)*d0_i(:,nLinks); z_0];
            else
                Jv(:,1) = [z_0; zeros(3,1)];
            end
        else
            if (linkList(i).isRotary)
                Jv(:,i) = [cpMap(z_i(:,i-1))*(d0_i(:,nLinks) - d0_i(:,i-1)); ...
                           z_i(:,i-1)];
            else
                Jv(:,i) = [z_i(:,i-1); zeros(3,1)];
            end
        end
    end
    
    % Calculate JvDot using linear and angular velocities
    for i = 1:nLinks
        if (i == 1)
            if (linkList(i).isRotary)
                JvDot(1:6,i) = [cpMap(cpMap(zeros(3,1))*[0;0;1])* ...
                                (d0_i(:,nLinks)) + (cpMap([0;0;1])* ...
                                (v_i(:,nLinks))); cpMap(zeros(3,1))* ...
                                ([0;0;1])];
            else
                JvDot(1:6,i) = [cpMap(zeros(3,1))* ...
                                ([0;0;1]); zeros(3,1)];
            end
        else
            if (linkList(i).isRotary)
                JvDot(1:6,i) = [cpMap(cpMap(w_i(:,i-1))*z_i(:,i-1))* ...
                                (d0_i(:,nLinks) - d0_i(:,i-1)) + ...
                                (cpMap(z_i(:,i-1))*(v_i(:,nLinks) - ...
                                v_i(:,i-1))); cpMap(w_i(:,i-1))*(z_i(:,i-1))];
            else
                JvDot(1:6,i) = [cpMap(w_i(:,i-1))*(z_i(:,i-1)); zeros(3,1)];
            end
        end
    end
    %========== Forward Kinemtaics ==========%
    
    %========== Inverse Dynamics (Newton-Euler Recursion) ==========%
    %*** Velocities - Now including boundary conditions ***
    % Get linear and angular velocities by following the recursive
    % function in section 8.4 of the course notes (Hollerbach).
    v_0 = [0;0;0];
    w_0 = boundary_conditions.base_angular_velocity;
    v_i = zeros(3,nLinks);
    w_i = zeros(3,nLinks);    
    for i = 1:nLinks
        if (i == 1)
            if (linkList(i).isRotary)
                w_i(:,i) = w_0 + paramListDot(i)*z_0;
                v_i(:,i) = v_0 + cross(w_i(:,i), d0_i(:,i));
            else
                w_i(:,i) = w_0 + zeros(3,1);
                v_i(:,i) = v_0 + cross(w_i(:,i), d0_i(:,i)) + paramListDot(i);
            end
        else
            if (linkList(i).isRotary)
                w_i(:,i) = w_i(:,i-1) + paramListDot(i)*z_i(:,i-1);
                v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                    d0_i(:,i-1)));
            else
                w_i(:,i) = w_i(:,i-1) + zeros(3,1);
                v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                    d0_i(:,i-1))) + paramListDot(i)*z_i(:,i-1);
            end
        end
    end
    
    %*** Accelerations ***
    % Calculate Linear and Angular Accelerations using iterative equations 
    % from Section 8.4 (Hollerbach).
    a_0 = boundary_conditions.base_linear_acceleration;
    wdot_0 = boundary_conditions.base_angular_acceleration;
    a_i = zeros(3,nLinks);
    wdot_i = zeros(3,nLinks);
    for i = 1:nLinks
        if (i == 1)
            if (linkList(i).isRotary)
                wdot_i(:,i) = wdot_0 + paramListDDot(i)*z_0 + ...
                    paramListDot(i)*cross(w_0,z_0);
                a_i(:,i) = a_0 + cross(wdot_i(:,i),d0_i(:,i)) + cross(w_i(:,i),cross(w_i(:,i),d0_i(:,i)));
            else
                wdot_i(:,i) = wdot_0 + zeros(3,1);
                a_i(:,i) = a_0 + cross(wdot_i(:,i), d0_i(:,i)) + cross(w_i(:,i),cross(w_i(:,i),d0_i(:,i))) + ...
                    paramListDDot(i)*z_0 + 2*paramListDot(i)*cross(w_0, z_0);
            end
        else
            if (linkList(i).isRotary)
                wdot_i(:,i) = wdot_i(:,i-1) + paramListDDot(i)*z_i(:,i-1) + ...
                    paramListDot(i)*cross(w_i(:,i-1),z_i(:,i-1));
                a_i(:,i) = a_i(:,i-1) + cross(wdot_i(:,i),(d0_i(:,i) - d0_i(:,i-1))) + ...
                    cross(w_i(:,i),cross(w_i(:,i),(d0_i(:,i) - d0_i(:,i-1))));
            else
                wdot_i(:,i) = wdot_i(:,i-1) + zeros(3,1);
                a_i(:,i) = a_i(:,i-1) + cross(wdot_i(:,i),(d0_i(:,i) - d0_i(:,i-1))) + ...
                    cross(w_i(:,i),cross(w_i(:,i),(d0_i(:,i) - d0_i(:,i-1)))) + ...
                    paramListDDot(i)*z_i(:,i-1) + 2*paramListDot(i)*cross(w_i(:,i-1),z_i(:,i-1));
            end
        end
    end
    
    
    Fd = boundary_conditions.distal_force;
    Td = boundary_conditions.distal_torque;
    w_0 = boundary_conditions.base_angular_velocity;
    F_i = zeros(3,nLinks);
    T_i = zeros(3,nLinks);
    r_i = zeros(3,nLinks);
    ri_1_i = zeros(3,nLinks);
    rdot0_i = zeros(3,nLinks);
    rddot0_i = zeros(3,nLinks);
    J0_i = zeros(3,3,nLinks);
    m_i = zeros(nLinks,1);
    jointTorques = zeros(nLinks,1);
    % Loop from Base and Calculate Center of Mass Parameters
    for i = 1:nLinks
        if (i > 1)
            m_i(i,1) = linkList(i).mass;
            J0_i(:,:,i) = R0_i(:,:,i)*linkList(i).inertia*R0_i(:,:,i)';
            r_i(:,i) = R0_i(:,:,i)*linkList(i).com;
            ri_1_i(:,i) = (d0_i(:,i) - d0_i(:,i-1)) + r_i(:,i);
            if (linkList(i).isRotary)
                rdot0_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), ri_1_i(:,i));
                rddot0_i(:,i) = a_i(:,i-1) + cross(w_i(:,i), cross(w_i(:,i),ri_1_i(:,i))) + ...
                    cross(wdot_i(:,i), ri_1_i(:,i));
            else
                rdot0_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), ri_1_i(:,i)) + paramListDot(i)*z_i(:,i-1);
                rddot0_i(:,i) = a_i(:,i-1) + cross(w_i(:,i), cross(w_i(:,i),ri_1_i(:,i))) + ...
                    cross(wdot_i(:,i), ri_1_i(:,i)) + cross(2*paramListDot(i)*w_i(:,i-1), z_i(:,i-1)) + ...
                    paramListDDot(i)*z_i(:,i-1);
            end
        else
            m_i(i,1) = linkList(i).mass;
            J0_i(:,:,i) = R0_i(:,:,i)*linkList(i).inertia*R0_i(:,:,i)';
            r_i(:,i) = R0_i(:,:,i)*linkList(i).com;
            ri_1_i(:,i) = (d0_i(:,i) - zeros(3,1)) + r_i(:,i);
            if (linkList(i).isRotary)
                rdot0_i(:,i) = v_0 + cross(w_i(:,i), ri_1_i(:,i));
                rddot0_i(:,i) = a_0 + cross(w_i(:,i), cross(w_i(:,i),ri_1_i(:,i))) + ...
                    cross(wdot_i(:,i), ri_1_i(:,i));
            else
                rdot0_i(:,i) = v_0 + cross(w_i(:,i), ri_1_i(:,i)) + paramListDot(i)*z_0;
                rddot0_i(:,i) = a_0 + cross(w_i(:,i), cross(w_i(:,i),ri_1_i(:,i))) + ...
                    cross(wdot_i(:,i), ri_1_i(:,i)) + cross(2*paramListDot(i)*w_0, z_0) + ...
                    paramListDDot(i)*z_0;
            end
        end
    end
    
    % Loop from Distal End Towards Base to find World and Joint Forces/Torques
    for i = nLinks:-1:1
        if (i == nLinks)
            %--- Base Force
            F_i(:,i) = Fd + m_i(i,1)*rddot0_i(:,i);
            %--- Base Torque
            T_i(:,i) = Td - cross(r_i(:,i), Fd) + ...
                cross(ri_1_i(:,i), F_i(:,i)) + J0_i(:,:,i)*wdot_i(:,i) + ...
                cross(w_i(:,i), J0_i(:,:,i)*w_i(:,i));
            if (i > 1)
                if (linkList(i).isRotary)
                    jointTorques(i) = dot(z_i(:,i-1),T_i(:,i));
                else
                    jointTorques(i) = dot(z_i(:,i-1),F_i(:,i));
                end
            else
                if (linkList(i).isRotary)
                    jointTorques(i) = dot(z_0,T_i(:,i));
                else
                    jointTorques(i) = dot(z_0,F_i(:,i));
                end
            end
        else
            %--- Forces
            F_i(:,i) = F_i(:,i+1) + m_i(i,1)*rddot0_i(:,i);
            %--- Torques
            T_i(:,i) = T_i(:,i+1) - cross(r_i(:,i), F_i(:,i+1)) + ...
                cross(ri_1_i(:,i), F_i(:,i)) + J0_i(:,:,i)*wdot_i(:,i) + ...
                cross(w_i(:,i), J0_i(:,:,i)*w_i(:,i));
            if (i > 1)
                if (linkList(i).isRotary)
                    jointTorques(i) = dot(z_i(:,i-1),T_i(:,i));
                else
                    jointTorques(i) = dot(z_i(:,i-1),F_i(:,i));
                end
            else
                if (linkList(i).isRotary)
                    jointTorques(i) = dot(z_0,T_i(:,i));
                else
                    jointTorques(i) = dot(z_0,F_i(:,i));
                end
            end
        end
        
    end
    %========== Inverse Dynamics (Newton-Euler Recursion) ==========%

end