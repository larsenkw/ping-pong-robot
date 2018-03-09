% velocityJacobian: Returns the velocity Jacobian of the manipulator given
% an array of links created by the createLink() function and the current
% joint variables.
% 
%   [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
%   This function computes the velocity Jacobian matrix for a given set of
%   links characterized by the DH parameters provided in the link structure
%   created by createLink() and the current action variables included in
%   paramList.  If the input paramRateList is provided, the derivative of
%   the Jacobian is also computed and returned.
%
%   Jv = the velocity Jacobian (6xn, n = number of links)
%   JvDot = the velocity Jacobian derivative with respect to time (6xn)
%
%   linkList = array of structures containing the link parameters, each
%   created by createLink() (nx1)
%   paramList = array of current action variables for each link (nx1)
%   paramRateList = derivative of each action variable with respect to time
%   (nx1, optional)
%
%   Kyle Larsen
%   10832395
%   MEGN544
%   19 Nov 2017

function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)

    % There are two methods available for generating the Jacobian matrix.
    % First is the start from the base and work towards the end link.
    % Second is to start from the end link and work back towards the base.
    % Both options are included.  Change the method by altering the
    % following variable (true = start from base, false = start from end).
    startFromBase = true;
    
    nLinks = length(paramList);
    Jv = zeros(6, nLinks);
    
    calcJvDot = exist('paramRateList', 'var');
    if calcJvDot
        JvDot = zeros(6, nLinks);
    else
        JvDot = [];
    end
    
    %----- Calculate the Jacobian -----%
    if startFromBase
        %--- Starting from Base
        T0_n = eye(4);
        z_0 = [0; 0; 1];
        z_i = zeros(3, nLinks);
        d0_i = zeros(3, nLinks);

        % Iterate over links to generate z-vectors and d-vectors
        for i = 1:nLinks
            % Rotary Joint
            if (linkList(i).isRotary)
                T0_n = T0_n*dhTransform(linkList(i).a, linkList(i).d, ...
                       linkList(i).alpha, paramList(i));
            % Prismatic Joint
            else
                T0_n = T0_n*dhTransform(linkList(i).a, paramList(i), ...
                       linkList(i).alpha, linkList(i).theta);
            end
            % Extract z_i and d0_i
            z_i(:,i) = T0_n(1:3,3);
            d0_i(:,i) = T0_n(1:3,4);
        end

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
    
    else
        %--- Starting from End
        Tn_i = eye(4);
        
        for i = nLinks:-1:1
            if (linkList(i).isRotary)
                Ti_1_i = dhTransform(linkList(i).a, linkList(i).d, ...
                       linkList(i).alpha, paramList(i,1));
            else
                Ti_1_i = dhTransform(linkList(i).a, paramList(i,1), ...
                       linkList(i).alpha, linkList(i).theta);
            end
            Ri_1_i = Ti_1_i(1:3,1:3);
            di_1_i = Ti_1_i(1:3,4);
            
            Ti_i_1 = [Ri_1_i', -Ri_1_i'*di_1_i; 0, 0, 0, 1];
            Tn_i_1 = Tn_i*Ti_i_1;
            Tn_i = Tn_i_1;
            
            if (linkList(i).isRotary)
                Jv_n(:,i) = [cpMap(Tn_i_1(1:3,3))*(-Tn_i_1(1:3,4)); ...
                             Tn_i_1(1:3,3)];
            else
                Jv_n(:,i) = [Tn_i_1(1:3,3); zeros(3,1)];
            end
        end
        Rn_0 = Tn_i_1(1:3,1:3);
        Jv = [Rn_0', zeros(3); zeros(3), Rn_0']*Jv_n;
    end
    %----- Calculate the Jacobian -----%
    
    %----- Calculate the Jacobian Derivative -----%
    if calcJvDot
        % ******************************************
        % You must use the 'startFromBase' method because it calculates the
        % d0_i and z_i vectors that are used within this function
        % ******************************************
        
        % Get linear and angular velocities by following the recursive
        % function in section 8.4 of the course notes (Hollerbach).
        v_i = zeros(3,nLinks);
        w_i = zeros(3,nLinks);
        v_0 = [0;0;0];
        w_0 = [0;0;0];
        for i = 1:nLinks
            if (i == 1)
                if (linkList(i).isRotary)
                    w_i(:,i) = w_0 + paramRateList(i)*z_0;
                    v_i(:,i) = v_0 + cross(w_i(:,i),d0_i(:,i));
                else
                    w_i(:,i) = w_0 + zeros(3,1);
                    v_i(:,i) = v_0 + cross(w_i(:,i), d0_i(:,i)) + paramRateList(i);
                end
            else
                if (linkList(i).isRotary)
                    w_i(:,i) = w_i(:,i-1) + paramRateList(i)*z_i(:,i-1);
                    v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                        d0_i(:,i-1)));
                else
                    w_i(:,i) = w_i(:,i-1) + zeros(3,1);
                    v_i(:,i) = v_i(:,i-1) + cross(w_i(:,i), (d0_i(:,i) - ...
                        d0_i(:,i-1))) + paramRateList(i)*z_i(:,i-1);
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
    end
    %----- Calculate the Jacobian Derivative -----%

end