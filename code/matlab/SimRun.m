clear; clc; close all;

%% Controller Parameter Definitions
%output = 'kp_kd_error';
%output = 'a_eps_error';
output = 'plot';

% kp = [1,500,1000];
% kd = [1,500,1000];
% kp = linspace(1,2000,10);
% kd = linspace(1,2000,10);
% avg_rms = zeros(length(kp),length(kd));
% for j = 1:length(kp)
%     for k = 1:length(kd)
% 
% fprintf('Row %d of %d, Col %d of %d\n',j,length(kp),k,length(kd));

% kp = 1000;
% kd = 450;
kp = 1000;
kd = 450;
Kp = eye(3)*kp;
Kd = eye(3)*kd;

eps = 10^-2;
a_slope = 100;
Tmax = 1; % from his plant saturation block

%% Run Simulation

simOut =  sim('Part_3_Control','SimulationMode','normal','AbsTol','1e-6','StopTime', '30',...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');

X_desired = simOut.get('X_desired');
X_actual = simOut.get('X_actual');
Theta_actual = simOut.get('Theta_actual');
ControlTorque = simOut.get('ControlTorque');

% Calculate RMS
% figure(3*(j-1)+k);
figure(1);
plot(X_actual(:,1),sqrt(sum((X_actual(:,2:4)-X_desired(:,2:4)).^2,2)));
title('Rms Position Error (0.9 cycles per second)')
xlabel('time')
ylabel('error [m]')
xlim([0,X_actual(end,1)]);

% rms_error = sqrt(sum((X_actual(:,2:4)-X_desired(:,2:4)).^2,2));
% avg_rms(j) = mean(rms_error);

rms_error = sqrt(sum((X_actual(:,2:4)-X_desired(:,2:4)).^2,2));
avg_rms = mean(rms_error);

% figure(1); 
% plot(X_actual(:,1),X_desired(:,2)-X_actual(:,2));
% hold on;
% plot(X_actual(:,1),X_desired(:,3)-X_actual(:,3));
% plot(X_actual(:,1),X_desired(:,4)-X_actual(:,4));
% title('Position Error (With Sliding Mode)')
% xlabel('time')
% ylabel('error [m]')
% xlim([0,X_actual(end,1)]);
% ylim([-0.012,0.012]);
% legend('x','y','z');

%% Make 3d Plots

% fwd kin for joint locations
L1 = 0.25; L2 = 1; L3 = .5;
jointPos = @(t1,t2,t3)[[    0, 0, 0, L2*cos(t1)*cos(t2), cos(t1)*(L3*cos(t2 + t3) + L2*cos(t2))]
[    0, 0, 0, L2*cos(t2)*sin(t1), sin(t1)*(L3*cos(t2 + t3) + L2*cos(t2))]
[ 0, L1, L1,         L2*sin(t2) + L1,           L3*sin(t2 + t3) + L2*sin(t2)+L1]];

switch output
    case 'plot'
        %for i=1:50:length(X_desired)
        for i=1:100:length(X_desired)
        figure(1)
%         figure(3*(j-1)+k);
        subplot(1,2,1)
        jP = jointPos(Theta_actual(i,2),Theta_actual(i,3),Theta_actual(i,4));
        plot3(jP(1,:), jP(2,:), jP(3,:),'k','LineWidth',4)
        hold on;
        if (i <= 500)
            plot3(X_desired(1:i,2),X_desired(1:i,3),X_desired(1:i,4),'k:','LineWidth',2)
        else
            plot3(X_desired(i-500:i,2),X_desired(i-500:i,3),X_desired(i-500:i,4),'k:','LineWidth',2)
        end
        plot3(X_actual(1:i,2),X_actual(1:i,3),X_actual(1:i,4),'r')
        hold off;
        axis([-1.5,1.5,-1.5,1.5,-1,2])

        subplot(1,2,2)
        plot(X_actual(1:i,1),sqrt(sum((X_actual(1:i,2:4)-X_desired(1:i,2:4)).^2,2)));
        title('Rms Position Error')
        xlabel('time')
        ylabel('error [cm]')
        xlim([0,X_actual(end,1)]);

        pause(.05)
        end
    case 'a_eps_error'
        disp('eps:');
        disp(10.^-power);
        disp('a:');
        disp(a_slope);
        disp('Avg_rms');
        disp(avg_rms);
    case 'kp_kd_error'
        disp('Kp:');
        disp(kp);
        disp('Kd:');
        disp(kd);
        disp('Avg_rms');
        disp(avg_rms);
    otherwise
        fprintf("No output mode selected\n");
end

 