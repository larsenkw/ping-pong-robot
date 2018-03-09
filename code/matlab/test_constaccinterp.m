clear
clc
close all
trajectory = [0,0,0; 1.5,.7,1.5; 4,4,-2.1; 6.8,6.1,2.8; 8,1,3; 10,3,5];
t=linspace(trajectory(1,1),trajectory(end,1),1001);
%create x,y points in time for trajectory to hit
for m = 1:length(t)
    [p,v,a] = constAccelInterp(t(m),trajectory,.2);
    pos(m,:) = p;
    vel(m,:) = v;
    acc(m,:) = a;
end

plot(pos(:,1),pos(:,2),'d')
hold on
plot(trajectory(:,2),trajectory(:,3),'o')
figure
plot(t,sqrt(vel(:,1).^2+vel(:,2).^2))
hold on
plot(t,sqrt(acc(:,1).^2+acc(:,2).^2))

