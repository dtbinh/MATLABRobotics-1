clc
close all
clear all

l = 0.3; % m
v = 3.0; % m/s
dt = 0.1; % s
x = [0.0 0.0 0.0]'; %init state
R = diag([0.02^2 0.02^2 (pi/180)^2]);
[RE, Re] = eig(R);

% angle limit = +- 0.523599 radians

n=200; % # of samples ...20 seconds at 0.1s freq
for i=2:n
    % Disturbance
    E = RE*sqrt(Re)*randn(3,1);
    % Dynamics
    steerAngle = 10 - i/10; % input control in degrees
    if steerAngle > 30
        steerAngle = 30;
    elseif steerAngle < -30
            steerAngle = -30;
    end
    x(:,i) = x(:,i-1) + [dt*(v)*cos(x(3,i-1)) ; dt*(v)*sin(x(3,i-1))  ; dt*(1/l)*(v)*tand(steerAngle)] + E;
   end

figure(1); clf; hold on;
plot(x(1,:), x(2,:), 'Color', 'r');
plot(x(1), x(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
%plot(x(1,:), x(2,:), 'Color', 'r');
title('Motion Model for Ackermann Model')
xlabel('x (m)');
ylabel('y (m)');
axis equal