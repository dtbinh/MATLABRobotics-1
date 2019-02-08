clc
close all
clear all

% This code defines the necessary inputs for Question 2A

l = 0.5; % m
v = 3.0; % m/s   constant input velocity
steerAngle = 3; % constant input control in degrees
steerAngleRad = degtorad(steerAngle); % constant input control in radians
dt = 0.1; % seconds --> update rate is 10 Hz
x = [0.0 0.0 0.0]'; %init state
R = diag([0.05^2 0.05^2 (0.01)^2]);   % Gaussian disturbances 
[RE, Re] = eig(R);


n=200; % # of samples ...20 seconds at 0.1s freq
for i=2:n
    % Disturbance
    E = RE*sqrt(Re)*randn(3,1);
    % Dynamics
    
    x(:,i) = x(:,i-1) + [dt*(v)*cos(x(3,i-1)) ; dt*(v)*sin(x(3,i-1))  ; dt*(1/l)*(v)*tan(steerAngleRad)] + E;
   end

figure(1); clf; hold on;
plot(x(1,:), x(2,:), 'Color', 'r');
plot(x(1), x(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
%plot(x(1,:), x(2,:), 'Color', 'r');
title('Motion Model for Ackermann Model')
xlabel('x (m)');
ylabel('y (m)');
axis equal