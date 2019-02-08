clc
close all
clear all

%{ 
Requirements:
- generate 100 random features located aroudn vehicle's circular path 
between 0 and 2 m --> don't count features as obstacles
- Implement motion and measurement models
- Plot robot pose and measurements to features at 3 different instances
for a simulation where it completes a single near circle
- Try to represent measurment vectors clearly and correspond to particular
features.... do this by only plotting visible features and by positioning 
plot view to best illustrate measurement vectors

Note: reduced 3D elevation to 2D so I can finish this problem in time....

%}

l = 0.5; % m
v = 3.0; % m/s   constant input velocity
steerAngle = 3; % constant input control in degrees
steerAngleRad = degtorad(steerAngle); % constant input control in radians

dt = 0.1; % seconds --> update rate is 10 Hz
Tf = 20;
T = 0:dt:Tf; % discretized time vector

% motion Disturbance model
x = [0.0 0.0 0.0]'; %init state
R = diag([0.05^2 0.05^2 (0.01)^2]);   % Gaussian disturbances 
[RE, Re] = eig(R);

% measurement Disturbance model
% Q = [1e-4 0 0; 
%      0 6.4e-3 0; 
%      0 0 1e-4];
% reducted to 2D by neglecting the less important elevation
Q = [1e-4 0; 
     0 6.4e-3]; 
[QE, Qe] = eig(Q);

% Sensor footprint
rmax = 6 ; % meters  % max scan distance = 6m
thmax = pi/6; % rads   % max scan angle = 60 deg --> half is 30 deg


% create random features here and plot 
% using randi([min max],size,Ntime)
map = [];
for i=1:100
    map = [map; randi([-10 10],1) randi([0 20], 1)];
end

% simulation initializations
M = length(map(:,1));
mf = zeros(2*M,2); 
y = zeros(2*M,length(T));
m = length(Q(:,1));

figure(1); clf; hold on;
plot(x(1,:), x(2,:), 'Color', 'r');
plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);

%plot(x(1), x(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
%plot(x(1,:), x(2,:), 'Color', 'r');
title('Random Environment Generation part C')
xlabel('x (m)');
ylabel('y (m)');
hold on;

n=200; % # of samples ...20 seconds at 0.1s freq
for t=2:length(T)
    % Motion Disturbance
    E = RE*sqrt(Re)*randn(3,1);
    % Motion Model
    x(:,t) = x(:,t-1) + [dt*(v)*cos(x(3,t-1)) ; dt*(v)*sin(x(3,t-1))  ; dt*(1/l)*(v)*tan(steerAngleRad)] + E;


    % Take measurement
    % Identify features that are in view
    nj = 0;
    for i=1:M
        % If feature is visible
        if (inview(map(i,:),x(:,t),rmax,thmax))
            nj = nj+1;
            mf(nj,:) = map(i,:);

            % measurement disturbance
            d = QE*sqrt(Qe)*randn(m,1);
            % Determine measurement based on model from part 
             y(nj:nj+1,t) = [sqrt((mf(nj,1)-x(1,t))^2 + (mf(nj,2)-x(2,t))^2)
                 atan2(mf(nj,2)-x(2,t),mf(nj,1)-x(1,t))-x(3,t)] +  d ;
%             y(nj:nj+1,t) = [(atan2((mf(nj,2)-x(2,t)),(mf(nj,1)-x(1,t)))-x(3,t)) ...
%                 sqrt((mf(nj,2)-x(2,t))^2 + (mf(nj,1)-x(1,t))^2)] + d;
            %y(nj+1,t) = mod(y(nj+1,t)+pi,2*pi)-pi;
            nj = nj+1;
            mf(nj,:) = map(i,:);
        end
    end

% UNCOMMENT THIS FOR FULL PATH IMAGE
%     for j = 1:nj
%             plot(mf(j,1),mf(j,2),'mx', 'MarkerSize',10,'LineWidth',2) % plots circle of interest
%             plot(x(1,1:t),x(2,1:t), 'ro--') % actual path
%             if (mod(j,2))
%                 % plots line to circle
%                 plot([x(1,t) x(1,t)+y(j,t)*cos(y(j+1,t)+x(3,t))], [ x(2,t) x(2,t)+y(j,t)*sin(y(j+1,t)+x(3,t))], 'c');
%             end
%     end
end
for j = 1:nj
            plot(mf(j,1),mf(j,2),'mx', 'MarkerSize',10,'LineWidth',2) % plots circle of interest
            plot(x(1,1:5),x(2,1:5), 'ro--') % actual path
            if (mod(j,2))
                % plots line to circle
                plot([x(1,5) x(1,5)+y(j,5)*cos(y(j+1,5)+x(3,5))], [ x(2,5) x(2,5)+y(j,5)*sin(y(j+1,5)+x(3,5))], 'c');
            end
end
axis equal