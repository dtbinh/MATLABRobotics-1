clc
close all
clear all
%% Ackerman Bicycle Motion Model
r = 0.25; 
l = 0.3; 

L = 0.3; 
MAX_ANGLE = 30*pi/180; 

dt = 0.1; % Timestep

% init states for Q3
x = [400 50 2]';
x1 = [400 50 2]'; 

% init states for Q2
% x = [ 0.0 0.0 0.0]';
% x1 = [ 0.0 0.0 0.0]';

R = diag([0.02^2 0.02^2 (1*pi/180)^2]);
[RE, Re] = eig(R);


vt = 3; %3m/s
n=200; % Samples

% Q2 rectangle discretized path
%points = [[0,0];[2,0];[4,0];[6,0];[8,0];[10,0];[12,0];[14,0];[16,0];[18,0];[20,0];[20,1];[20,3];[20,5];[18,5];[16,5];
%[14,5];[12,5];[10,5];[8,5];[6,5];[4,5];[2,5];[0,5];[0,3];[0,1]]

% PRM path
points = [[400,50];[342,103];[338,113];[269,129];[214,171];[168,310];[149,355];
[132,442];[144,484];[129,529];[81,573];[151,644];[201,620];[271,694];[285,570];
[344,414];[412,338];[484,269];[567,281];[572,377];[577,447];[610,528];[660,616];
[706,647];[788,673];[821,644];[817,613];[801,600];[792,513];[788,499];[798,425];
[807,315];[811,226];[819,144];[831,60];[675,92];[581,100];]

%points = [[0,0];[5,0];[10,0];[15,0];[20,0];[20,5];[15,5];[10,5];[5,5];[0,5]];
%points = [[0,0];[20,0];[20,5];[0,5]]

%while loop control variables
endLoop = 0;
halfWayCheck = 0;
%conservative thresholding
upperXLimit = 22;
upperYLimit = 6;
lowerXLimit = 18;
lowerYLimit = 4;

state = 0;
tolerance = 0.5;
%steer_ang = 0; 
route_done = 0;
lookup = 1;
i = 2; 
x_next = 50; % init random shit
init_dir = 'forward';
init_loop = 1; 
kp = 1;
while route_done == 0
    %Update positions x, y of robot
    x(1,i) = x(1,i-1) + vt * cos(x(3,i-1)) * dt;
    x(2,i) = x(2,i-1) + vt * sin(x(3,i-1)) * dt;
    
    %determine which point on path is closest
    distances = sqrt((points(:,1) - x(1,i)).^2 + (points(:,2) - x(2,i)).^2); 
    [M,I] = min(distances);
    
%This part should check if we need to keep increasing out step, but don't worry   
%        if M < 0
%            I = I + lookup;
%            if I > length(points)
%                I = 1;
%            elseif I < 1
%                I = length(points);
%            end
%            M = distances(I);
%        end
    
    %get index of new carrot points
    next_ind = I + lookup;
    prev_ind = I - lookup;    
    %min_dist_x = points(I,1);
    %min_dist_y = points(I,2); 
    
    % DESIGN POINT: CHOOSE PREVIOUS STATE OF CURRENT (i)
    x0 = x(1,i-1);
    y0 = x(2,i-1);
    theta0 = x(3,i-1); 
    
    % loop around path points
    if (next_ind > length(points))
        next_ind = 1;
    end
    if prev_ind < 1
        prev_ind = length(points); 
    end
    
    % determine x,y values of lookahead points (both ahead and before)
    % before not used
    x_next = points(next_ind,1); 
    y_next = points(next_ind,2); 
    
    x_prev = points(prev_ind,1); 
    y_prev = points(prev_ind,2);
    
    % when first moving, check direction of path you're going (fwd or bwd)
    if init_loop == 1
        
        if abs(atan2(y_next-y0,x_next-x0)-theta0) < abs(atan2(y_prev-y0,x_prev-x0)-theta0)
            init_dir = 'forward';
        else
            init_dir = 'backward';
        end
        init_loop = 0; 
        
    end
    
    %depending on if you were initially moving fwd of bwd, moving in that
    %direction along path
     if strcmp(init_dir, 'forward')
         theta_err = atan2(y_next-y0,x_next-x0)-theta0;
         %theta_err = atan((y_next-y0)/(x_next-x0));
     else
         theta_err = atan2(y_prev-y0,x_prev-x0)-theta0;
         %theta_err = atan((y_prev-y0)/(x_prev-x0));
     end
    
    if theta_err > pi || theta0 > pi || theta_err < -pi
        
        %disp(theta0);
        theta_err = (wrapToPi(theta_err));
    end
    
    %P control
    %theta_err = min(abs(atan2(y_next-y0,x_next-x0)-theta0), abs(atan2(y_prev-y0,x_prev-x0)-theta0));
    theta_err = theta_err * kp; 
    steer_ang = theta_err; 
       
    %angle saturation due to bike angle
    if steer_ang < -MAX_ANGLE
        steer_ang = -MAX_ANGLE; 
    elseif steer_ang > MAX_ANGLE
        steer_ang = MAX_ANGLE; 
    end
    
    
    %update heading of robot with new steering angle
    x(3,i) = (x(3,i-1) + vt * tan(steer_ang)/L * dt);
    
    %plot lookahead points (should be rectangle)
    path(1,i) = points(next_ind, 1);
    path(2,i) = points(next_ind,2);
    
    %guassian noise
    x(:,i) = x(:,i) + RE*sqrt(Re)*randn(3,1);
    i = i + 1; 
    if(i == 9005)
        disp(i);
        route_done=1;
    end
end

%read map and store it into occ grid (nodes)
I = imread('IGVCmap.jpg');
I = imgaussfilt(I,9);
%I = im2bw(I,0.0001); 
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.

% Plot
figure(1); clf; hold on;
colormap('gray'); % added colormap
imagesc(1-map');  % added colormap
plot(x(1,:), x(2,:), 'Color', 'r');
plot(path(1,2:end),path(2,2:end), 'Color', 'b');  % Removed first param for 0,0
%plot (x1(1,:), x1(2,:), 'Color', 'b'); 
plot( x(1,1), x(2,1), 'bo', 'MarkerSize',20, 'LineWidth', 3)
%plot( x1(1), x1(2), 'bo', 'MarkerSize',20, 'LineWidth', 3)
%plot( [x0(1) x1(1)], [x0(2) x1(2)],'b')
%plot( x(1,:),x(2,:), 'm.', 'MarkerSize', 3)
title('Carrot Follower, K = 1')
xlabel('x (m)');
ylabel('y (m)');
axis equal
hold on
%rectangle('Position',[0,0, 20,5],'Curvature',[0,0],...
         %'LineWidth',2,'LineStyle','--');