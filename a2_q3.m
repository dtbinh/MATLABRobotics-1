%% Probabilistic Road Map example
clear all;
clc;
close all;


%% Problem parameters
tic;

%time limit
timeLimit = 900;

%read map and store it into occ grid (nodes)
I = imread('IGVCmap.jpg');
I = imgaussfilt(I,9);
%I = im2bw(I,0.0001); 
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size


% Robot start position
dxy = 0.1;
startpos = [40/dxy 5/dxy];

% Target location
searchgoal = [50/dxy 10/dxy];

% Set up the map
xMax = [M N]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = startpos;
xF = searchgoal;

%  % Set up the obstacles
% rand('state', 1);
% nO = 30; % number of obstacles
% nE = 4; % number of edges per obstacle (not changeable).
% minLen.a = 1; % Obstacle size bounds
% maxLen.a = 4;
% minLen.b = 1;
% maxLen.b = 6;
 
obstBuffer = 1; % Buffer space around obstacles
maxCount = 1000; % Iterations to search for obstacle locations

env = map;

%% Multi-query PRM, created until solution found
tic;
done = 0;
milestones = [x0; xF];
e = zeros(2,2);
% Number of edges to add
p = 12;
t = 0;
tm = 0;
te = 0;
ts = 0;
ec = 0
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startpos(1), startpos(2), 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1), searchgoal(2), 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal
hold on;

while ((~done) && (t < timeLimit))
    %disp('Enter Loop'); 
    t=t+1;
    % Get new milestone
    newstone = 0;
    t0 = cputime;
    while (~newstone)
        %disp('Getting milestone');
        
        if(length(milestones) < 400) %sample all first
            sample = [(xR(1)-1)*rand(1,1)+xMin(1)+1 (xR(2)-1)*rand(1,1)+xMin(2)+1];
            keep = map(int16(sample(1)),int16(sample(2)))
        else
            sample_q = [randsample(M,1) randsample(N,1)];
            while(sample_q(1) > M || sample_q(2) > N)
                sample_q = [randsample(M,1) randsample(N,1)];
            end

            sample_qnot = [2.*randn(1,1)+sample_q(1) 2.*randn(1,1)+sample_q(2)];
            while((sample_qnot(1) > M || sample_qnot(2) > N) || sample_qnot(1)<1 || sample_qnot(2)<1)
                sample_qnot = [2.*randn(1,1)+sample_q(1) 2.*randn(1,1)+sample_q(2)];
            end
            if (map(int16(sample_q(1)),int16(sample_q(2))) && ~map(int16(sample_qnot(1)),int16(sample_qnot(2))))
                keep = 0;
                sample = sample_qnot;
            elseif (map(int16(sample_qnot(1)),int16(sample_qnot(2))) && ~map(int16(sample_q(1)),int16(sample_q(2))))
                keep = 0;
                sample = sample_q;
            else
                keep = 1; 
            end
            
        end

        if (~keep)
            %disp('Got milestone'); 
            milestones = [milestones; sample];
            newstone = 1;
            figure(1); hold on;
            plot(milestones(:,1),milestones(:,2),'m.');
        end
    end
    %disp('Got Milestone'); 
    t1 = cputime;
    tm = tm+t1-t0;
    % Attempt to add closest p edges
    t2 = cputime;
    cur = length(milestones(:,1));
    for i = 1:cur-1
        d(i) = norm(milestones(cur,:)-milestones(i,:));
    end
    [d2,ind] = sort(d); %stores sorted array with their index order (closest edges)
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:min(p,length(ind))
        ec = ec + 1;
        x1 = milestones(cur,1);
        y1 = milestones(cur,2);
        x2 = milestones(ind(j),1);
        y2 = milestones(ind(j),2);
        points_2 = [x1,y1;x2,y2];
        [vec_x vec_y] = bresenham(x1,y1,x2,y2);
        ray_points = [vec_x vec_y];
        collision = false;
        for k=1:length(vec_x)
            if map(vec_x(k),vec_y(k))
                collision = true;
            end
        end
        
        %if (~CheckCollision(milestones(cur,:),milestones(ind(j),:), obsEdges))
        %(pdist(points_2,'euclidean') < 500) // this checks if points
        %within certain distance
        if (~collision)
            %disp('NO COLLISION'); 
            e(ind(j),cur) = 1;
            e(cur,ind(j)) = 1;
            plot([milestones(ind(j),1) milestones(cur,1)],[milestones(ind(j),2) milestones(cur,2)],'m');
        else
            %disp('Collision');
            e(ind(j),cur) = 0;
            e(cur,ind(j)) = 0;
        end
    end
    t3 = cputime;
    te = te + t3 - t2;
    
    % Check if a path from start to end is found
    t4 = cputime;
    [sp, sd] = shortestpath(milestones, e, 1, 2);
    path_points = []; 
    if (sd>0) %&& length(sd) > 15)
        done = 1;
        for i=1:length(sp)-1
        plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
        path_points(i:i+1,:) = [milestones(sp(i:i+1),1) milestones(sp(i:i+1),2)]
        end
    end
    t5 = cputime;
    ts = ts + t5 - t4;
end
tm
te
ts
ec

% figure(1);clf;hold on; 
% axis equal
disp('Time to find shortest path');
toc;

if(length(sp) ~= 0)
    test_xx = M544_A2(path_points); 
else
    disp('no path');
end
