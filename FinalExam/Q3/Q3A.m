%% Probabilistic Road Map and Motion Primitives
clear all;
clc;
close all;

%{ 

   Author: Michal Kaca

- Two wheeled robot
- Robot can only turn on the spot OR drive in straight line
        - SO: w and v can't both be non zero at the same time
- map is given yay
- Start location: 1,    9.5
- end location:   16.5, 4

Overview: 

First, create map of 0s (free) and 1s (occupied)that matches the maze file

Define PRM algo, to find path from start to end
1. Sampling strategy sets a max limit of 400 randomly generated samples
within the map boundaries
2. Check if samples are in an occupied or free space, keep all the samples
in free space, and call them milestones
3. Milestones are connected using simple pythagorean thereom edges (sample 1
connects to sample 2,3,4,5,etc... and repeat for all samples)
4. For each edge connection, check using Bresenham if any discretized point
along the edge is in an occupied space. If so, disregard edge.
5. Once a closed path from start to end is found and it once it contains at
least 25 points (for accuracy), find shortest path using A* search 
algorithm.
If closed path is not found, add more samples, and repeat algorithm. If
number of samples exceeds 400, remove old samples to avoid clustering and
lag
6. Finally plot the path 


%}

%% Problem parameters
tic;

%time limit
timeLimit = 500;

%Max milestones:
maxMilestones = 400;

% Map size = 20 x 10m --> make each grid cell be 0.1m x 0.1m

I = imread('CustomMapB.png');
%I = imgaussfilt(I,9);  % blurs image to account for error
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startpos = [1/dxy 9.5/dxy];

% Target location
searchgoal = [16.5/dxy 4/dxy];

% Set up the map
xMax = [M N]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = startpos;
xF = searchgoal;
 
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
        
        if(length(milestones) < maxMilestones) %sample all first
            sample = [(xR(1)-1)*rand(1,1)+xMin(1)+1 (xR(2)-1)*rand(1,1)+xMin(2)+1];
            keep = map(int16(sample(1)),int16(sample(2)));
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
    if (sd>0)
        done = 1;
        for i=1:length(sp)-1
            plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',5);
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

%figure(1);clf;hold on; 
% axis equal
disp('Time to find shortest path');
toc;

% plot shortest path


if(length(sp) ~= 0)
    disp('yay a path is found'); 
else
    disp('no path');
end
