%% Probabilistic Road Map example
clear; clc;

%% Problem parameters
tic;

% Map size = 20 x 10m --> make each grid cell be 0.1m x 0.1m

I = imread('CustomMapB.png');
%I = imgaussfilt(I,9);  % blurs image to account for error
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startpos = [1/dxy 9.5/dxy degtorad(-90)];

% Target location
searchgoal = [16.5/dxy 4/dxy degtorad(180)]; % final orientation doesn't matter

% Set up the map
xMax = [M N]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = startpos;
xF = searchgoal;

%obstBuffer = 1; % Buffer space around obstacles
%maxCount = 1000; % Iterations to search for obstacle locations

env = map;

% Plot map
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startpos(1), startpos(2), 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1), searchgoal(2), 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal

disp('Time to create environment');
toc;

%% Vehicle
dt = 0.1;
speedV = 2;      % m/s
speedW = 0.7854; % rad/s
speedVCirc = 1.57; % m/s
initOrientation = x0(3);

%% Multi-query PRM, created until solution found
tic;
done = 0;
milestones = [x0];
nM = 1;
t= 0;

while ((~done) && (t < 100))
    t=t+1;
    % Select node to expand and creates its 5 neighbours
    % Uniform curstone = max(1,min(nM,round(nM*rand(1,1)))) Weighted on distance to goal
    for i=1:nM
        curAll = milestones(i);
        curX = curAll(1);
        curY = curAll(2);
        if (nM ==1)
            cOrient = initOrientation;
        else
            cOrient = curAll(3);
        end
        if (cOrient == -1.5708) % down
            distance = rand(0,1.414); % creates random distance for straight path
            sampleRight = [curX+distance curY 0];
            sampleLeft = [curX-distance curY 1.5708*2];
            sampleDown = [curX curY-distance cOrient];
            potSamples = [sampleRight sampleLeft sampleDown]; % potential samples
            
        elseif (cOrient == 1.5708) % up
            fdfd
        elseif (cOrient == 0) % right
            dsgds
        elseif (cOrient == abs(1.5708*2))
            dfdf
        else
            disp ("ANGLE IS WRONG");
        end
        samples = [];
        for (j=1:len(potSamples))
            curSample = potSamples(j);
            % now we check boundaries
            if (curSample(1) > xMin(1) && curSample(1) < xMax(1) && ...
                    curSample(2) > xMin(2) &&  curSample(2) < xMax(2) && ...
                    env(curSample(1),curSample(2)) == 0)
                samples = [samples; curSample];
            end
        end
        
    end  % iteration through all milestones ends here
        
    
    done = 1;
end
        
        
%{     
        
        
        
        
        
        
        d(i) = norm(milestones(i,1:2)-xF);
    end
    [ds,ind] = sort(d);
    w(ind) = exp(-0.1*[1:nM]);
    W = cumsum(w);
    seed = W(end)*rand(1);
    curstone = find(W>seed,1);

    % Create a target location using Gaussian sampling
    newgoal = 0;
    s = 0;
    while (~newgoal)
        s = s+1;
        % Gaussian about milestone to be expanded
        curgoal = milestones(curstone, 1:2)' + QE*sqrt(Qe)*randn(2,1);
        % Uniform box about milestone to be expanded
%         curgoal = milestones(curstone, 1:2)' + 4*rand(2,1)-2;
        if (env(round(curgoal(1)),round(curgoal(2))) == 0)
            keep = 1;
        else
            keep = 0;
        end
        %keep = inpolygon(curgoal(1), curgoal(2), env(:,1),env(:,2));
        if (keep)
            newgoal = 1;
        end
    end
     
    % Get new control input and trajectory
    newstone = 0;
    s = 0;
    while (~newstone)
        s=s+1;
        if (s == 50)
            break;
        end
        input = [uR(1)*rand(1,1)+uMin(1) uR(2)*rand(1,1)+uMin(2)];
        steps = sR*rand(1,1)+sMin;
        samples = milestones(curstone,1:3);
        % Dynamics
        for i=2:steps
            samples(i,:) = samples(i-1,:)+[input(1)*cos(samples(i-1,3))*dt input(1)*sin(samples(i-1,3))*dt input(2)*dt]; 
        end
        % Check for not achieving end goal
        if (norm(samples(end,1:2)'-curgoal) > 1)
            continue;
        end
        
        % Check for collisions
        keep = inpolygon(samples(:,1), samples(:,2), env(:,1),env(:,2));
        
        if (sum(keep)==length(samples(:,1)))
            milestones = [milestones; samples(end,:) curstone];
            newstone = 1;
            nM = nM+1;
            plot(samples(:,1),samples(:,2),'m');
            plot(milestones(end,1),milestones(end,2),'mo');
            F(f) = getframe(gcf);
            f=f+1;
        else
            disp('check bad');
        end
    end
    % Check if a path from start to end is found
    if (norm(milestones(end,1:2)-xF)<1)
        done = 1;
    end
end

% Find and plot final path through back tracing
done = 0;
cur = nM
curC = milestones(nM,:);
prev = curC(4);
i=2;
p=1;
dtot= 0;
nMiles = 0;
while (~done)
    if (prev == 1)
        done = 1;
    end
    plot([milestones(prev,1) milestones(cur,1)], [milestones(prev,2) milestones(cur,2)],'go','MarkerSize',6, 'LineWidth',2)
    dtot = dtot + norm(milestones(prev,1:2)-milestones(cur,1:2));
    nMiles = nMiles+1;
    F(f) = getframe(gcf);
    f=f+1;
    cur = prev;
    curC = milestones(cur,:);
    prev = curC(4);
    p=p+1;
end
        
%}
        
disp('Time to find a path');
toc;