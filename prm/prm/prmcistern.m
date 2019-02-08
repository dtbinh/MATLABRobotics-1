%% Probabilistic Road Map example
clear; clc;


%% Problem parameters
tic;

% Set up the map
xMax = [100 100]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = [30 85 pi];
xF = [80 75];

%Load map
load cisternmap.csv
map = cisternmap';

figure(1); clf; hold on;
image(100*(1-cisternmap));
plot(x0(1),x0(2),'bx', 'MarkerSize',6,'LineWidth',2)
plot(xF(1),xF(2),'ro', 'MarkerSize',6,'LineWidth',2)
colormap(gray);
axis([0 100 0 100]);
break;

%% Vehicle
dt = 0.1;
uMin = [0 -pi/4];
uMax = [5 pi/4];
uR = uMax-uMin;
sMin = 30;
sMax = 100;
sR = sMax-sMin;


sMin = 30;
sMax = 100;
sR = sMax-sMin;

%% Multi-query PRM, created until solution found
tic;
done = 0;
milestones = [x0 0];
nM = 1;
t= 0;
f = 1;
while ((~done) && (t < 300))
    t=t+1;
    % Select node to expand
    % Uniform
    % curstone = max(1,min(nM,round(nM*rand(1,1))))
    % Weighted on distance to goal
    for i=1:nM
        d(i) = norm(milestones(i,1:2)-xF);
    end
    [ds,ind] = sort(d);
    w(ind) = exp(-0.1*[1:nM]);
    W = cumsum(w);
    seed = W(end)*rand(1);
    curstone = find(W>seed,1);

    % Get new control input and trajectory
    newstone = 0;
    s = 0;
    while (~newstone && (s < 10))
        s=s+1;
        input = [uR(1)*rand(1,1)+uMin(1) uR(2)*rand(1,1)+uMin(2)];
        steps = sR*rand(1,1)+sMin;
        samples = milestones(curstone,1:3);
        % Dynamics
        for i=2:steps
            samples(i,:) = samples(i-1,:)+[input(1)*cos(samples(i-1,3))*dt input(1)*sin(samples(i-1,3))*dt input(2)*dt]; 
        end
        % Collision check
        collide = 0;
        for i=1:steps
            cx = round(samples(i,1));
            cy = round(samples(i,2));
            if (cx < 1) || (cx > 100) || (cy < 1) || (cy > 100)
                collide = 1;
            elseif (map(cx,cy)==1)
                collide = 1;
            end
        end
        
        if (collide==0)
            milestones = [milestones; samples(end,:) curstone];
            newstone = 1;
            nM = nM+1;
            plot(samples(:,1),samples(:,2),'m');
            plot(milestones(end,1),milestones(end,2),'mo');
            F(f) = getframe(gcf);
            f=f+1;
        end
    end
    % Check if a path from start to end is found
    if (norm(milestones(end,1:2)-xF)<1)
        done = 1;
    end
end
break;
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
disp('Time to find a path');
toc;
