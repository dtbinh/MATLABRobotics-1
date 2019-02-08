% Extended Kalman Filter Localization example
clear;clc;

%%%% THIS IS WAY FASTER THAN PF LOCALIZATION

% Time
Tf = 20;
dt = 0.1; % time step size
T = 0:dt:Tf; % discretized time vector 

% Initial State
x0 = [0 0 0]';

% Prior
mu = [0 0 0]'; % mean (mu)
S = 0.1*eye(3);% covariance (Sigma)

% Control inputs
u = ones(2, length(T));
u(2,:)=0.3*u(2,:);

% Disturbance model
R = [1e-4 0 0; 
     0 1e-4 0; 
     0 0 1e-5];
[RE, Re] = eig(R);

% error thingy
% Q = uncertainty in sensor observations

 Q = [0.05 0; 
      0 0.05];
[QE, Qe] = eig(Q); % error
% Sensor footprint
rmax = 10 ; % meters
thmax = pi/4; % rads

% Feature Map
% create random features here and plot 
% using randi([min max],size,Ntime)
map = [];
for i=1:100
    map = [map; randi([-10 10],1) randi([0 20], 1)];
end
M = length(map);


% Simulation Initializations
n = length(x0);
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T)); % mean proposal sigma
mu_S = zeros(n,length(T));  % mean sigma
mf = zeros(2,1);    % this is the landmark that we see (x, y)

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    E = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = [x(1,t-1)+u(1,t)*cos(x(3,t-1))*dt;
              x(2,t-1)+u(1,t)*sin(x(3,t-1))*dt;
              x(3,t-1)+u(2,t)*dt] + E;
          
    % modifying this in progress
    %x(:,i) = x(:,t-1) + [dt*(v)*cos(x(3,t-1)) ; dt*(v)*sin(x(3,t-1))  ; dt*(1/l)*(v)*tan(steerAngleRad)] + E;

    
    % Take measurement
    % Identify features that are in view
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(map(i,:),x(:,t),rmax,thmax)) % checks if landmark is in view
            flist(i) = 1;
            mf = map(i,:);
            % Select a motion disturbance
            d = QE*sqrt(Qe)*randn(m,1);
            % Determine measurement
            y(2*(i-1)+1:2*i,t) = [max(0.001, sqrt((mf(1)-x(1,t))^2 + (mf(2)-x(2,t))^2));
                atan2(mf(2)-x(2,t),mf(1)-x(1,t))-x(3,t)] + d;
        end
    end

    %% Extended Kalman Filter Estimation
    % Prediction update
    mup =    [mu(1)+u(1,t)*cos(mu(3))*dt;
              mu(2)+u(1,t)*sin(mu(3))*dt;
              mu(3)+u(2,t)*dt];
    
    % this is our linearized function Gt, 
    Gt = [ 1 0 -u(1,t)*sin(mu(3))*dt;
           0 1 u(1,t)*cos(mu(3))*dt;
           0 0 1];
    
    Sp = Gt*S*Gt' + R;  % this updates our prediction step 

    % Store results
    mup_S(:,t) = mup;

    
    % Linearization
    for i=1:M
        if (flist(i))
            mf = map(i,:);  % mf is what landmark we see
            rp = sqrt((mf(1)-mup(1))^2+(mf(2)-mup(2))^2); % distance from predicted position to landmark position
            
            % this linearizes our C --> H for the correction step
            
            Ht = [ -(mf(1)-mup(1))/rp ...
                -(mf(2)-mup(2))/rp ...
                0;
                (mf(2)-mup(2))/rp^2 ...
                -(mf(1)-mup(1))/rp^2 ...
                -1]; ...
            % Measurement update
            K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);  % formula for getting EKF Kalman gain

            I = y(2*(i-1)+1:2*i,t)-[sqrt((mf(1)-mup(1))^2+(mf(2)-mup(2))^2);
                (atan2(mf(2)-mup(2),mf(1)-mup(1)) - mup(3))];
            I(2) = mod(I(2)+pi,2*pi)-pi;
            Inn(:,t) = I;
            
            if (norm(I) > 10)
                keyboard;
            end
            mu = mup + K*I;  % update real control param with proposed controls + innovation
            % also, I --> corrected observation 
            S = (eye(n)-K*Ht)*Sp;  % update real Sigma 
        end
    end
    if (sum(flist)==0) mu = mup; end
    mu_S(:,t) = mu;


    %% Plot results
    figure(1);clf; hold on;
    axis equal
    axis([-4 6 -1 7])
    plot(x(1,1:t),x(2,1:t), 'ro--')  % actual path
    plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);  % landmarks
    for i = 1:M
        if (flist(i))
            plot(map(i,1),map(i,2),'mx', 'MarkerSize',10,'LineWidth',2)
            plot([x(1,t) x(1,t)+y(2*(i-1)+1,t)*cos(y(2,t)+x(3,t))], [ x(2,t) x(2,t)+y(2*(i-1)+1,t)*sin(y(2*i,t)+x(3,t))], 'c');
        end
    end
    plot(mu_S(1,1:t-1),mu_S(2,1:t-1), 'bx--')
    plot(mup_S(1,1:t),mup_S(2,1:t), 'go--') % predicted path
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.95);
    error_ellipse(S_pos,mu_pos,0.999);
    title('Range & Bearing Measurements for Localization')
    drawnow;

    %plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
    %plot(mf(1,t),mf(2,t),'mx', 'MarkerSize',10,'LineWidth',2)
    %plot(x(1,1:t),x(2,1:t), 'ro--')
    %if (meas==1) circle(1,x(1:2,t), y(1,t)); end
    %if (meas==2) plot([x(1,t) x(1,t)+10*cos(y(1,t)+x(3,t))], [ x(2,t) x(2,t)+10*sin(y(1,t)+x(3,t))], 'c');end
    %if (meas==3) plot([x(1,t) x(1,t)+y(1,t)*cos(y(2,t)+x(3,t))], [ x(2,t) x(2,t)+y(1,t)*sin(y(2,t)+x(3,t))], 'c');end
    %plot(mup_S(1,1:t),mup_S(2,1:t), 'go--')
    %mu_pos = [mu(1) mu(2)];

end

figure(2);clf; hold on;
plot(T, mu_S)
plot(T,Inn)
title('Mean and Innovation')