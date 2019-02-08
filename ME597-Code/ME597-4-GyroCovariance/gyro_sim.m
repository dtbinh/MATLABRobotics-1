% MN: This code shows the importance of filtering the simulated gyro data.
% MN: After filtering the simulated gyro data based on a moving average of size 3, the
% MN: simulated filtered data more closely resembles the non-simulated
% MN: data. This is because the covariance of the filtered data is a lot less
% MN: than the original covariance
% MN: We also plot the error data and the gaussian approximation of the
% MN: error

% Gyro data reproduction

load('gyro.mat');

% Convert to degrees
gx = 180/pi*gx;
gy = 180/pi*gy;
gz = 180/pi*gz;

% Assume underlying motion is negligible, and measurements are independent
S = cov([gx gy gz]);                        % MN: If given a vector of data, cov returns the 
                                            % MN: corresponding covariance
                                            % matrix
[SE,Se] = eig(S);

% Create a reproduction of gyro data
g_sim = SE*sqrt(Se)*randn(3,length(gx));    % MN: Reproduces the gyro data using covariance from 
                                            % MN: existing data. This is
                                            % similar to how we create
                                            % noise models

% Plot data for a certain time
tmin = 10;
tmax = 20;
t1= find(tg>tmin,1);                        % MN: returns the first index that is larger than tmin
t2= find(tg>tmax,1);                        % MN: returns the first index that is larger than tmax

figure(1);clf;
subplot(2,1,1); hold on;
plot(tg(t1:t2), gx(t1:t2), 'b--');
plot(tg(t1:t2), gy(t1:t2), 'r--');
plot(tg(t1:t2), gz(t1:t2), 'g--');
axis([tmin tmax -60 60])
title('Original Data')
xlabel('Time(s)')
ylabel('Rate (deg/s)')
subplot(2,1,2); hold on;
plot(tg(t1:t2), g_sim(1,(t1:t2)), 'b');
plot(tg(t1:t2), g_sim(2,(t1:t2)), 'r');
plot(tg(t1:t2), g_sim(3,(t1:t2)), 'g');
axis([tmin tmax -60 60]);
xlabel('Time(s)')
ylabel('Rate (deg/s)')
title('Unfiltered simulation')


% Moving average filter and subtract average from measurments
f_len = 3;
gxf = filter(ones(1,f_len)/f_len,1,gx);             % MN: filters the data in gx, gy, and gz by
gyf = filter(ones(1,f_len)/f_len,1,gy);             % MN: moving average of size 3 define by the 
gzf = filter(ones(1,f_len)/f_len,1,gz);             % MN: first 2 vectors

ex = gx-gxf;                                        % MN: obtain the error between measurements
ey = gy-gyf;                                        % MN: and the average
ez = gz-gzf;

Sf = cov([ex ey ez]);                               % MN: find the covariance on this error
[SfE, Sfe]=eig(Sf);

% Create a reproduction of gyro data
g_simf = SfE*sqrt(Sfe)*randn(3,length(gxf));

% Plot data for a certain time
tmin = 10;
tmax = 20;
t1= find(tg>tmin,1);
t2= find(tg>tmax,1);

figure(2);clf;
subplot(2,1,1); hold on;
plot(tg(t1:t2), gx(t1:t2), 'b--');
plot(tg(t1:t2), gy(t1:t2), 'r--');
plot(tg(t1:t2), gz(t1:t2), 'g--');
axis([tmin tmax -60 60])
title('Original Data')
xlabel('Time(s)')
ylabel('Rate (deg/s)')
subplot(2,1,2); hold on;
subplot(2,1,2); hold on;
plot(tg(t1:t2), gxf(t1:t2)'+g_simf(1,t1:t2), 'b');
plot(tg(t1:t2), gyf(t1:t2)'+g_simf(2,t1:t2), 'r');
plot(tg(t1:t2), gzf(t1:t2)'+g_simf(3,t1:t2), 'g');
axis([tmin tmax -60 60]);
title('Filtered simulation')
xlabel('Time(s)')
ylabel('Rate (deg/s)')

figure(3); clf; hold on
bins = 100;
[Nex, Xex] = hist(ex,bins);
binsize = Xex(2)-Xex(1);
xvec = -50:.1:50;
% Compare to pdf, by multiplying by number of samples and by bin size
% Num samples to convert to frequency distribution and bin size as smaller
% bins will have fewer samples.

%Gex = normpdf(xvec,muf(1),sqrt(Sf(1,1)))*length(ex)*binsize;  
Gex = normpdf(xvec,0,sqrt(Sf(1,1)))*length(ex)*binsize; 

% MN: returns the pdf of the normal distribution with mean MU and the
% MN: standard deviation SIGMA evaluated at the values in xvec 
bar(Xex,Nex);
plot(xvec,Gex,'r', 'LineWidth',2);
title('Comparison of Error Data and Gaussian Approximation')
xlabel('Error Value')
ylabel('Histogram Count')

