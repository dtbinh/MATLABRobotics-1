% Laser scanner inverse measurement model plot
clc; clear;

M = 50;
N = 60;
m = 0.5*ones(M,N); %map                             % MN: equal probability of being occupied
                                                    % MN: vs being free

alpha = 1; % Distance about measurement to fill in  % MN: when it fills up the cells, this is the
                                                    % MN: best resolution
                                                    % MN: it can have for
                                                    % MN: checking. Since 1
                                                    % MN: grid is is 1x1
beta = 0.01; % Angle beyond which to exclude 

% Robot location
x = 25;
y = 10;
theta = pi/2;
rmax = 80; 

% Measurements
meas_phi = [-.4:0.01:.4]; % heading

% These are the actual measured values (from laser scanner)
% currently are all set to 40 for visualization purposes
meas_r = 40*ones(size(meas_phi)); % range 

m = inversescanner(M,N,x,y,theta,meas_phi,meas_r,rmax,alpha,beta);

figure(2);clf;hold on;
image(100*(1-m));
plot(y,x,'kx','MarkerSize',8,'LineWidth',2)
% for i=1:length(meas_r)
%     plot( y+meas_r(i)*sin(meas_phi(i) + theta),x+meas_r(i)*cos(meas_phi(i)+ theta),'ko')
% end
colormap('gray')
axis equal