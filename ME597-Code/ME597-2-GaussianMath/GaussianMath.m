%% Gaussian Math

% Distributions
n = 100000;
x = randn(n,1);
y = 2*randn(n,1);

% Math
z1 = x+y;
z2 = x.*y;
z3 = x./y;

% Histogram fit

figure(1); clf; hold on;
histfit(z1);
xlim([-10 10])

figure(2); clf; hold on;
histfit(z2);
xlim([-10 10])

figure(3); clf; hold on;
histfit(z3,10000);
xlim([-200 200])
