%% Least squares curve fit
clear; clc;
% Main function
zt = -1:0.01:1;
Nt = length(zt);
bt = 1./(1+25*zt.^2);
figure(1);clf;hold on;
plot(zt,bt)

% Measurements
n = 100;
e = 0.1^2*randn(1,n);
z = [-1:2/(n-1):1];
b = 1./(1+25*z.^2) + e;
plot(z,b ,'ro')
legend ('Function', 'Measurements');

% Poly approx
m = 15
x = ones(m,1);
for i=1:n
    A(i,1) = 1;
    for j=2:m
        aj = A(i,j-1)*z(i);
        A(i,j) = aj;
    end
end

xe = pinv(A)*b';

be = A*xe;
plot(z,be,'gx')

% Compare functions
afit = ones(Nt,1);
for j = 2:m
    afit = [afit, afit(:,j-1).*zt'];
end
bfit = afit*xe;
plot(zt,bfit,'c')
