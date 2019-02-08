%% Random Noise simulation
clear;

Q = [1 -1; -1 2]; % row1; row2; row 3; ...etc.
n = length(Q(:,1));
[QE, Qe] = eig(Q)

RE(:,1) = QE(:,2);
RE(:,2) = QE(:,1);
Re(1,1) = Qe(2,2);
Re(2,2) = Qe(1,1);

S =  10000;
for i = 1:S
    ra(:,i) = randn(n,1);
    q(:,i) = QE*sqrt(Qe)*ra(:,i);
    r(:,i) = RE*sqrt(Re)*ra(:,i);
end

figure(1); clf; hold on;
plot(q(1,:), q(2,:),'g.')
mu = [0 0];
error_ellipse(Q(1:2,1:2),mu,0.75);
error_ellipse(Q(1:2,1:2),mu,0.95);
axis equal

figure(2); clf; hold on;
plot(r(1,:), r(2,:),'g.')
mu = [0 0];
error_ellipse(Q(1:2,1:2),mu,0.75);
axis equal


