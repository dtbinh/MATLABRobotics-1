%% Planning map

I = imread('CustomMapB.png');
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startpos = [1 9.5];

% Target location
searchgoal = [16.5 4];

% Plotting
figure(1); clf; hold on;
axis([ 0 200 0 100])

colormap('gray');
imagesc(1-map');
plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1)/dxy, searchgoal(2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal