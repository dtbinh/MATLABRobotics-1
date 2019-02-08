%% MTE-544 Final Exam - Fall 2018
% Maze definitions for Question #3

% Walls are defined by bottom left corner, width and height
walls = [ 2 0 1 2;
          2 3 1 7;
          6 0 1 4;
          7 3 3 1;
          7 6 4 4;
          12 0 1 4;
          14 7 1 3;
          15 7 3 1;
          15 2 5 1;
          15 3 1 3;
          16 5 2 1];

% Start and end points
startpos = [1 9.5];
endpos = [16.5 4];

% Plot of the maze environment
figure(1);clf; hold on;
for i = 1:length(walls)
    rectangle('Position', walls(i,:), 'FaceColor', 'b', 'EdgeColor', 'none');
end
rectangle('Position', [ 0 0 20 10], 'EdgeColor', 'b', 'LineWidth', 2);
plot(startpos(1), startpos(2), 'gx', 'LineWidth', 2, 'MarkerSize', 10)
plot(endpos(1), endpos(2), 'rx', 'LineWidth', 2, 'MarkerSize', 10)
axis equal
axis([ 0 20 0 10])

% Basic collision check using walls definition
edge = [ 0 0 3 4];
for i = 1:length(walls)
    [collides(i)] = CheckCollisionRect(edge,walls(i,:));
end
% Output collision occurrance
collides