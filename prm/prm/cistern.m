%% ME 595 Final Exam - Fall 2009
% Cistern definitions for Question #3

load cisternmap.csv
startpos = [30 85];
endpos = [80 75];

figure(1);clf; hold on;
image(100*cisternmap);
colormap(gray);
plot(startpos(1), startpos(2), 'go')
plot(endpos(1), endpos(2), 'rx')
axis([ 0 100 0 100])
