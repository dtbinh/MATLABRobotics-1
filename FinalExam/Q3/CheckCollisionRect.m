function [ inCollision, edge ] = CheckCollisionRect( edge, rectangle )
%CHECKCOLLISION Checks if a line interesects with a rectangle
% Takes in an edge ([ x1 y1 x2 y2]) and checks for a collision with a
% rectangle ([ x0 y0 width height]).

% Define rectangle edges
x0 = rectangle(1);
y0 = rectangle(2);
w = rectangle(3);
h = rectangle(4);

obstEdges = [ x0 y0 x0+w y0;
              x0+w y0 x0+w y0+h;
              x0+w y0+h x0 y0+h;
              x0 y0+h x0 y0];
ptA = edge(1:2);
ptB = edge(3:4);

% Check for each edge
edge = [];
for k = 1:4
    % If both vertices aren't to the left, right, above, or below the
    % edge's vertices in question, then test for collision
    if ~((max([ptA(1),ptB(1)])<min([obstEdges(k,1),obstEdges(k,3)])) || ...
         (min([ptA(1),ptB(1)])>max([obstEdges(k,1),obstEdges(k,3)])) || ...
         (max([ptA(2),ptB(2)])<min([obstEdges(k,2),obstEdges(k,4)])) || ...
         (min([ptA(2),ptB(2)])>max([obstEdges(k,2),obstEdges(k,4)])))
        if (EdgeCollision([ptA, ptB], obstEdges(k,:)))
            % Eliminate end-to-end contacts from collisions list
            if (sum(abs(ptA-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptB-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptA-obstEdges(k,3:4)))>0 && ...
                sum(abs(ptB-obstEdges(k,3:4)))>0)
            
                edge = k;
                inCollision = 1 ; % In Collision
                return
            end
        end
    end
end
inCollision = 0 ; % Not in collision