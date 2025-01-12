% Checks if the line from posA to posB is free from obstacles.
% Uses simplified Bresenham approach.
% Output:
%   returns true if no obstacle is detected in the line
%   returns false if there is an obstacle in any part or it goes out of bounds.
function freePath = isCollisionFree(posA, posB, gridMap)
    
        freePath = true;
        x0 = posA(1); y0 = posA(2);
        x1 = posB(1); y1 = posB(2);

        % These are variables set up for Bresenham's line algorithm.
        % dx, dy: Determine how far the line moves horizontally and vertically.
        % sx, sy: Indicate the direction of movement (+1 for right/up, -1 for left/down).
        % err: Keeps track of whether to adjust the x or y coordinate during each step.
        dx = abs(x1 - x0);
        sx = sign(x1 - x0);
        dy = -abs(y1 - y0);
        sy = sign(y1 - y0);
        err = dx + dy;
    
        x = x0; y = y0;
        [h, w] = size(gridMap);
    
        while true
            % if line goes outside of the bounds, then path is not valid
            if x < 1 || x > w || y < 1 || y > h
                freePath = false;
                return;
            end
            % if current gridcell contains obstacle, then path is not valid
            if gridMap(y, x) == 1
                freePath = false;
                return;
            end
            % if the line has reached the end point posB, the loop terminates.
            if x == x1 && y == y1
                break;
            end
            % Updating Position Using Bresenham's Algorithm

            e2 = 2*err; %determines whether to move horizontally, vertically, or both.
            % according to e2 value  the err and and current position (x, y) is updated 
            if e2 >= dy
                err = err + dy;
                x   = x + sx;
            end

            if e2 <= dx
                err = err + dx;
                y   = y + sy;
            end
        end
    end
    