function freePath = isCollisionFree(posA, posB, gridMap)
    % ISCOLLISIONFREE Check if the line from posA->posB is free of obstacles.
    %   Uses a simple Bresenham approach in integer grid.
    %
    %   freePath = isCollisionFree(posA, posB, gridMap)
    %
    % Inputs:
    %   posA   : [x0, y0]
    %   posB   : [x1, y1]
    %   gridMap: obstacle map (0=free, 1=obstacle)
    %
    % Output:
    %   freePath: boolean, true if no obstacle is encountered along the line
    
        freePath = true;
        x0 = posA(1); y0 = posA(2);
        x1 = posB(1); y1 = posB(2);
    
        dx = abs(x1 - x0);
        sx = sign(x1 - x0);
        dy = -abs(y1 - y0);
        sy = sign(y1 - y0);
        err = dx + dy;
    
        x = x0; y = y0;
        [h, w] = size(gridMap);
    
        while true
            % Bounds check
            if x < 1 || x > w || y < 1 || y > h
                freePath = false;
                return;
            end
            % Obstacle check
            if gridMap(y, x) == 1
                freePath = false;
                return;
            end
            % Check if done
            if x == x1 && y == y1
                break;
            end
    
            e2 = 2*err;
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
    