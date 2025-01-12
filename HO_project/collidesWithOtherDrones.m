% Time Complexity: O(d), where d is number of Drones.
    %Because we the loop through numDrones in allTrees, which is O(n) and  
    %getting last position and calculating Euclidean distance are O(1)

% Space Complexity: O(1), because it doesn't use additional memory

% Checks if drones newPos will collide with other drones using  Euclidean distance 
function collision = collidesWithOtherDrones(newPos, thisDrone, starts, allTrees, radius)

    collision = false;
    numDrones = length(allTrees);

    for d = 1:numDrones
        if d == thisDrone 
            continue;  %don't check collision with itself
        end
        otherTree = allTrees{d};

        if isempty(otherTree)% if drone hasn't moved yet, skip it
            continue;
        end
        otherPos = otherTree(end, 1:2);  % Get the last t position of other drone ( O(1) )

        % Calculate Euclidean distance between newPos and otherPos (another drone) ( O(1) )
        distance = sqrt((newPos(1) - otherPos(1))^2 + (newPos(2) - otherPos(2))^2);

        % If distance is less than radius, collision detected
        if distance < radius
            collision = true;
            return;
        end
    end
end
