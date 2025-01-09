function collision = collidesWithOtherDrones(newPos, thisDrone, starts, allTrees, radius)
    % Check for collisions with other drones using manual distance calculation

    collision = false;
    numDrones = length(allTrees);

    for d = 1:numDrones
        if d == thisDrone
            continue;  % Skip self
        end
        otherTree = allTrees{d};
        if isempty(otherTree)
            continue;
        end
        % Get the most recent position of the other drone
        otherPos = otherTree(end, 1:2);

        % Calculate Euclidean distance manually
        distance = sqrt((newPos(1) - otherPos(1))^2 + (newPos(2) - otherPos(2))^2);

        % Check if the distance is less than the collision radius
        if distance < radius
            collision = true;
            return;
        end
    end
end
