function [updatedTree, justFoundTarget] = rrtExpansionWithCoordination(rrtTree, gridMap, starts, params, thisDrone, target, allTrees)
    % RRT Expansion with Multi-Agent Coordination
    
    updatedTree = rrtTree;
    justFoundTarget = false;

    % Generate a random free cell
    randPt = getRandomFreeCell(gridMap);

    % Find nearest node
    [nearestIdx, ~] = findNearestNode(rrtTree, randPt);
    nearestPos = rrtTree(nearestIdx, 1:2);

    % Steer toward the random point
    theta = atan2(randPt(2) - nearestPos(2), randPt(1) - nearestPos(1));
    newPos = [nearestPos(1) + round(params.stepSize * cos(theta)), ...
              nearestPos(2) + round(params.stepSize * sin(theta))];

    % Check for collisions
    if isCollisionFree(nearestPos, newPos, gridMap) && ...
            ~collidesWithOtherDrones(newPos, thisDrone, starts, allTrees, params.collisionRadius)

        % Add new node
        newIndex = size(rrtTree, 1) + 1;
        updatedTree(newIndex, :) = [newPos(1), newPos(2), nearestIdx];

        % Check if the new position is near the target
        if norm(newPos - target) < params.thresholdDist
            justFoundTarget = true;
            updatedTree(newIndex + 1, :) = [target(1), target(2), newIndex];
        end
    end
end
