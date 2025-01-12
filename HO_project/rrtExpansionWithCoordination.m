% TC: O((h × w) + n + d).  n: number of nodes in the tree, d: number of drones, (h, w): grid dimensions
    % Generating a random free cell : O(h x w)
    % Finding the nearest node in the RRT tree: O(n)
    % Checking for collisions:
        % Obstacles: O(h x w)
        % Other drones: O(d)
% SC: O(n), n is number of nodes in the tree. space for updatedTree



% Performs sinfle expansion for RRT, adding new node to the tree
% Returns updated tre with found target
function [updatedTree, justFoundTarget] = rrtExpansionWithCoordination(rrtTree, gridMap, starts, params, thisDrone, target, allTrees)
 
    
    updatedTree = rrtTree;
    justFoundTarget = false;

    % Generate a random free cell
    randPt = getRandomFreeCell(gridMap); % O(h x w)

    % Finds nearest node to a given random point randPt
    [nearestIdx, ~] = findNearestNode(rrtTree, randPt);  % O(n)
    nearestPos = rrtTree(nearestIdx, 1:2);

    % calculates the angle theta to the random point
    theta = atan2(randPt(2) - nearestPos(2), randPt(1) - nearestPos(1));

    % Using theta the newPos is computed by moving stepSize units towards the random point
    newPos = [nearestPos(1) + round(params.stepSize * cos(theta)), ...
              nearestPos(2) + round(params.stepSize * sin(theta))];

    % Check for collisions
    if isCollisionFree(nearestPos, newPos, gridMap) && ... 
            ~collidesWithOtherDrones(newPos, thisDrone, starts, allTrees, params.collisionRadius)  % O(d)

        % Adds the  new node
        newIndex = size(rrtTree, 1) + 1;
        updatedTree(newIndex, :) = [newPos(1), newPos(2), nearestIdx];

        % Check if the new position is near the target
        if norm(newPos - target) < params.thresholdDist % if the newPos within the thresholdDist of target
            justFoundTarget = true;
            updatedTree(newIndex + 1, :) = [target(1), target(2), newIndex]; % target is added as a new node in the tree, connected to the current new node.
        end
    end
end


% Time Complexity:O(n), n is the number of nodes in the tree. Because it computes the distance to all nodes in rrtTree.
% Space Complexity: O(n), n is the number of nodes in the tree. 
    % Because dx stores the differences between x-coordinates of all nodes in rrtTree and randPt(1) and it's O(n)
    % same for dy
    % dists stores the Euclidean distances between randPt and all n nodes

% Returns index of nearest node in rrtTree to new generated random free cell.
function [nearestIdx, minDist] = findNearestNode(rrtTree, randPt)
        dx = rrtTree(:,1) - randPt(1);
        dy = rrtTree(:,2) - randPt(2);
        dists = sqrt(dx.^2 + dy.^2); % Euclidean distances
        [minDist, nearestIdx] = min(dists); % Find the smallest distance
end
