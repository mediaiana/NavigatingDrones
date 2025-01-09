function shareTarget(rrtTrees, target)
    % SHARETARGET Adds the target node to all RRT trees for all drones.
    %
    %   shareTarget(rrtTrees, target)
    %
    % Inputs:
    %   rrtTrees - cell array of RRT trees for each drone
    %   target   - [x, y] coordinates of the target
    
    for d = 1:length(rrtTrees)
        currentTree = rrtTrees{d};
        % Add the target to the current drone's RRT tree if not already there
        if isempty(currentTree) || ~any(ismember(currentTree(:, 1:2), target, 'rows'))
            newIndex = size(currentTree, 1) + 1;
            % Add the target as a node with no parent (or with a dummy parent index)
            currentTree(newIndex, :) = [target(1), target(2), -1];
            rrtTrees{d} = currentTree;
        end
    end
end
