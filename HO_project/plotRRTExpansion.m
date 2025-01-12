% Time Complexity: O(d⋅n), where d is number of Drones and n is the average number of nodes in the tree
% Because we have 2 loops: outer loop goes through each drone and inner one loops through each node in the tree for each node

% Space Complexity: O(1), because it doesn't use extra large amount of memory

% Plots the trees for each drone at a given moment
function plotRRTExpansion(rrtTrees, colors)
    
        hold on;  % To make sure that the plots of all drones are drawn on the same figure
        numDrones = length(rrtTrees);

        %loop through each drone
        for d = 1:numDrones
            tree = rrtTrees{d};
            % skip if it has lees than 2nodes
            if size(tree,1) < 2
                continue; 
            end
            %starting from 2nd node
            for n = 2:size(tree, 1)
                % get current node's coordinates
                px   = tree(n, 1);
                py   = tree(n, 2);
                parIdx = tree(n, 3);
                if parIdx > 0
                    % get parent node's coordinates
                    ppx = tree(parIdx, 1);
                    ppy = tree(parIdx, 2);

                    % Plot a line between the current node and its parent 
                    plot([px, ppx], [py, ppy], '-', ...
                         'Color', colors(d,:), 'LineWidth', 1.5);
                end
            end
        end
end
    