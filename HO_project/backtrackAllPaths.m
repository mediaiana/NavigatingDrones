function [allPaths, finalPositions] = backtrackAllPaths(rrtTrees)
    % BACKTRACKALLPATHS For each drone, backtrack from last node in its tree to root.
    %
    %   [allPaths, finalPositions] = backtrackAllPaths(rrtTrees)
    %
    % Outputs:
    %   allPaths       - cell array, each cell is a Nx2 matrix of path coordinates
    %   finalPositions - Nx2 array of the final coordinate of each drone
    
        numDrones = length(rrtTrees);
        allPaths = cell(numDrones, 1);
        finalPositions = zeros(numDrones, 2);
    
        for d = 1:numDrones
            tree = rrtTrees{d};
            if isempty(tree)
                allPaths{d} = [];
                continue;
            end
    
            % The last row is presumably the final node
            idx = size(tree, 1);
            path = [];
    
            while idx > 0
                node = tree(idx, 1:2);
                path = [node; path]; %#ok<AGROW>
                idx = tree(idx, 3);
            end
    
            allPaths{d} = path;
            finalPositions(d,:) = tree(end, 1:2);
        end
    end
    