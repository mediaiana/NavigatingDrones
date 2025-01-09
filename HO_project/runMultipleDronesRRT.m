function [finalPositions,droneThatFoundPosition] = runMultipleDronesRRT()
    clc; clear; rng('shuffle');

    %% Parameters
    params.gridHeight      = 70;
    params.gridWidth       = 70;
    params.obstacleDensity = 0.15;
    params.numDrones       = 4;
    params.maxIterations   = 2000;
    params.stepSize        = 3;
    params.thresholdDist   = 1.5;
    params.collisionRadius = 2;

    % Create results folder
    resultsFolder = 'results';
    if ~exist(resultsFolder, 'dir')
        mkdir(resultsFolder);
    end

    % Generate a clustered or variable-density grid
    numClusters = 10;  % For clustered grid
    gridMap = readmatrix("maze\gridMap.txt");

    % Add paths through the grid
    numPaths = 3;  % Number of trails
    gridMap = addPaths(gridMap, numPaths);

    % Visualize the forest
    plotForest(gridMap);


    %% Initialize Drones and Target
    starts = zeros(params.numDrones, 2);
    for d = 1:params.numDrones
        starts(d, :) = getRandomFreeCell(gridMap);
    end
    target = getRandomFreeCell(gridMap);

    % Initialize RRT trees
    rrtTrees = cell(params.numDrones, 1);
    for d = 1:params.numDrones
        rrtTrees{d} = [starts(d, 1), starts(d, 2), -1];
    end

    foundTarget = false;
    droneThatFound = -1;

    %% Visualization
    figure('Name', 'Multi-Drone RRT Search');
    axis equal; hold on;
    set(gca, 'YDir', 'reverse');
    xlabel('x'); ylabel('y');
    title('Multi-Drone RRT Search');

    colors = lines(params.numDrones);
    plotObstacles(gridMap);
    plotInitialPositions(starts, colors);
    plotTarget(target);

    % Video Writer
    videoFileName = fullfile(resultsFolder, 'RRT_Search_Animation.mp4');
    video = VideoWriter(videoFileName, 'MPEG-4');
    open(video);

    %% Main Search Loop
    for iter = 1:params.maxIterations
        for d = 1:params.numDrones
            if foundTarget
                shareTarget(rrtTrees, target);
                break;
            end

            % RRT Expansion
            [rrtTrees{d}, success] = rrtExpansionWithCoordination(...
                rrtTrees{d}, gridMap, starts, params, d, target, rrtTrees);

            if success
                foundTarget = true;
                droneThatFound = d;
            end
        end

        % Visualization
        if mod(iter, 10) == 0
            plotRRTExpansion(rrtTrees, colors);
            drawnow;
            %frame = getframe(gcf);
            %ideo(video, frame);
        end

        if foundTarget
            break;
        end
    end
    
    close(video);
    fprintf('Animation saved to: %s\n', videoFileName);

    %% Results
    if foundTarget
        fprintf('Drone %d found the target at (%.0f, %.0f)!\n', ...
                droneThatFound, target(1), target(2));
        
    else
        fprintf('No drone found the target within %d iterations.\n', params.maxIterations);
    end
    

    % Backtrack paths
    [allPaths, finalPositions] = backtrackAllPaths(rrtTrees);
    disp(finalPositions);
    droneThatFoundPosition = finalPositions(droneThatFound, :);
    finalPositions(droneThatFound, :) = [];  % Removes the entire row (pair)
    disp(finalPositions);
    % Final Visualization
    figure('Name', 'Final Drone Paths');
    axis equal; hold on;
    set(gca, 'YDir', 'reverse');
    xlabel('x'); ylabel('y');
    title('Final Drone Paths');

    plotObstacles(gridMap);
    for d = 1:params.numDrones
        path = allPaths{d};
        if ~isempty(path)
            plot(path(:, 1), path(:, 2), 'LineWidth', 2, 'Color', colors(d, :), ...
                'DisplayName', sprintf('Drone %d Path', d));
        end
    end
    plot(target(1), target(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2, ...
         'DisplayName', 'Target');
    legend show;

    saveas(gcf, fullfile(resultsFolder, 'FinalPaths.png'));
    fprintf('Final paths figure saved to: %s\n', fullfile(resultsFolder, 'FinalPaths.png'));
    return;
end

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
    


%HELPER FUNCTIONS:

function gridMap = addPaths(gridMap, numPaths)
    % ADDPATHS Clears paths through the grid to mimic trails in a forest.
    %   gridMap  - Input grid (0 = free, 1 = obstacle)
    %   numPaths - Number of open paths to create

    [h, w] = size(gridMap);

    for p = 1:numPaths
        startCol = randi([1, w]);  % Random starting column
        for row = 1:h
            col = max(1, min(w, startCol + round(randn() * 2)));  % Path deviation
            gridMap(row, col) = 0;  % Clear obstacles along the path
        end
    end
end

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


function plotTarget(target)
    % PLOTTARGET Plots the target location.
    %
    %   plotTarget(target)
    %
    % Inputs:
    %   target - [x, y]
    
        plot(target(1), target(2), 'r*', ...
             'MarkerSize', 12, ...
             'LineWidth', 2, ...
             'DisplayName', 'Target');
end

function plotObstacles(gridMap)
    % PLOTOBSTACLES Plots the obstacles (cells that are 1 in gridMap).
    %
    %   plotObstacles(gridMap)
    %
    % This is a simplified approach that just draws black squares for obstacles.
    % If you want more advanced "multi-size" obstacle visualization,
    % consider a connected-components approach or a polygon-based approach.
    
        [obsRows, obsCols] = find(gridMap == 1);
        plot(obsCols, obsRows, 'ks', ...
             'MarkerFaceColor', 'k', ...
             'MarkerSize', 5, ...
             'DisplayName', 'Obstacles');
end

function plotInitialPositions(starts, colors)
    % PLOTINITIALPOSITIONS Plots the starting positions for multiple drones.
    %
    %   plotInitialPositions(starts, colors)
    %
    % Inputs:
    %   starts - Nx2 array of start positions
    %   colors - Nx3 array of colors (like from the 'lines' function)
    
        numDrones = size(starts,1);
        for d = 1:numDrones
            plot(starts(d, 1), starts(d, 2), 'o', ...
                 'MarkerFaceColor', colors(d,:), ...
                 'MarkerSize', 8, ...
                 'DisplayName', sprintf('Start %d', d));
            text(starts(d, 1) + 1, starts(d, 2), ...
                 sprintf('Drone %d', d), ...
                 'Color', 'black', ...
                 'FontSize', 10, ...
                 'FontWeight', 'bold');
        end
end
    
function pos = getRandomFreeCell(gridMap)
    % GETRANDOMFREECELL Return a random (x,y) for a free cell (0) in the grid.
    %   pos = getRandomFreeCell(gridMap)
    %
    % Output:
    %   pos = [x, y]
    
        [h, w] = size(gridMap);
        freeIdx = find(gridMap == 0);
        idx = freeIdx(randi(numel(freeIdx)));
        % Convert linear idx -> (row, col)
        row = mod(idx-1, h) + 1;
        col = floor((idx-1)/h) + 1;
    
        % We interpret x=col, y=row
        pos = [col, row];
end
      