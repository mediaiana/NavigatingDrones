
% Simulates a multi-drone search using the Rapidly-exploring Random Tree (RRT) algorithm.
% The drones colaborate with each other to find the target avoiding obstacles and collisions with each others
% The fucntion initializes random positions for drones and target, expanding RRT for each drone. 
% The process is being visualized and results are saved 
function [finalPositions,droneThatFoundPosition] = runMultipleDronesRRT()
    %Todo: add starting points for result plotting
    clc; clear; rng('shuffle');

    %% Parameters
    params.numDrones       = 4;
    params.maxIterations   = 2000;
    params.stepSize        = 3;
    params.thresholdDist   = 2;
    params.collisionRadius = 1.5;

    % Create folder to save results
    resultsFolder = 'results';
    if ~exist(resultsFolder, 'dir')
        mkdir(resultsFolder);
    end

    gridMap = readmatrix("/Users/AkmatovArtur/Documents/GitHub/NavigatingDrones/maze/gridMap.txt"); % Load the grid map

    % Add paths through the grid
    numPaths = 5;  % Number of trails
    gridMap = addPaths(gridMap, numPaths);  % Add open trails to mimic a forest : O(numPaths x h)


    %% Initialize random positions for Drones and Target
    starts = zeros(params.numDrones, 2);
    for d = 1:params.numDrones
        starts(d, :) = getRandomFreeCell(gridMap); % O(h x w)
    end
    target = getRandomFreeCell(gridMap); % O(h x w)

    % Initialize RRT trees with the starting position
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

    % Display obstacles, drones, and the target using helper functions
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
            %if a drone found the target share with other drones and stop the loop

            if foundTarget % if drone finds the target, it shares target info with others
                shareTarget(rrtTrees, target);
                break; % The search stops.
            end

            %rrtExpansionWithCoordination() expands the current drone's RRT tree toward a random Point
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
    

    % From each drone's final position path is backtracked
    [allPaths, finalPositions] = backtrackAllPaths(rrtTrees); % O(d x n)
    disp(finalPositions);

    droneThatFoundPosition = finalPositions(droneThatFound, :);
    finalPositions(droneThatFound, :) = [];  % Removes the position of Drone that found the target
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



%--------------HELPER FUNCTIONS-------------------:

% SHARETARGET: 
% TC: O(d x n), d is the number of drones and n is average number of nodes per tree.
    % outer loop iterates over all drones --> O(d)
    % ismember checks if the target is in the tree, which has n nodes --> O(n)
% SC: O(1), because it modifies the existing RRT trees

% Adds the target node to all RRT trees for all drones.
function shareTarget(rrtTrees, target)
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
 
% ADDPATHS:
% TC: O(numPaths x h), h is number of rows in the grid. 
    % Because outer loop iterates numPaths times and inner loop iterates over all h rows
% SC: O(1), because it just modifies gridMap

% Modifies a grid map to simulate the presence of trails or paths through a forest by clearing obstacles along randomized paths
function gridMap = addPaths(gridMap, numPaths)
    [h, w] = size(gridMap);
    % loop until given numPaths
    for p = 1:numPaths
        startCol = randi([1, w]);  % Random starting column for each path
        for row = 1:h
            col = max(1, min(w, startCol + round(randn() * 2)));  % Path deviation from the starting column by adding a small random value randn()
            gridMap(row, col) = 0;  % Clear obstacles along the path
        end
    end
end


% BACKTRACKALLPATHS:
% TC: O(d x n), d: number of drones and n: average number of nodes per tree
    % function iterates over all d drones: O(d)
    % For each drone, backtracking iterates through the nodes in its tree. average number of nodes per tree is n.
    % backtracking for one drone is O(n), for d Drones it's O(d x n);
% SC: O(d x n), each drone's path is stored in a cell array and average size of path is O(n).
    % for all drones it's O(d x n) 

% For each drone, backtrack from last node in its tree to root.
% Outputs:
%   allPaths       - cell array, each cell is a Nx2 matrix of path coordinates
%   finalPositions - Nx2 array of the final coordinate of each drone
function [allPaths, finalPositions] = backtrackAllPaths(rrtTrees)
    
    numDrones = length(rrtTrees);
    allPaths = cell(numDrones, 1); % Pre-allocated as a cell array to store paths for each drone.
    finalPositions = zeros(numDrones, 2); % matrix to store the final coordinates of each drone.
    
    % loops through each drone's RRT tree and if the tree is empty the path is set to an empty array, and moves to the next drone.
    for d = 1:numDrones
        tree = rrtTrees{d};
        if isempty(tree)
            allPaths{d} = [];
            continue;
        end


        idx = size(tree, 1); %the index of the last node in the tree
        path = []; %array to store the backtracked pat
 
        while idx > 0 % loops untill idx is the root node
            node = tree(idx, 1:2); % appends the current node to the path
            path = [node; path];
            idx = tree(idx, 3); % updates idx to the parent index
        end

        allPaths{d} = path; % save the backtracked path
        finalPositions(d,:) = tree(end, 1:2); % store the final position
    end
end


% Plots the target location.
function plotTarget(target) 
    plot(target(1), target(2), 'r*', ...
         'MarkerSize', 12, ...
         'LineWidth', 2, ...
         'DisplayName', 'Target');
end

%Plots the obstacles (cells that are 1 in gridMap).
function plotObstacles(gridMap)
    [obsRows, obsCols] = find(gridMap == 1);
    plot(obsCols, obsRows, 'ks', ...
         'MarkerFaceColor', 'k', ...
         'MarkerSize', 5, ...
         'DisplayName', 'Obstacles');
end

% Plots the starting positions for all drones.
function plotInitialPositions(starts, colors)
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
    
