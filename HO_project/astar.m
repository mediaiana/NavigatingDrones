% define the binary grid
grid = readmatrix("/Users/AkmatovArtur/Documents/GitHub/NavigatingDrones/maze/gridMap.txt");
disp(grid);


% Define start points for the drones
[startPoints, goal] = runMultipleDronesRRT(); % each row is a start point [row, column]
disp('Starting algorithm...');

startPoints = startPoints(:, [2, 1]); % reorder columns: column 2 first, column 1 second
goal = goal([2, 1]); % swap row and column for the goal


% call the A* algorithm using the MEX function
paths = cppastar(startPoints, goal); % `paths` will be a cell array
disp('All drones paths found!');

% Adjust paths for MATLAB's 1-based indexing

adjusted_startPoints = startPoints + 1; % add 1 to start points
adjusted_goal = goal + 1;               % add 1 to goal point


% adjust each path in the cell array
adjusted_paths = cell(size(paths));
for i = 1:numel(paths)
    if ~isempty(paths{i})
        adjusted_paths{i} = paths{i}; % add 1 to each path
    else
        adjusted_paths{i} = []; % keep empty if no path
    end
end

% display the grid
figure;
plotObstacles(grid);      % visualize the grid
colormap([1 1 1; 0 0 0]); % custom colormap: 1->white (free space), 0->black (obstacle)
axis equal;           % equal aspect ratio
hold on;

% plot the start and goal points
for i = 1:size(adjusted_startPoints, 1)
    plot(adjusted_startPoints(i, 2), adjusted_startPoints(i, 1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % start points
end
plot(adjusted_goal(2), adjusted_goal(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % goal point

% simulation: Explore the grid and animate the process for all paths simultaneously
max_len = max(cellfun(@length, adjusted_paths)); % maximum length for the animation loop

% loop for simultaneous path animation
colors = ['r', 'g', 'b']; % Colors for paths
for i = 1:max_len
    for j = 1:numel(adjusted_paths)
        if i <= size(adjusted_paths{j}, 1)
            plot(adjusted_paths{j}(i, 2), adjusted_paths{j}(i, 1), ...
                 [colors(j), 'o'], 'MarkerSize', 5, 'MarkerFaceColor', colors(j)); % Plot path points
        end
    end
    drawnow; % Update the figure window
    pause(0.1); % Pause to create animation effect
end

% Plot the full paths after exploration
for j = 1:numel(adjusted_paths)
    if ~isempty(adjusted_paths{j})
        plot(adjusted_paths{j}(:, 2), adjusted_paths{j}(:, 1), ...
             [colors(j), '-o'], 'LineWidth', 2); % Plot full paths
    end
end

% Add labels and title
xlabel('X-axis (Columns)');
ylabel('Y-axis (Rows)');
title('Simultaneous A* Pathfinding Simulation for Multiple Drones');
