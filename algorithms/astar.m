% Define the binary grid
% 1 represents passable cells, 0 represents obstacles
grid = readmatrix("maze\modified_maze.txt");

% Define the start and goal points as [row, column]
start = [2, 2];  % Start at the top-left corner
goal = [52, 18]; % Goal at the bottom-right corner


% Call the A* algorithm using the MEX function
% Replace 'astar_mex' with the name of your compiled MEX file
path = project(grid, start, goal);

% Adjust start, goal, and path for MATLAB's 1-based indexing
adjusted_start = start + 1; % Add 1 to row and column indices
adjusted_goal = goal + 1;   % Add 1 to row and column indices
adjusted_path = path + 1;   % Add 1 to each row of the path matrix

% Display the grid
figure;
imagesc(grid);        % Visualize the grid
colormap([1 1 1; 0 0 0]); % Custom colormap: 1->black (obstacle), 0->white (free space)
axis equal;           % Equal aspect ratio
hold on;

% Plot the start and goal points
plot(adjusted_start(2), adjusted_start(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start point
plot(adjusted_goal(2), adjusted_goal(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');   % Goal point

% Simulation: Explore the grid and animate the process
explored_points = [];
for i = 1:length(adjusted_path)
    % Plot the explored path
    plot(adjusted_path(i, 2), adjusted_path(i, 1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    drawnow; % Update the figure window
    pause(0.1); % Pause to create animation effect
    
    % Store explored points
    explored_points = [explored_points; adjusted_path(i, :)];
end

% Plot the full path after exploration
plot(adjusted_path(:, 2), adjusted_path(:, 1), 'r-o', 'LineWidth', 2); % Plot the full path in red

% Add labels and title
xlabel('X-axis (Columns)');
ylabel('Y-axis (Rows)');
title('A* Pathfinding Simulation');
