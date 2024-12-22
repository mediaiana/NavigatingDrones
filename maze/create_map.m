% Generate a random maze with specific parameters
map = mapMaze(1, 1, 'MapSize', [100 100], 'MapResolution', 1);

% Extract the maze grid from the binaryOccupancyMap
mazeGrid = getOccupancy(map);  % This gives a logical matrix (0's and 1's)

% Write the maze grid to a text file
fileID = fopen('maze_grid.txt', 'w');  % Open the file for writing
if fileID == -1
    error('Failed to open the file for writing.');
end

% Write the maze grid to the file, one row at a time
for i = 1:size(mazeGrid, 1)
    fprintf(fileID, '%d ', mazeGrid(i, :));  % Write each row of the grid
    fprintf(fileID, '\n');  % New line after each row
end

% Close the file
fclose(fileID);

% Visualize the maze
figure;
imshow(mazeGrid);
title('Generated Maze');

disp('Maze Grid:');
disp(mazeGrid);  % This prints the matrix directly
