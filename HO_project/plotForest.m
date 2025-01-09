function plotForest(gridMap)
    % PLOTFORREST Plots the grid with obstacles represented as trees.
    %   gridMap - Input grid (0 = free, 1 = obstacle)

    [rows, cols] = find(gridMap == 1);  % Find obstacle locations
    scatter(cols, rows, 60, 'g', 'filled', 'DisplayName', 'Trees');  % Green dots for trees
    hold on;
end
