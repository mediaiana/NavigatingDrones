function gridMap = createVariableDensityGrid(h, w)
    % CREATEVARIABLEDENSITYGRID Generates a grid with regions of varying obstacle densities.
    %   h, w - Grid dimensions (height and width)

    gridMap = zeros(h, w);
    regions = 4;  % Divide the grid into 4 horizontal regions
    densities = linspace(0.1, 0.4, regions);  % Obstacle densities for each region

    regionHeight = round(h / regions);

    for r = 1:regions
        startRow = (r - 1) * regionHeight + 1;
        endRow = min(r * regionHeight, h);
        for row = startRow:endRow
            for col = 1:w
                if rand() < densities(r)
                    gridMap(row, col) = 1;  % Place an obstacle
                end
            end
        end
    end
end
