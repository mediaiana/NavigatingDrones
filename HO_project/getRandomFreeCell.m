function pos = getRandomFreeCell(gridMap)
    % GETRANDOMFREECELL Return a random (x, y) for a free cell (0) in the grid.
    [h, w] = size(gridMap);
    freeIdx = find(gridMap == 0);  % Find all free cells (0s)
    idx = freeIdx(randi(numel(freeIdx)));  % Randomly pick a free cell
    % Convert linear index to (row, col)
    row = mod(idx-1, h) + 1;
    col = floor((idx-1)/h) + 1;
    pos = [col, row];  % Return position as (x, y)
end

    