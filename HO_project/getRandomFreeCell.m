% Finds random (x, y) free cell (0) in the grid and returns it.

%Time Complexity: O(h x w), h: height and w: width. Because it scans all cells in the gridMap
%Space Complexity: O(h x w) in the worst case. Because freeIdx stores the indecies of free cells

function pos = getRandomFreeCell(gridMap)
    [h, w] = size(gridMap);
    freeIdx = find(gridMap == 0);  % Find all free cells (0s)

    idx = freeIdx(randi(numel(freeIdx)));  % Randomly pick a free cell

    % Convert linear index to (row, col)
    row = mod(idx-1, h) + 1;
    col = floor((idx-1)/h) + 1;

    pos = [col, row];  % Return position as (x, y)
end

    