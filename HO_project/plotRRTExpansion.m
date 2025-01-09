function plotRRTExpansion(rrtTrees, colors)
    % PLOTRRTEXPANSION Plots the trees for each drone at a given moment.
    %
    %   plotRRTExpansion(rrtTrees, colors)
    %
    % Inputs:
    %   rrtTrees - cell array of Nx3 RRTs
    %   colors   - Nx3 array of colors
    
        hold on;  % Ensure we overlay on existing figure
        numDrones = length(rrtTrees);
    
        for d = 1:numDrones
            tree = rrtTrees{d};
            if size(tree,1) < 2
                continue; % nothing to plot
            end
            for n = 2:size(tree, 1)
                px   = tree(n, 1);
                py   = tree(n, 2);
                parIdx = tree(n, 3);
                if parIdx > 0
                    ppx = tree(parIdx, 1);
                    ppy = tree(parIdx, 2);
                    plot([px, ppx], [py, ppy], '-', ...
                         'Color', colors(d,:), 'LineWidth', 1.5);
                end
            end
        end
    end
    