function gridMap = createDenseClusteredGrid(h, w, obstacleDensity, numClusters)
    gridMap = zeros(h, w);
    totalObstacles = round(h * w * obstacleDensity);

    clusterCenters = randi([1, min(h, w)], numClusters, 2);

    for i = 1:numClusters
        center = clusterCenters(i, :);

        % Define cluster area
        clusterAreaX = max(1, center(1)-5):min(h, center(1)+5);
        clusterAreaY = max(1, center(2)-5):min(w, center(2)+5);

        % Dynamically calculate obstacles per cluster
        clusterMaxObstacles = length(clusterAreaX) * length(clusterAreaY);
        obstaclesPerCluster = min(round(totalObstacles / numClusters), clusterMaxObstacles);

        placedObstacles = 0;
        attempts = 0;

        while placedObstacles < obstaclesPerCluster && attempts < 10 * obstaclesPerCluster
            % Generate a random position within the cluster area
            x = clusterAreaX(randi(length(clusterAreaX)));
            y = clusterAreaY(randi(length(clusterAreaY)));

            % Place the obstacle if the cell is free
            if gridMap(x, y) == 0
                gridMap(x, y) = 1;
                placedObstacles = placedObstacles + 1;
            end

            attempts = attempts + 1;

            % Break early if the cluster area is already full
            if placedObstacles >= clusterMaxObstacles
                break;
            end
        end

        % Warn if not all obstacles were placed
        if placedObstacles < obstaclesPerCluster
            fprintf('Warning: Cluster %d placed %d/%d obstacles\n', ...
                    i, placedObstacles, obstaclesPerCluster);
        end
    end
end
