function [nearestIdx, minDist] = findNearestNode(rrtTree, randPt)
    % FINDNEARESTNODE Return index of nearest node in rrtTree to randPt.
    %
    % Inputs:
    %   rrtTree : Nx3 array => [x, y, parentIdx]
    %   randPt  : [xr, yr]
    %
    % Outputs:
    %   nearestIdx : index of the closest node in rrtTree
    %   minDist    : the distance to that node
    
        dx = rrtTree(:,1) - randPt(1);
        dy = rrtTree(:,2) - randPt(2);
        dists = sqrt(dx.^2 + dy.^2);
        [minDist, nearestIdx] = min(dists);
    end

    