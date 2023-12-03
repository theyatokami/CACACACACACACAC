function path = extractPath(distanceToNode, parentOfNode, nodeTrajectory, start, goal)
    % Extract the path from Dijkstra's algorithm output
    currentNode = goal;
    path = [];

    while currentNode ~= start
        path = [path, currentNode];
        currentNode = round(parentOfNode(currentNode)); % Round to the nearest integer
    end

    % Add the start node to the path
    path = [path, start];

    % Reverse the path to get it from start to goal
    path = fliplr(path);
end

