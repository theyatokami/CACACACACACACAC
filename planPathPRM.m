function path = planPathPRM(L1, L2, nbPoints)
    % Run buildPRM to get the PRM
    [q1q2_valid, obstacles, connectionMap] = buildPRM(L1, L2, nbPoints);

    % Convert q1q2_valid to Cartesian coordinates
    q1q2_2d = []; % Initialize the array for 2D Cartesian coordinates
    for i = 1:size(q1q2_valid, 2)
        pos = dh2ForwardKinematics([q1q2_valid(1, i); q1q2_valid(2, i)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
        q1q2_2d = [q1q2_2d, pos];
    end

    % Add start and goal points to the set of points
    start_point = [2000, 0];
    goal_point = [-2000, 0];
    points2D = [start_point; q1q2_2d'; goal_point];

    % Find nearest neighbors for start and goal points
    [start_nearest_idx, start_nearest_dist] = findNearestNeighbor(start_point, q1q2_2d');
    [goal_nearest_idx, goal_nearest_dist] = findNearestNeighbor(goal_point, q1q2_2d');

    % Update connectionMap for start and goal points
    connectionMap = [connectionMap; zeros(1, size(connectionMap, 2))];
    connectionMap = [connectionMap, zeros(size(connectionMap, 1), 1)];
    connectionMap(1, start_nearest_idx+1) = 1; % +1 to account for start point being the first row
    connectionMap(start_nearest_idx+1, 1) = 1;
    connectionMap(end, goal_nearest_idx+1) = 1; % end row is the goal point
    connectionMap(goal_nearest_idx+1, end) = 1;

    % Create the visibility graph
    [nbNodes, visibilityGraph] = createVisibilityGraph(connectionMap, points2D);

    % Plan the path using Dijkstra's algorithm
    [distanceToNode, parentOfNode, nodeTrajectory] = dijkstra(nbNodes, visibilityGraph);
    if isempty(nodeTrajectory)
        error('No valid path found');
    end
    % Extract the path in Cartesian coordinates
    path = points2D(nodeTrajectory, :);

    % Plot the path in Cartesian space
    % ... [plotting code here] ...

    return;
end

function [nearest_idx, nearest_dist] = findNearestNeighbor(point, points)
    distances = sqrt(sum((points - point).^2, 2));
    [nearest_dist, nearest_idx] = min(distances);
end

