function connectionMatrix = generateConnectionMatrix(q1q2_valid, obstacles, L1, L2)
    nbPoints = size(q1q2_valid, 2);
    connectionMatrix = zeros(nbPoints, nbPoints);

    for i = 1:nbPoints
        for j = 1:nbPoints
            if i ~= j
                pos1 = jointToCartesian(q1q2_valid(:,i), L1, L2);
                pos2 = jointToCartesian(q1q2_valid(:,j), L1, L2);

                if ~isLineIntersectingObstacle(pos1, pos2, obstacles)
                    connectionMatrix(i, j) = 1;
                end
            end
        end
    end
end

function intersecting = isLineIntersectingObstacle(point1, point2, obstacles)
    % Simple check for line-obstacle intersection
    intersecting = false;
    for i = 1:length(obstacles)
        obstacle = obstacles(i);
        if strcmp(obstacle.type, 'circle')
            % Check if line intersects circle (obstacle)
            % Implement your own logic or use an existing algorithm
            % This is a placeholder for collision checking logic
            % intersecting = [your collision checking logic]
        end
    end
end

