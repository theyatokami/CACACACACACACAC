function index = findNearestPoint(point, points2D)
    % Find the index of the nearest point in points2D to the given point
    distances = sqrt(sum((points2D - point).^2, 2));
    [~, index] = min(distances);
end

