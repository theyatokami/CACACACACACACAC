function isInRectangle = isPointInRectangle(point, rect)
    xMin = rect.corner(1);
    xMax = rect.corner(1) + rect.width;
    yMin = rect.corner(2);
    yMax = rect.corner(2) + rect.height;

    % Debugging: Print the boundaries and the point
    fprintf('xMin: %.2f, xMax: %.2f, yMin: %.2f, yMax: %.2f\n', xMin, xMax, yMin, yMax);
    fprintf('Point: [%.2f, %.2f]\n', point(1), point(2));

    isInRectangle = point(1) >= xMin && point(1) <= xMax && point(2) >= yMin && point(2) <= yMax;

    % Debugging: Print whether the point is in the rectangle
    if isInRectangle
        fprintf('Point is inside the rectangle.\n');
    else
        fprintf('Point is outside the rectangle.\n');
    end
end

