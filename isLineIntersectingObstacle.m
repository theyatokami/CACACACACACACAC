function isIntersecting = isLineIntersectingObstacle(lineStart, lineEnd, obstacles)
    % This function checks if a line segment between lineStart and lineEnd
    % intersects with any obstacles.
    % - lineStart and lineEnd are 2D points [x; y]
    % - obstacles is a struct array with fields 'type', 'center', 'radius' for circles,
    %   and 'type', 'corner', 'width', 'height' for rectangles.

    isIntersecting = false; % Initial assumption: no intersection

    for i = 1:length(obstacles)
        obstacle = obstacles(i);

        if strcmp(obstacle.type, 'circle')
            % Check intersection with circle
            if isLineCircleIntersecting(lineStart, lineEnd, obstacle.center, obstacle.radius)
                isIntersecting = true;
                return;
            end
        elseif strcmp(obstacle.type, 'rectangle')
            % Check intersection with rectangle
            if isLineRectangleIntersecting(lineStart, lineEnd, obstacle)
                isIntersecting = true;
                return;
            end
        end
    end
end

function isIntersecting = isLineCircleIntersecting(A, B, center, radius)
    % Check if line AB intersects with a circle
    % A, B: 2D points of the line segment
    % center: 2D point of circle center
    % radius: radius of the circle

    % Vector from A to B
    d = B - A;

    % Vector from A to circle center
    f = A - center;

    a = dot(d, d);
    b = 2*dot(f, d);
    c = dot(f, f) - radius^2;

    discriminant = b^2 - 4*a*c;
    if discriminant < 0
        isIntersecting = false;
    else
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant) / (2*a);
        t2 = (-b + discriminant) / (2*a);

        if (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)
            isIntersecting = true;
        else
            isIntersecting = false;
        end
    end
end

function isIntersecting = isLineRectangleIntersecting(A, B, rect)
    % Check if line AB intersects with a rectangle
    % A, B: 2D points of the line segment
    % rect: struct with 'corner', 'width', 'height'

    % Rectangle vertices
    R1 = rect.corner;
    R3 = R1 + [rect.width; 0];
    R2 = R1 + [rect.width; rect.height];
    R4 = R1 + [0; rect.height];

    % Check each side of the rectangle
    if isLineLineIntersecting(A, B, R1, R2) || ...
       isLineLineIntersecting(A, B, R2, R3) || ...
       isLineLineIntersecting(A, B, R3, R4) || ...
       isLineLineIntersecting(A, B, R4, R1)
        isIntersecting = true;
    else
        isIntersecting = false;
    end
end

function isIntersecting = isLineLineIntersecting(A, B, C, D)
    % Check if line segments AB and CD intersect
    % A, B, C, D: 2D points

    % Line AB represented as a1x + b1y = c1
    a1 = B(2) - A(2);
    b1 = A(1) - B(1);
    c1 = a1*A(1) + b1*A(2);

    % Line CD represented as a2x + b2y = c2
    a2 = D(2) - C(2);
    b2 = C(1) - D(1);
    c2 = a2*C(1) + b2*C(2);

    determinant = a1*b2 - a2*b1;

    if determinant == 0
        % Lines are parallel
        isIntersecting = false;
    else
        % Intersection point
        x = (b2*c1 - b1*c2) / determinant;
        y = (a1*c2 - a2*c1) / determinant;

        % Check if the intersection point is on both line segments
        if (min(A(1), B(1)) <= x && x <= max(A(1), B(1)) && ...
            min(A(2), B(2)) <= y && y <= max(A(2), B(2))) && ...
           (min(C(1), D(1)) <= x && x <= max(C(1), D(1)) && ...
            min(C(2), D(2)) <= y && y <= max(C(2), D(2)))
            isIntersecting = true;
        else
            isIntersecting = false;
        end
    end
end

