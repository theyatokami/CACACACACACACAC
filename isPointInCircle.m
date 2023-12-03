function isInCircle = isPointInCircle(point, center, radius)
    isInCircle = norm(point - center) <= radius;
end
