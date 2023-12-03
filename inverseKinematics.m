function [theta1, theta2] = inverseKinematics(point, L1, L2)
    x = point(1);
    y = point(2);

    cosTheta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    sinTheta2 = sqrt(1 - cosTheta2^2);
    theta2 = atan2(sinTheta2, cosTheta2);

    k1 = L1 + L2 * cosTheta2;
    k2 = L2 * sinTheta2;
    theta1 = atan2(y, x) - atan2(k2, k1);

    % Convert to degrees
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);
end

