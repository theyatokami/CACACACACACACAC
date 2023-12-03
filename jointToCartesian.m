function point = jointToCartesian(theta, L1, L2)
    theta1 = deg2rad(theta(1));
    theta2 = deg2rad(theta(2));

    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    point = [x; y];
end

