function [q1q2_valid, obstacles] = buildPRM(L1, L2, nbPoints)
%%%%%%%%%%%%%%%%%%
%function buildPRM(L1, L2, nbPoints)
% ex. buildPRM(2000, 1000, 10)
%
% Inputs:
%	-L1: lenght of the first link (in mm)
%	-L2: length of the second link (in mm)
%	-nbPoints: the number of valid points in the roadmap
%
% Outputs: None
%
% author: Guillaume Gibert, guillaume.gibert@ecam.fr
% date: 22/11/2023
%%%%%%%%%%%%%%%%%%

% Initialize variables
    q1q2_valid = [];
    q1q2_2d = [];
    counter = 0;
    inner = L1 - L2;

% Define circular obstacles with placeholder values for rectangle fields
circleObstacle1 = struct('type', 'circle', 'center', [1500; 500], 'radius', 400, 'corner', [0; 0], 'width', 0, 'height', 0);
circleObstacle2 = struct('type', 'circle', 'center', [2000; -800], 'radius', 500, 'corner', [0; 0], 'width', 0, 'height', 0);
circleObstacle3 = struct('type', 'circle', 'center', [0; 0], 'radius', inner , 'corner', [0; 0], 'width', 0, 'height', 0);

% Define rectangular obstacles with placeholder values for circle fields
rectObstacle1 = struct('type', 'rectangle', 'corner', [-2000; -2000], 'width', 600, 'height', 1000, 'center', [0; 0], 'radius', 0);
rectObstacle2 = struct('type', 'rectangle', 'corner', [-1000; 1000], 'width', 2000, 'height', 2000, 'center', [0; 0], 'radius', 0);

% Combine all obstacles into an array
obstacles = [circleObstacle1, circleObstacle2, circleObstacle3, rectObstacle1, rectObstacle2];

% creates a figure
figure;

while (size(q1q2_valid,2) < nbPoints)
	% samples randomly the joint space
	q1 = rand()*360.0;
	q2 = rand()*360.0;

	% creates the DH table
	theta = [q1; q2];
	d = [0; 0];
	a = [L1; L2];
	alpha = [0; 0];

	% computes the FK
	wTee = dh2ForwardKinematics(theta, d, a, alpha, 1);

	% determines the position of the end-effector
	position_ee = wTee(1:2,end);

	% checks if the end-effector is not hitting any obstacle
	eeHittingObstacle = 0;


  inAnyObstacle = false;
    for obs = obstacles
        if strcmp(obs.type, 'circle') && isPointInCircle(position_ee, obs.center, obs.radius)
            inAnyObstacle = true;
            break;
        elseif strcmp(obs.type, 'rectangle') && isPointInRectangle(position_ee, obs)
            inAnyObstacle = true;
            break;
        end
    end

	% stores these random values


	  if (eeHittingObstacle == 0) && (inAnyObstacle == false)
            % Store and plot valid points
            q1q2_valid = [q1q2_valid theta];
            subplot(1, 2, 1);
            plot(q1, q2, '+g'); hold on;
            subplot(1, 2, 2);
            plot(position_ee(1), position_ee(2), '+g'); hold on;
        else
            % Plot invalid points
            subplot(1, 2, 1);
            plot(q1, q2, '*r'); hold on;
            subplot(1, 2, 2);
            plot(position_ee(1), position_ee(2), '*r'); hold on;
        end

        counter = counter + 1;
    end

% displays stats
fprintf("%d points were sorted to achieve %d valid points\n", counter, nbPoints)

% add limits, lables and obstacles to the map
subplot(1,2,1);
	xlim([0 360]);
	ylim([0 360]);

	xlabel('q1(deg)');
	ylabel('q2(deg)');
	title('Joint space');

subplot(1,2,2);



	drawCircle(0,0, L1+L2);
	drawCircle(0,0, L1-L2);

	xlim([-(L1+L2) (L1+L2)]);
	ylim([-(L1+L2) (L1+L2)]);

	xlabel('x(mm)');
	ylabel('y(mm)');
	title('Cartesian space');

% creates a connection map
connectionMap = zeros(nbPoints, nbPoints);
% After generating q1q2_valid but before the plotting section
for i = 1:size(q1q2_valid, 2)
    for j = 1:size(q1q2_valid, 2)
        if i ~= j

            pos1 = dh2ForwardKinematics([q1q2_valid(1, i); q1q2_valid(2, i)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
            pos2 = dh2ForwardKinematics([q1q2_valid(1, j); q1q2_valid(2, j)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
            if ~isLineIntersectingObstacle(pos1, pos2, obstacles)
                connectionMap(i, j) = 1;
            end
        end
    end
end

 % Convert q1q2_valid into 2D Cartesian coordinates
    for i = 1:size(q1q2_valid, 2)
        pos = dh2ForwardKinematics([q1q2_valid(1, i); q1q2_valid(2, i)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
        q1q2_2d = [q1q2_2d, pos]; % Append the 2D position
    end

% Plotting connections between valid points
for i = 1:size(q1q2_valid, 2)
    for j = 1:size(q1q2_valid, 2)
        if connectionMap(i, j) == 1

            pos1 = dh2ForwardKinematics([q1q2_valid(1, i); q1q2_valid(2, i)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
            pos2 = dh2ForwardKinematics([q1q2_valid(1, j); q1q2_valid(2, j)], [0; 0], [L1; L2], [0; 0], 1)(1:2, end);
            plot([pos1(1), pos2(1)], [pos1(2), pos2(2)], 'g'); % Plot connections
            hold on;
        end
    end
end




% Plotting in Joint Space
subplot(1,2,1);
plot(q1q2_valid(1,:), q1q2_valid(2,:), 'go'); % Plot valid points in green
xlabel('q1 (deg)');
ylabel('q2 (deg)');
title('Joint Space');

% Plotting in Cartesian Space
subplot(1,2,2);

% Plotting obstacles
for i = 1:length(obstacles)
    obstacle = obstacles(i);
    if strcmp(obstacle.type, 'circle')
        % Draw circle obstacle
        drawCircle(obstacle.center(1), obstacle.center(2), obstacle.radius);
    elseif strcmp(obstacle.type, 'rectangle')
        % Draw rectangular obstacle
        % Define the rectangle position using [x, y, width, height]
        rectPosition = [obstacle.corner(1), obstacle.corner(2), obstacle.width, obstacle.height];
        rectangle('Position', rectPosition, 'EdgeColor', 'r', 'LineWidth', 2);
    end
end



xlabel('X (mm)');
ylabel('Y (mm)');
title('Cartesian Space');

% Ensure all plots remain visible
hold off;
return;
end

