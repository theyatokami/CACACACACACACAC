function [path] = planPathPRM(L1, L2, nbPoints, start, goal, obstacles)
%%%%%%%%%%%%%%%%%%
% function planPathPRM(L1, L2, nbPoints, start, goal, obstacles)
%
% This function plans a path for a 2D robotic manipulator with two links
% using the PRM method and Dijkstra's algorithm.
%
% Inputs:
%   - L1: Length of the first link (in mm)
%   - L2: Length of the second link (in mm)
%   - nbPoints: The number of valid points in the roadmap
%   - start: Start point coordinates [x_start, y_start]
%   - goal: Goal point coordinates [x_goal, y_goal]
%   - obstacles: An array of obstacle structures (same format as in buildPRM)
%
% Outputs:
%   - path: An array of waypoints representing the planned path [(x1, y1); (x2, y2); ...]
%
% author: Your Name
% date: Current Date
%%%%%%%%%%%%%%%%%%

% Call the buildPRM function to get the valid points and obstacles
[q1q2_valid, obstacles] = buildPRM(L1, L2, nbPoints);

% Add start and goal points to the roadmap
startTheta = inverseKinematics(start, L1, L2);
goalTheta = inverseKinematics(goal, L1, L2);
q1q2_valid = [q1q2_valid, startTheta, goalTheta];

% Create a visibility graph
connectionMap = createVisibilityGraph(q1q2_valid, obstacles, L1, L2);

% Use Dijkstra's algorithm to find the shortest path
startIdx = size(q1q2_valid, 2) - 1; % Index of the start point
goalIdx = size(q1q2_valid, 2); % Index of the goal point
[~, pathIndices] = dijkstra(connectionMap, startIdx, goalIdx);

% Extract the path waypoints from the indices and convert back to Cartesian
path = q1q2_valid(1:2, pathIndices)';
path = forwardKinematics(path, L1, L2);

% Plot the PRM and path (optional)
plotPRM(q1q2_valid, connectionMap, start, goal, obstacles, path);

end

