TIME_STEP = 64;
SPEED = 1.0;

% Get motors
wheel_back_left = wb_robot_get_device('wheel_back_left');
wheel_front_left = wb_robot_get_device('wheel_front_left');
wheel_back_right = wb_robot_get_device('wheel_back_right');
wheel_front_right = wb_robot_get_device('wheel_front_right');

% Get distance sensors
distance_sensor_middle = wb_robot_get_device('distance_sensor_middle');
distance_sensor_left = wb_robot_get_device('distance_sensor_left');
distance_sensor_right = wb_robot_get_device('distance_sensor_right');

% Get Vehicle GPS
vehicle_gps = wb_robot_get_device('vehicle_gps');

%Get Vehicle Inertial Unit
vehicle_inertial_unit = wb_robot_get_device('vehicle_inertial_unit');

% Create arrays for wheels and sensors
wheels = [
    wheel_back_left
    wheel_front_left
    wheel_back_right
    wheel_front_right
];

distance_sensors = [
    distance_sensor_left
    distance_sensor_middle
    distance_sensor_right
];

%Enable the wheels and sensors
intialiseWheels(wheels);
initialiseDistanceSensors(distance_sensors, TIME_STEP);
wb_gps_enable(vehicle_gps, TIME_STEP);
wb_inertial_unit_enable(vehicle_inertial_unit, TIME_STEP);

%Threshold for obstacle detection
distance_threshold = 900;
turning = false;
turn_delay = 0.5;


%A* Algorithm Grid Variables (m)
cell_size = 0.5;
floor_x = 3;
floor_y = 3;
grid_width = floor_x / cell_size;
grid_height = floor_y / cell_size;
grid = zeros(grid_height, grid_width);

%Note Start TIme
start_time = wb_robot_get_time();


% Goal Position
goal_x = 0;
goal_y = 0;
goal_threshold = 0.1;
goal_reached = false;
grid_based_goal_x = floor((goal_x + floor_x/2) / cell_size) + 1;
grid_based_goal_y = floor((goal_y + floor_y/2) / cell_size) + 1;
grid(grid_based_goal_y, grid_based_goal_x) = 2;

while wb_robot_step(TIME_STEP) ~= -1
    if ~goal_reached
        % Get current position and distance sensor readings
        position = getPosition(vehicle_gps);
        rpy = getOrientation(vehicle_inertial_unit);
        ds_values = getDS(distance_sensors);

        % Check if goal is reached
        distance_to_goal = sqrt((position(1) - goal_x)^2 + (position(2) - goal_y)^2);
        if distance_to_goal < goal_threshold
            stop(wheels);
            fprintf('Goal Reached!\n');
            goal_reached = true;
            continue;
        end

        %Update grid map with current sensor readings
        grid = mapGridCoordinates(position, ds_values, rpy(3), cell_size, grid, floor_x, floor_y);

        % Run A* algorithm to get path to goal
        grid_based_position_x = floor((position(1) + floor_x/2) / cell_size) + 1;
        grid_based_position_y = floor((position(2) + floor_y/2) / cell_size) + 1;
        path = aStarSearch(grid, grid_based_position_x, grid_based_position_y, grid_based_goal_x, grid_based_goal_y);

        % Convert paths from grid coordinates to real world coordinates
        real_path = [];
        for i = 1:size(path, 1)
            [real_x, real_y] = gridToRealCoordinates(path(i, 1), path(i, 2), cell_size, floor_x, floor_y);
            real_path = [real_path; real_x, real_y];
        end

        fprintf('Path Length: %d\n', size(path, 1));
        fprintf('Current Position: (%.2f, %.2f)\n', position(1), position(2));
        fprintf('Path:\n');
        disp(real_path);
        % Find Next point in path to current position
        if size(real_path, 1) >= 2
            closest_path_x = real_path(2, 1);
            closest_path_y = real_path(2, 2);
        else
            closest_path_x = real_path(1, 1);
            closest_path_y = real_path(1, 2);
        end

        % Find angle to next point and turn towards it if not facing it, then move forward
        [alligned, angle_to_target, angle_difference] = turnToTarget(closest_path_x, closest_path_y, position, rpy(3));
        if alligned == "left"
            turnLeft(SPEED, wheels);
        elseif alligned == "right"
            turnRight(SPEED, wheels);
        else
            goForwards(SPEED, wheels);
        end

        % Obstacle Avoidance
        object_detected = any(ds_values < distance_threshold);

        if object_detected && ~turning
            turning = true;
            turn_start_time = wb_robot_get_time();

            if ds_values(1) > ds_values(3)
                turn_direction = 'left';
            else
                turn_direction = 'right';
            end
        end

        if turning
            current_turn_time = wb_robot_get_time() - turn_start_time;
            if current_turn_time < turn_delay
                if turn_direction == "left"
                    turnLeft(SPEED, wheels);
                else
                    turnRight(SPEED, wheels);
                end
            end

            if ~(object_detected) 
                turning = false;
            end
        end

        % fprintf(['Robot: (%.2f, %.2f) | Goal: (%.2f, %.2f)\n' ...
        %  'Target: (%.2f, %.2f) | Yaw: %.2f | Angle to Target: %.2f | Decision: %s | Path length: %d\n\n'], ...
        %  position(1), position(2), goal_x, goal_y, ...
        %  closest_path_x, closest_path_y, rpy(3), angle_to_target, alligned, size(path, 1));
    
    end

end

wb_robot_cleanup();



%Put Wheels in velocity control mode and set their speed to 0
function intialiseWheels(wheels)
    for i = 1:4
        wb_motor_set_position(wheels(i), inf);
        wb_motor_set_velocity(wheels(i), 0);
    end
end

%Initialize Distance Sensors
function initialiseDistanceSensors(distance_sensors, TIME_STEP)
    for i = 1:3
        wb_distance_sensor_enable(distance_sensors(i), TIME_STEP);
    end
end

%Get the current position of the vehiclefunction position = getPosition(gps)
function position=getPosition(gps)   
    position = wb_gps_get_values(gps);
end

%Get the current distance sensor readings
function values=getDS(distance_sensors)
    values = zeros(1, 3);
    for i = 1:3
        values(i) = wb_distance_sensor_get_value(distance_sensors(i));
    end
end

%Get the current orientation of the vehicle, within the range of 0 to 2*pi rad
function orientation=getOrientation(inertial_unit)
    orientation = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    for i = 1:3
        if orientation(i) < 0
            orientation(i) = 2 * pi - abs(orientation(i));
        end
    end
end

function path = aStarSearch(grid, start_x, start_y, goal_x, goal_y)

    % Create unexplored and explored arrays
    unexplored = [];
    explored = [];

    % Create parent, g, and f score maps
    parents = containers.Map;
    g_costs = containers.Map;
    f_costs = containers.Map;

    % Add start cell
    unexplored = [unexplored; start_x, start_y];
    g_costs(sprintf('%d,%d', start_x, start_y)) = 0;
    f_costs(sprintf('%d,%d', start_x, start_y)) = getHeuristic(start_x, start_y, goal_x, goal_y);
    parents(sprintf('%d,%d', start_x, start_y)) = 'None';

    % Iterate through the grid
    while size(unexplored, 1) > 0

        % Choose cell with lowest f_cost
        lowest_f_cost = inf;
        for i = 1:size(unexplored, 1)
            cell_x = unexplored(i, 1);
            cell_y = unexplored(i, 2);
            cell_f_cost = f_costs(sprintf('%d,%d', cell_x, cell_y));
            if cell_f_cost < lowest_f_cost
                lowest_f_cost = cell_f_cost;
                current_cell = [cell_x, cell_y];
                current_index = i;
            end
        end

        current_cell_x = current_cell(1);
        current_cell_y = current_cell(2);

        % Check if its the goal cell
        if current_cell_y == goal_y && current_cell_x == goal_x
            break;
        end
        
        % Move current cell from unexplored to explored
        unexplored(current_index, :) = [];
        explored = [explored; current_cell];

        % Get neighbors
        neighbors = getNeighbors(current_cell_x, current_cell_y, grid);
        
        % Check each neighbor
        for i = 1:size(neighbors, 1)
            neighbor_x = neighbors(i, 1);
            neighbor_y = neighbors(i, 2);

            % Check if neighbor is in explored
            if any(ismember(explored, [neighbor_x, neighbor_y], 'rows'))
                continue;
            end

            % Calculate g cost
            new_g_cost = g_costs(sprintf('%d,%d', current_cell_x, current_cell_y)) + 1;

            % Check if neighbor is in unexplored
            if ~any(ismember(unexplored, [neighbor_x, neighbor_y], 'rows'))
                unexplored = [unexplored; neighbor_x, neighbor_y];

            % if new g cost is higher than existing g cost, skip
            elseif new_g_cost >= g_costs(sprintf('%d,%d', neighbor_x, neighbor_y))
                continue;
            end

            % Update parent, g cost, and f cost
            % Parent: Current Cell is new best way to reach this neighbor
            parents(sprintf('%d,%d', neighbor_x, neighbor_y)) = sprintf('%d,%d', current_cell_x, current_cell_y);
            % G Cost: Cost from Start Cell to Neighbour through Current Cell
            g_costs(sprintf('%d,%d', neighbor_x, neighbor_y)) = new_g_cost;
            % F Cost: Estimated cost from Start Cell to Goal Cell through Neighbour
            f_costs(sprintf('%d,%d', neighbor_x, neighbor_y)) = new_g_cost + getHeuristic(neighbor_x, neighbor_y, goal_x, goal_y);
        end
    end

    % Reconstruct path from goal to start using parent map
    path = [];
    current_cell_x = goal_x;
    current_cell_y = goal_y;
    
    while true
        % Get parent of current cell
        parent = parents(sprintf('%d,%d', current_cell_x, current_cell_y));

        % Add parent to path
        path = [[current_cell_x, current_cell_y]; path];

        % If parent is None, break
        if strcmp(parent, 'None')
            break;
        end

        % Update current cell to parent cell
        string_parts = strsplit(parent, ',');
        current_cell_x = str2double(string_parts{1});
        current_cell_y = str2double(string_parts{2});
    end
end

function heuristic = getHeuristic(x1, y1, x2, y2)
    %Using Manhattan Distance as heuristic
    heuristic = abs(x1 - x2) + abs(y1 - y2);
end

function valid_neighbors = getNeighbors(cell_x, cell_y, grid)

    %Get Neighbouring Cells including diagonals
    neighbors = [
        cell_x - 1, cell_y;     % Left
        cell_x + 1, cell_y;     % Right
        cell_x, cell_y - 1;     % Up
        cell_x, cell_y + 1;     % Down
        cell_x - 1, cell_y - 1; % Up-Left
        cell_x + 1, cell_y - 1; % Up-Right
        cell_x - 1, cell_y + 1; % Down-Left
        cell_x + 1, cell_y + 1; % Down-Right
    ];

    % Loop through neighbors and check if they are within grid bounds and not obstacles
    valid_neighbors = [];
    for i = 1:size(neighbors, 1)
        neighbour_x = neighbors(i, 1);
        neighbour_y = neighbors(i, 2);

        if neighbour_x < 1 || neighbour_x > size(grid, 2)
            continue;
        end
        if neighbour_y < 1 || neighbour_y > size(grid, 1)
            continue;
        end
        if grid(neighbour_y, neighbour_x) == 1
            continue;
        end
        valid_neighbors = [valid_neighbors; neighbors(i, :)];
    end

end

%Plot positions on the grid using gps position and distance sensor readings, to be used for A* algorithm
function grid=mapGridCoordinates(gps_position, distance_sensor_values, yaw,  cell_size, grid, floor_x, floor_y)
    x = gps_position(1);
    y = gps_position(2);
    theta = yaw;

    %Positions of the distance sensors at based on sensor reading and offset from vehicle centre (m)
    middle_sensor_pos_x = 0;
    middle_sensor_pos_y = 0.2 + distance_sensor_values(2) / 3333.3333;
    left_sensor_pos_x = -0.125 + -1*(cos(1.309) * distance_sensor_values(1) / 3333.3333);
    left_sensor_pos_y = 0.2 + sin(1.309) * distance_sensor_values(1) / 3333.3333;
    right_sensor_pos_x = 0.125 + (cos(1.309) * distance_sensor_values(3) / 3333.3333);
    right_sensor_pos_y = 0.2 + sin(1.309) * distance_sensor_values(3) / 3333.3333;



    %Using 2D Rotation
    rotated_middle_sensor_pos_x = middle_sensor_pos_x * cos(theta) - middle_sensor_pos_y * sin(theta);
    rotated_middle_sensor_pos_y = middle_sensor_pos_x * sin(theta) + middle_sensor_pos_y * cos(theta);
    rotated_left_sensor_pos_x = left_sensor_pos_x * cos(theta) - left_sensor_pos_y * sin(theta);
    rotated_left_sensor_pos_y = left_sensor_pos_x * sin(theta) + left_sensor_pos_y * cos(theta);
    rotated_right_sensor_pos_x = right_sensor_pos_x * cos(theta) - right_sensor_pos_y * sin(theta);
    rotated_right_sensor_pos_y = right_sensor_pos_x * sin(theta) + right_sensor_pos_y * cos(theta);

    %Using Current GPS position
    actual_middle_sensor_pos_x = rotated_middle_sensor_pos_x + x;
    actual_middle_sensor_pos_y = rotated_middle_sensor_pos_y + y;
    actual_left_sensor_pos_x = rotated_left_sensor_pos_x + x;
    actual_left_sensor_pos_y = rotated_left_sensor_pos_y + y;
    actual_right_sensor_pos_x = rotated_right_sensor_pos_x + x;
    actual_right_sensor_pos_y = rotated_right_sensor_pos_y + y;

    %Account for floor being centred at (0,0)
    actual_middle_sensor_pos_x = actual_middle_sensor_pos_x + floor_x / 2;
    actual_middle_sensor_pos_y = actual_middle_sensor_pos_y + floor_y / 2;
    actual_left_sensor_pos_x = actual_left_sensor_pos_x + floor_x / 2;
    actual_left_sensor_pos_y = actual_left_sensor_pos_y + floor_y / 2;
    actual_right_sensor_pos_x = actual_right_sensor_pos_x + floor_x / 2;
    actual_right_sensor_pos_y = actual_right_sensor_pos_y + floor_y / 2;

    %Convert to grid
    middle_sensor_grid_x = floor(actual_middle_sensor_pos_x / cell_size) + 1;
    middle_sensor_grid_y = floor(actual_middle_sensor_pos_y / cell_size) + 1;
    left_sensor_grid_x = floor(actual_left_sensor_pos_x / cell_size) + 1;
    left_sensor_grid_y = floor(actual_left_sensor_pos_y / cell_size) + 1;
    right_sensor_grid_x = floor(actual_right_sensor_pos_x / cell_size) + 1;
    right_sensor_grid_y = floor(actual_right_sensor_pos_y / cell_size) + 1;

    %Using grid layout: 0=free, 1=obstacle, 2=goal)

    %Handle Left Sensor
    if left_sensor_grid_x >= 1 && left_sensor_grid_x <= size(grid, 2) && left_sensor_grid_y >= 1 && left_sensor_grid_y <= size(grid, 1)
        if grid(left_sensor_grid_y, left_sensor_grid_x) ~= 2
            if distance_sensor_values(1) < 1000
                grid(left_sensor_grid_y, left_sensor_grid_x) = 1;
            else
                grid(left_sensor_grid_y, left_sensor_grid_x) = 0;
            end
        end
    end

    %Handle Middle Sensor
    if middle_sensor_grid_x >= 1 && middle_sensor_grid_x <= size(grid, 2) && middle_sensor_grid_y >= 1 && middle_sensor_grid_y <= size(grid, 1)
        if grid(middle_sensor_grid_y, middle_sensor_grid_x) ~= 2
            if distance_sensor_values(2) < 1000
                grid(middle_sensor_grid_y, middle_sensor_grid_x) = 1;
            else
                grid(middle_sensor_grid_y, middle_sensor_grid_x) = 0;
            end
        end
    end

    %Handle Right Sensor
    if right_sensor_grid_x >= 1 && right_sensor_grid_x <= size(grid, 2) && right_sensor_grid_y >= 1 && right_sensor_grid_y <= size(grid, 1)
        if grid(right_sensor_grid_y, right_sensor_grid_x) ~= 2
            if distance_sensor_values(3) < 1000
                grid(right_sensor_grid_y, right_sensor_grid_x) = 1;
            else
                grid(right_sensor_grid_y, right_sensor_grid_x) = 0;
            end
        end
    end
end

% Reverse Real-Grid coordinate Transformation
% Reverses floor centering and grid conversion by converting grid coordinates back to real world coordinates in meters, relative to the center of the floor (0,0)
function [real_x, real_y] = gridToRealCoordinates(grid_x, grid_y, cell_size, floor_x, floor_y)
    real_x = (grid_x - 1) * cell_size - floor_x / 2 + cell_size / 2;
    real_y = (grid_y - 1) * cell_size - floor_y / 2 + cell_size / 2;
end

% Find next point in path to current position
function [closest_x, closest_y] = findNextPathPoint(position, path)
    % Find the closest point in the path to the current position then find next point in path after that to use as target
    smallest_distance = inf;
    vehicle_x = position(1);
    vehicle_y = position(2);
    closest_point_index = 1;
    for i = 1:size(path, 1)
        path_x = path(i, 1);
        path_y = path(i, 2);
        distance = sqrt((vehicle_x - path_x)^2 + (vehicle_y - path_y)^2);
        if distance < smallest_distance
            smallest_distance = distance;
            closest_point_index = i;
        end
    end

    % Find the next point in the path after the closest point
    if closest_point_index < size(path, 1)
        closest_x = path(closest_point_index + 1, 1);
        closest_y = path(closest_point_index + 1, 2);
    else
        closest_x = path(closest_point_index, 1);
        closest_y = path(closest_point_index, 2);
    end
end

% Turn to target point function
function [aligned, target_angle, angle_difference] = turnToTarget(target_x, target_y, position, yaw)

    % Calculate world angle from robot position to target
    target_angle = atan2(target_y - position(2), target_x - position(1));

    % Convert target angle to 0 to 2*pi
    if target_angle < 0
        target_angle = target_angle + 2*pi;
    end

    % Convert robot yaw to same reference frame as atan2
    current_angle = yaw + pi/2;

    if current_angle > 2*pi
        current_angle = current_angle - 2*pi;
    end

    % Smallest signed angle difference [-pi, pi]
    angle_difference = atan2(sin(target_angle - current_angle), ...
                             cos(target_angle - current_angle));

    tolerance = 0.1;

    if angle_difference > tolerance
        aligned = "left";
    elseif angle_difference < -tolerance
        aligned = "right";
    else
        aligned = "aligned";
    end
end

%Forward Function
function goForwards(speed, wheels)
    for i = 1:4
        wb_motor_set_velocity(wheels(i), speed);
    end
end

%Stop Function
function stop(wheels)
    for i = 1:4
        wb_motor_set_velocity(wheels(i), 0);
    end
end

%Turn Left Function
function turnLeft(speed, wheels)
    % Left wheels
    wb_motor_set_velocity(wheels(1), -speed); 
    wb_motor_set_velocity(wheels(2), -speed); 

    % Right wheels
    wb_motor_set_velocity(wheels(3), speed);  
    wb_motor_set_velocity(wheels(4), speed); 
end

%Turn Right Function
function turnRight(speed, wheels)
    % Left wheels
    wb_motor_set_velocity(wheels(1), speed); 
    wb_motor_set_velocity(wheels(2), speed); 

    % Right wheels
    wb_motor_set_velocity(wheels(3), -speed);  
    wb_motor_set_velocity(wheels(4), -speed); 
end
