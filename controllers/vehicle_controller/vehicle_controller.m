TIME_STEP = 64;
SPEED = 1.5;

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

% Intialise the Vehicle Emitter and receiver
vehicle_emitter = wb_robot_get_device('vehicle_emitter');
vehicle_receiver = wb_robot_get_device('vehicle_receiver');
wb_receiver_enable(vehicle_receiver, TIME_STEP);
sent_message = false;


%A* Algorithm Variables (m)
test = 3;
if test == 1 || test == 2
    % Test 1-2
    cell_size = 0.03;
    floor_x = 3;
    floor_y = 3;
end

% Test 3
if test == 3
    cell_size = 0.075;
    floor_x = 7.5;
    floor_y = 7.5;
end

grid_width = floor_x / cell_size;
grid_height = floor_y / cell_size;
grid = zeros(grid_height, grid_width);
obstacle_count = 0;
penalty_grid = zeros(size(grid));
last_astar_time = wb_robot_get_time();
path = [];
astar_position_x = 0;
astar_position_y = 0;
astar_turning = false;
grid_obstacle_enlargening_amount = 3;

% Goal Position from Grabber GPS
gripper_position = [inf,inf,inf];
received_gripper_position = false;

% Receive gripper position
while ~received_gripper_position && wb_robot_step(TIME_STEP) ~= -1
    if ~received_gripper_position
        if wb_receiver_get_queue_length(vehicle_receiver) > 0
            % Read Message as string, convert to position
            received_message = wb_receiver_get_data(vehicle_receiver, 'string');
            gripper_position = sscanf(received_message, '%f,%f,%f');
            fprintf("Received Gripper Position: %f, %f, %f\n", gripper_position(1), gripper_position(2), gripper_position(3));

            if numel(gripper_position) == 3 && all(isfinite(gripper_position))
                received_gripper_position = true;
            end
            wb_receiver_next_packet(vehicle_receiver);
        end
    end
end

gripper_x = gripper_position(1);
gripper_y = gripper_position(2);

goal_x = gripper_x;
goal_y = gripper_y ;
goal_threshold = 0.4;
goal_reached = false;
grid_based_goal_x = floor((goal_x + floor_x/2) / cell_size) + 1;
grid_based_goal_y = floor((goal_y + floor_y/2) / cell_size) + 1;
grid(grid_based_goal_y, grid_based_goal_x) = 2;


% Debug Display
grid_display = wb_robot_get_device('grid_display');

%Obstacle Avoidance Variables
distance_threshold = 800;
cleared_threshold = 900;
turn_direction = "None";
turn_chosen = false;
turn_degree_rad = pi/8;
target_turn_direction = inf;
turned_to_direction = false;
turn_tolerance = 0.05;
started_forward_motion = false;
forward_motion_time = 0.4;
forward_motion_start_time = inf;
finished_obstacle_avoidance = false;
front_blocked = false;   

state = "ASTAR";
previous_state = state;

while wb_robot_step(TIME_STEP) ~= -1
    if ~goal_reached
        % Get current position and distance sensor readings
        position = getPosition(vehicle_gps);
        rpy = getOrientation(vehicle_inertial_unit);
        ds_values = getDS(distance_sensors);
        left_sensor = ds_values(1);
        middle_sensor = ds_values(2);   
        right_sensor = ds_values(3);
        highest_sensor = max([left_sensor, right_sensor]);

        % Check if goal is reached
        distance_to_goal = sqrt((position(1) - goal_x)^2 + (position(2) - goal_y)^2);
        if distance_to_goal < goal_threshold
            stop(wheels);
            fprintf('Goal Reached!\n');
            goal_reached = true;


            % Signal Grabber to Grab
            message = uint8('Vehicle Arrived');
            if ~sent_message
                wb_emitter_send(vehicle_emitter, uint8(message));
                sent_message = true;
                fprintf("Sent Message: %s\n", message);
            end            

            continue;
        end

        if any(ds_values < distance_threshold) && ~astar_turning && state ~="Stuck"
            state = "OBSTACLEAVOIDANCE";
            previous_state = state;
        end
        if previous_state == "OBSTACLEAVOIDANCE" 
            if ~any(ds_values < cleared_threshold)
                state = "OBSTACLEAVOIDANCE";
                previous_state = state;
            end
        end


        fprintf("State: %s | Left: %2f | Middle: %2f | Right: %2f | Highest: %2f\n | Turn Direction: %s" , state, left_sensor, middle_sensor, right_sensor, highest_sensor, turn_direction);


        if state == "OBSTACLEAVOIDANCE"
            % Choose turn
            if ~turn_chosen
                if highest_sensor == left_sensor
                    turn_direction = "Left";
                    turnLeft(SPEED, wheels);
                else
                    turn_direction = "Right";
                    turnRight(SPEED, wheels);
                end
                turn_chosen = true;
            end


            % % Go forward a bit
            % if middle_sensor > cleared_threshold
            %     if ~started_forward_motion
            %         forward_motion_start_time = wb_robot_get_time();
            %         started_forward_motion = true;
            %     end
            %     if wb_robot_get_time() - forward_motion_start_time < forward_motion_time && middle_sensor > distance_threshold
            %         goForwards(SPEED, wheels);
            %     else
            %         stop(wheels);
            %         finished_obstacle_avoidance = true;
            %     end
            % end
            
            finished_obstacle_avoidance = true;

            % Reset
            if finished_obstacle_avoidance
                turn_chosen = false;
                state = "ASTAR";
                turned_to_direction = false;
                target_turn_direction = inf;
                started_forward_motion = false;
                forward_motion_start_time = inf;
                finished_obstacle_avoidance = false;
            end
        end



        if state == "ASTAR"
            %Update grid map with current sensor readings
            % tic;
            grid = mapGridCoordinates(position, ds_values, rpy(3), cell_size, grid, floor_x, floor_y, grid_obstacle_enlargening_amount);
            % fprintf("Time taken to update grid: %f\n", toc);
            

            % Run A* algorithm every second to get path to goal
            % tic;
            grid_based_position_x = floor((position(1) + floor_x/2) / cell_size) + 1;
            grid_based_position_y = floor((position(2) + floor_y/2) / cell_size) + 1;
            path = aStarSearch(grid, grid_based_position_x, grid_based_position_y, grid_based_goal_x, grid_based_goal_y);
            % fprintf("Time taken to find path: %f\n", toc);
            
            
            % Update Grid Display
            % tic;
            updateGridDisplay(grid_display, grid, path, grid_based_position_x, grid_based_position_y, goal_x, goal_y);
            % fprintf("Time taken to update display: %f\n", toc);
            
            tic;
            % Check if path is empty
            if isempty(path)
                stop(wheels);
                continue;
            end
            % fprintf("Time taken to check if path is empty: %f\n", toc);

            % tic;
            % Convert paths from grid coordinates to real world coordinates
            real_path = [];
            for i = 1:size(path, 1)
                [real_x, real_y] = gridToRealCoordinates(path(i, 1), path(i, 2), cell_size, floor_x, floor_y);
                real_path = [real_path; real_x, real_y];
            end
            % fprintf("Time taken to convert path: %f\n", toc);

            % tic;
            % Find Next point in path to current position
            if size(real_path, 1) >= 2
                index = min(5, size(real_path, 1));
                closest_path_x = real_path(index, 1);
                closest_path_y = real_path(index, 2);
            else
                closest_path_x = real_path(1, 1);
                closest_path_y = real_path(1, 2);
            end
            % fprintf("Time taken to find next point: %f\n", toc);

            % Find angle to next point and turn towards it if not facing it, then move forward
            [alligned, angle_to_target, angle_difference] = turnToTarget(closest_path_x, closest_path_y, position, rpy(3));
            if alligned == "left"
                turnLeft(SPEED, wheels);
            elseif alligned == "right"
                turnRight(SPEED, wheels);
            else
                goForwards(SPEED, wheels);
            end
            previous_state = "ASTAR";
        end
    end
end

wb_robot_cleanup();


% Update Grid Display Function
function updateGridDisplay(display, grid, path, vehicle_grid_location_x, vehicle_grid_location_y, goal_x, goal_y)
    
    % Get size of grid
    [grid_rows, grid_columns] = size(grid);

    % Get size of display
    width = wb_display_get_width(display);
    height = wb_display_get_height(display);

    % Get size of each cell
    cell_width = width / grid_rows;
    cell_height = height / grid_columns;

    % Get size of each drawn cell
    drawn_cell_width = max(1, ceil(cell_width));
    drawn_cell_height = max(1, ceil(cell_height));

    % Clear display
    wb_display_set_color(display, [1 1 1]);
    wb_display_fill_rectangle(display, 0, 0, width, height);

    % Draw grid
    for y = 1:grid_rows
        for x = 1:grid_columns
            

            pixel_x = round((y - 1) * cell_width);
            pixel_y = round((x - 1) * cell_height);
            
            % Fill cells for obstacles
            if grid(y, x) == 1
                wb_display_set_color(display, [0 0 0]);
                wb_display_fill_rectangle(display, pixel_x, pixel_y, drawn_cell_width, drawn_cell_height);
            
            % Fill cells for free space
            elseif grid(y, x) == 2
                wb_display_set_color(display, [0 1 0]);
                wb_display_fill_rectangle(display, pixel_x, pixel_y, drawn_cell_width, drawn_cell_height);
            end
        end
    end

    % Draw path in red
    wb_display_set_color(display, [1 0 0]);
    for i = 1:size(path, 1)
        x = path(i, 1);
        y = path(i, 2);

        pixel_x = round((y - 1) * cell_width);
        pixel_y = round((x - 1) * cell_height);

        wb_display_fill_rectangle(display, pixel_x, pixel_y, drawn_cell_width, drawn_cell_height);
    end


    marker_size = 5;

    % Draw vehicle in green
    wb_display_set_color(display, [0 1 0]);
    pixel_x = round((vehicle_grid_location_y - 1) * cell_width);
    pixel_y = round((vehicle_grid_location_x - 1) * cell_height);
    wb_display_fill_rectangle(display, pixel_x, pixel_y, drawn_cell_width+marker_size, drawn_cell_height+marker_size);

    % Draw Goal in Yellow
    for x = 1:grid_rows
        for y = 1:grid_columns
            if grid(y, x) == 2
                pixel_x = round((y - 1) * cell_width);
                pixel_y = round((x - 1) * cell_height);
                wb_display_set_color(display, [1 1 0]);
                wb_display_fill_rectangle(display, pixel_x, pixel_y, drawn_cell_width+marker_size, drawn_cell_height+marker_size);
            end
        end
    end


end

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

    [y_size, x_size] = size(grid);

    % Create unexplored and explored arrays by initializing them to false for all cells
    unexplored = false(y_size, x_size);
    explored = false(y_size, x_size);

    % Create parent, g, and f score arrays
    g_costs = inf(y_size, x_size);
    f_costs = inf(y_size, x_size);
    parents_x = zeros(y_size, x_size);
    parents_y = zeros(y_size, x_size);


    % Add start cell
    unexplored(start_y, start_x) = true;
    g_costs(start_y, start_x) = 0;
    f_costs(start_y, start_x) = getHeuristic(start_x, start_y, goal_x, goal_y);


    % Iterate through the grid if there are still unexplored cells
    unexplored_list = [start_x, start_y];
    while ~isempty(unexplored_list)

        % Choose cell with lowest f_cost
        unexplored_f_costs = zeros(size(unexplored_list, 1), 1);

        for i = 1:size(unexplored_list, 1)
            temp_x = unexplored_list(i, 1);
            temp_y = unexplored_list(i, 2);
            unexplored_f_costs(i) = f_costs(temp_y, temp_x);
        end
        [~, min_f_cost_index] = min(unexplored_f_costs);

        current_cell_x = unexplored_list(min_f_cost_index, 1);
        current_cell_y = unexplored_list(min_f_cost_index, 2);


        % Check if its the goal
        if current_cell_y == goal_y && current_cell_x == goal_x
            break;
        end


        % Move current cell from unexplored to explored
        unexplored_list(min_f_cost_index, :) = [];
        unexplored(current_cell_y, current_cell_x) = false;
        explored(current_cell_y, current_cell_x) = true;


        % Get neighbours
        neighbours = getneighbours(current_cell_x, current_cell_y, grid);

        % Check each neighbour
        for i = 1:size(neighbours, 1)
            neighbour_x = neighbours(i, 1);
            neighbour_y = neighbours(i, 2);

            % If neighbour is already explored, skip
            if explored(neighbour_y, neighbour_x)
                continue;
            end
            
            % Calculate movement penalty
            if grid(neighbour_y, neighbour_x) == 3
                neighbour_penalty = 1000;
            else
                neighbour_penalty = 0;
            end

            % Calculate g_cost 
            new_g_cost = g_costs(current_cell_y, current_cell_x) + 1 + neighbour_penalty;

            % If neighbour is unexplored, add to unexplored
            if ~unexplored(neighbour_y, neighbour_x)
                unexplored(neighbour_y, neighbour_x) = true;
                unexplored_list = [unexplored_list; neighbour_x, neighbour_y];
            
            % If neighbour is already unexplored but has a higher g_cost, skip
            elseif new_g_cost >= g_costs(neighbour_y, neighbour_x)
                continue;
            end

            % Update parent, g_cost, and f_cost
            parents_x(neighbour_y, neighbour_x) = current_cell_x;
            parents_y(neighbour_y, neighbour_x) = current_cell_y;
            g_costs(neighbour_y, neighbour_x) = new_g_cost;
            f_costs(neighbour_y, neighbour_x) = new_g_cost + getHeuristic(neighbour_x, neighbour_y, goal_x, goal_y);
        end

    end

    % Reconstruct path from goal to start using parent map
    path = [];
    current_cell_x = goal_x;
    current_cell_y = goal_y;
    
    tic;
    while true
        % Add parent to path
        path = [[current_cell_x, current_cell_y]; path];

        % If current cell is start cell, break
        if current_cell_x == start_x && current_cell_y == start_y
            break;
        end
        
        % Get Parent of current cell
        parent_x = parents_x(current_cell_y, current_cell_x);
        parent_y = parents_y(current_cell_y, current_cell_x);

        % Update current cell to parent cell
        current_cell_x = parent_x;
        current_cell_y = parent_y;
    end
    % fprintf("Time taken to reconstruct path: %f\n", toc);

end

function heuristic = getHeuristic(x1, y1, x2, y2)
    %Using Manhattan Distance as heuristic
    heuristic =  sqrt((x1 - x2)^2 + (y1 - y2)^2);
end

function valid_neighbours = getneighbours(cell_x, cell_y, grid)

    %Get Neighbouring Cells
    neighbours = [
        cell_x - 1, cell_y;     % Left
        cell_x + 1, cell_y;     % Right
        cell_x, cell_y - 1;     % Up
        cell_x, cell_y + 1;     % Down
    ];

    % Loop through neighbours and check if they are within grid bounds and not obstacles
    valid_neighbours = [];
    for i = 1:size(neighbours, 1)
        neighbour_x = neighbours(i, 1);
        neighbour_y = neighbours(i, 2);

        if neighbour_x < 1 || neighbour_x > size(grid, 2)
            continue;
        end
        if neighbour_y < 1 || neighbour_y > size(grid, 1)
            continue;
        end
        if grid(neighbour_y, neighbour_x) == 1
            continue;
        end
        valid_neighbours = [valid_neighbours; neighbours(i, :)];
    end

end

%Plot positions on the grid using gps position and distance sensor readings, to be used for A* algorithm
function grid=mapGridCoordinates(gps_position, distance_sensor_values, yaw,  cell_size, grid, floor_x, floor_y, grid_obstacle_enlargening_amount)
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

    %Using grid layout: 0=free, 1=obstacle, 2=goal, 3=near_obstacle)

    %Handle Left Sensor
    if left_sensor_grid_x >= 1 && left_sensor_grid_x <= size(grid, 2) && left_sensor_grid_y >= 1 && left_sensor_grid_y <= size(grid, 1)
        if grid(left_sensor_grid_y, left_sensor_grid_x) ~= 2
            if distance_sensor_values(1) < 1000
                grid = addObstacle(grid, left_sensor_grid_x, left_sensor_grid_y, grid_obstacle_enlargening_amount);
            end
        end
    end

    %Handle Middle Sensor
    if middle_sensor_grid_x >= 1 && middle_sensor_grid_x <= size(grid, 2) && middle_sensor_grid_y >= 1 && middle_sensor_grid_y <= size(grid, 1)
        if grid(middle_sensor_grid_y, middle_sensor_grid_x) ~= 2
            if distance_sensor_values(2) < 1000
                grid = addObstacle(grid, middle_sensor_grid_x, middle_sensor_grid_y, grid_obstacle_enlargening_amount);
            end
        end
    end

    %Handle Right Sensor
    if right_sensor_grid_x >= 1 && right_sensor_grid_x <= size(grid, 2) && right_sensor_grid_y >= 1 && right_sensor_grid_y <= size(grid, 1)
        if grid(right_sensor_grid_y, right_sensor_grid_x) ~= 2
            if distance_sensor_values(3) < 1000
                grid = addObstacle(grid, right_sensor_grid_x, right_sensor_grid_y, grid_obstacle_enlargening_amount);
            end
        end
    end
end

function grid=addObstacle(grid, obstacle_x, obstacle_y, grid_obstacle_enlargening_amount)
    near_obstacle_search_size = grid_obstacle_enlargening_amount;

    % Find Boundaries
    x_min = max(1, obstacle_x - near_obstacle_search_size);
    x_max = min(size(grid, 2), obstacle_x + near_obstacle_search_size);
    y_min = max(1, obstacle_y - near_obstacle_search_size);
    y_max = min(size(grid, 1), obstacle_y + near_obstacle_search_size); 

    for y = y_min:y_max
        for x = x_min:x_max

            if grid(y,x) ~= 2
                if x == obstacle_x && y == obstacle_y
                    grid(y,x) = 1;   
                elseif grid(y,x) ~= 1
                    grid(y,x) = 3;   
                end
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
