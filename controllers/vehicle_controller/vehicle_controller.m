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


%Threshold for obstacle detection
distance_threshold = 900;
turning = false;
turn_delay = 0.5;

while wb_robot_step(TIME_STEP) ~= -1
    % Get current position and distance sensor readings
    position = getPosition(vehicle_gps);
    ds_values = getDS(distance_sensors);

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
    else
        goForwards(SPEED, wheels);
    end

    fprintf("DS_Left: %.2f, DS_Middle: %.2f, DS_Right: %.2f\n", ds_values(1), ds_values(2), ds_values(3));

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
