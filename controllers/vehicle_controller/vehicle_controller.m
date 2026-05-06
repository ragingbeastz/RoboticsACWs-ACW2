TIME_STEP = 64;
SPEED = 3.0;

% Get motors by their Webots names
wheel_back_left = wb_robot_get_device('wheel_back_left');
wheel_front_left = wb_robot_get_device('wheel_front_left');
wheel_back_right = wb_robot_get_device('wheel_back_right');
wheel_front_right = wb_robot_get_device('wheel_front_right');

motors = [
    wheel_back_left
    wheel_front_left
    wheel_back_right
    wheel_front_right
];

% Set all wheels to velocity control mode
for i = 1:4
    wb_motor_set_position(motors(i), inf);
    wb_motor_set_velocity(motors(i), 0);
end

while wb_robot_step(TIME_STEP) ~= -1

    % Try this first
    wb_motor_set_velocity(wheel_back_left, SPEED);
    wb_motor_set_velocity(wheel_front_left, SPEED);
    wb_motor_set_velocity(wheel_back_right, SPEED);
    wb_motor_set_velocity(wheel_front_right, SPEED);

end

wb_robot_cleanup();