wb_robot_init();

timestep = wb_robot_get_basic_time_step();

% Get motors
motor1 = wb_robot_get_device('motor_1');
motor2 = wb_robot_get_device('motor_2');
motor3 = wb_robot_get_device('motor_3');

% Get sensors
sensor1 = wb_robot_get_device('pos_sensor_1');
sensor2 = wb_robot_get_device('pos_sensor_2');
sensor3 = wb_robot_get_device('pos_sensor_3');

% Enable sensors
wb_position_sensor_enable(sensor1, timestep);
wb_position_sensor_enable(sensor2, timestep);
wb_position_sensor_enable(sensor3, timestep);

%Get and Enable GPS End Effector
end_effector = wb_robot_get_device('End_Effector');
wb_gps_enable(end_effector, timestep);


wb_robot_step(timestep);

% Target joint angles
theta1 = 0.5;
theta2 = 0.3;
theta3 = -0.2;

%Wrap around 
if theta1 > 3.14
  theta1 = 3.14
elseif theta1 < -3.14
  theta1 = -3.14
end

if theta2 > 4.1888
    theta2 = 4.1888;
elseif theta2 < -1.0472
    theta2 = -1.0472;
end

if theta3 > 4.1888
    theta3 = 4.1888;
elseif theta3 < -1.0472
    theta3 = -1.0472;
end

% Tolerance
tolerance = 0.001;

%Set Motor Speed
m_speed = 0.8;


%Set Motor Positions and Velocities
wb_motor_set_position(motor3, theta3);
wb_motor_set_position(motor2, theta2);
wb_motor_set_position(motor1, theta1);

wb_motor_set_velocity(motor3, m_speed);
wb_motor_set_velocity(motor2, m_speed);
wb_motor_set_velocity(motor1, m_speed);


% Move joints
time = 0;
while wb_robot_step(timestep) ~= -1
    current1 = wb_position_sensor_get_value(sensor1);
    current3 = wb_position_sensor_get_value(sensor3);
    current2 = wb_position_sensor_get_value(sensor2);
    time = time + timestep/1000;

    if abs(current1 - theta1) < tolerance && abs(current2 - theta2) < tolerance && abs(current3 - theta3) < tolerance
        disp('All joints reached targets');
        break;
    end
    if time>=7.5
      disp('Reached Max Time Limit');
      break;
    end
end


while wb_robot_step(timestep) ~= -1
end

wb_robot_cleanup();