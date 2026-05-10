wb_robot_init();
timestep = wb_robot_get_basic_time_step();

%Original Box Position
box = wb_supervisor_node_get_from_def('Target_Box');
original_box_position = wb_supervisor_node_get_position(box);


% ACW2 Locating Vehicle and Placing Box
% Initialise receiver and emitter
receiver = wb_robot_get_device('grabber_receiver');
emitter = wb_robot_get_device('grabber_emitter');
wb_receiver_enable(receiver, timestep);

% Get Vehicle Position 
vehicle_node = wb_supervisor_node_get_from_def('VEHICLE');


% Gripper GPS
gripper_gps = wb_robot_get_device('Gripper_gps');
wb_gps_enable(gripper_gps, timestep);
gripper_position = wb_gps_get_values(gripper_gps);
gripper_x = gripper_position(1);
gripper_y = gripper_position(2);
gripper_z = gripper_position(3);
zero_position = [gripper_x, gripper_y, 1.3];


% Send Grabber Position
message_sent = false;
while ~message_sent && wb_robot_step(timestep) ~= -1
  if isnumeric(gripper_x) && isnumeric(gripper_y)
    message = sprintf('%f,%f,%f', gripper_x, gripper_y, 0);
    wb_emitter_send(emitter, uint8(message));
    fprintf("Sent Grabber Position: %s\n", message);
    message_sent = true;
  end
end

% Original ACW1 had gripper at 0,0,0, now gripper is at gripper_x, gripper_y, 0 so correct positions
arm_position = [gripper_x, gripper_y, gripper_z];
original_box_position = [original_box_position(1) - gripper_x, original_box_position(2) - gripper_y, original_box_position(3) - 0.01];
zero_position = [zero_position(1) - gripper_x, zero_position(2) - gripper_y, zero_position(3)];

while wb_robot_step(timestep) ~= -1
  % Check for messages
  if wb_receiver_get_queue_length(receiver) > 0
    % Read Message
    message = wb_receiver_get_data(receiver, 'string');

    if strcmp(message, 'Vehicle Arrived')

      fprintf("Vehicle Arrived\n");

      % Get Vehicle Position
      received_vehicle_position = false;
      while ~received_vehicle_position
        vehicle_position = wb_supervisor_node_get_position(vehicle_node);
        if numel(vehicle_position) == 3 && all(isfinite(vehicle_position))
          received_vehicle_position = true;
        end
      end

      % Original ACW1 had gripper at 0,0,0, now gripper is at gripper_x, gripper_y, 0 so correct positions
      vehicle_position = [vehicle_position(1) - gripper_x, vehicle_position(2) - gripper_y, vehicle_position(3) + 0.1];

      vehicle_x = vehicle_position(1);
      vehicle_y = vehicle_position(2);
      vehicle_z = vehicle_position(3);




      fprintf("Vehicle Position: %f, %f, %f\n", vehicle_x, vehicle_y, vehicle_z);
      fprintf("Target Box Position: %f, %f, %f\n", original_box_position(1), original_box_position(2), original_box_position(3));

      %Main Logic
      move_arm_to_position(original_box_position(1), original_box_position(2), original_box_position(3), "send");
      close_gripper();
      move_arm_to_position(zero_position(1), zero_position(2), zero_position(3), "return");
      move_arm_to_position(vehicle_position(1), vehicle_position(2), vehicle_position(3), "send");
      open_gripper();
      move_arm_to_position(zero_position(1), zero_position(2), zero_position(3), "return");
    end

    % Clear message
    wb_receiver_next_packet(receiver);
  end
end

%Gripper Functions
function open_gripper()
  timestep = wb_robot_get_basic_time_step();

  %Get Gripper Motors
  right_gripper_motor = wb_robot_get_device('gripper_right_motor');
  left_gripper_motor  = wb_robot_get_device('gripper_left_motor');
  
  %Set Gripper Force
  grip_force = 10;
  wb_motor_set_available_force(right_gripper_motor, grip_force)
  wb_motor_set_available_force(left_gripper_motor, grip_force)
  
  % Get Gripper sensors
  right_gripper_sensor = wb_robot_get_device('gripper_right_sensor');
  left_gripper_sensor  = wb_robot_get_device('gripper_left_sensor');
  
  % Enable sensors
  wb_position_sensor_enable(right_gripper_sensor, timestep);
  wb_position_sensor_enable(left_gripper_sensor, timestep);
  
  wb_robot_step(timestep);
  
  opened_position = 0;
  gripper_motor_speed = 0.1;
  wb_motor_set_velocity(right_gripper_motor, gripper_motor_speed);
  wb_motor_set_velocity(left_gripper_motor, gripper_motor_speed);
  wb_motor_set_position(right_gripper_motor, opened_position);
  wb_motor_set_position(left_gripper_motor, opened_position);
  
  time = 0;
  while wb_robot_step(timestep) ~= -1
    time = time + timestep/1000;
    if time>=2
      break
    end
  end
  
  
end

function close_gripper()
  timestep = wb_robot_get_basic_time_step();

  %Get Gripper Motors
  right_gripper_motor = wb_robot_get_device('gripper_right_motor');
  left_gripper_motor  = wb_robot_get_device('gripper_left_motor');
  
  %Set Gripper Force
  grip_force = 10;
  wb_motor_set_available_force(right_gripper_motor, grip_force)
  wb_motor_set_available_force(left_gripper_motor, grip_force)
  
  % Get Gripper sensors
  right_gripper_sensor = wb_robot_get_device('gripper_right_sensor');
  left_gripper_sensor  = wb_robot_get_device('gripper_left_sensor');
  
  % Enable sensors
  wb_position_sensor_enable(right_gripper_sensor, timestep);
  wb_position_sensor_enable(left_gripper_sensor, timestep);
  
  wb_robot_step(timestep);
  
  closed_position = 0.1;
  gripper_motor_speed = 0.1;
  wb_motor_set_velocity(right_gripper_motor, gripper_motor_speed);
  wb_motor_set_velocity(left_gripper_motor, gripper_motor_speed);
  wb_motor_set_position(right_gripper_motor, closed_position);
  wb_motor_set_position(left_gripper_motor, closed_position);
  
  time = 0;
  while wb_robot_step(timestep) ~= -1
    time = time + timestep/1000;
    if time>=2
      break
    end
  end
end

%Robot Functions
function move_arm_to_position(x, y, z, direction)
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
 

  
  %INVERSE KINEMATIC 
  % Link Dimensions
  ped_offset = 0.1;
  a1 = 0.4;
  a2 = 0.3;
  a3 = 0.5;
  
  % Base rotation (link 1)
  theta1 = atan2(y, x);
  
  % Reduce to 2D Plane 
  hor_distance = sqrt(x^2 + y^2);
  vert_distance = z - ped_offset - a1;
  
  % Distance
  diagonal_distance = (hor_distance^2 + vert_distance^2 - a2^2 - a3^2) / (2 * a2 * a3);
  diagonal_distance = max(-1, min(1, diagonal_distance));
  
  % Elbow (link 3)
  theta3 = acos(diagonal_distance);    
  
  % Shoulder (link 2)
  theta2 = atan2(hor_distance, vert_distance) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
  
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
  m_speed = 0.5;
  
  
  %Set Motor Positions and Velocities


  if direction == "send"  
    
    % Move joints
    time = 0;
    while wb_robot_step(timestep) ~= -1
        current1 = wb_position_sensor_get_value(sensor1);
        wb_motor_set_position(motor1, theta1);
        wb_motor_set_velocity(motor1, m_speed);


        time = time + timestep/1000;
    
        if abs(current1 - theta1) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end

    time = 0;
    while wb_robot_step(timestep) ~= -1
        current2 = wb_position_sensor_get_value(sensor2);
        time = time + timestep/1000;
        wb_motor_set_position(motor2, theta2);
        wb_motor_set_velocity(motor2, m_speed);
    
        if abs(current2 - theta2) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end

    time = 0;
    while wb_robot_step(timestep) ~= -1
        current3 = wb_position_sensor_get_value(sensor3);
        time = time + timestep/1000;
        wb_motor_set_position(motor3, theta3);
        wb_motor_set_velocity(motor3, m_speed);
    
        if abs(current3 - theta3) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end
  end

  if direction == "return"
  
    time = 0;
    while wb_robot_step(timestep) ~= -1
        current3 = wb_position_sensor_get_value(sensor3);
        time = time + timestep/1000;
        wb_motor_set_position(motor3, theta3);
        wb_motor_set_velocity(motor3, m_speed);
    
        if abs(current3 - theta3) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end

    time = 0;
    while wb_robot_step(timestep) ~= -1
        current2 = wb_position_sensor_get_value(sensor2);
        time = time + timestep/1000;
        wb_motor_set_position(motor2, theta2);
        wb_motor_set_velocity(motor2, m_speed);
    
        if abs(current2 - theta2) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end

    time = 0;
    while wb_robot_step(timestep) ~= -1
        current1 = wb_position_sensor_get_value(sensor1);
        wb_motor_set_position(motor1, theta1);
        wb_motor_set_velocity(motor1, m_speed);


        time = time + timestep/1000;
    
        if abs(current1 - theta1) < tolerance
            break;
        end
        if time>=7.5
          break;
        end
    end
  end
end
    



wb_robot_cleanup();




