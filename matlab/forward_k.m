function End_Effector = forward_k(theta1, theta2, theta3)

%Correct theta 1 and 2
theta1 = theta1 + pi;
theta2 = theta2 + pi/2;  

% Link Dimensions
d1 = 0.4;
a2 = 0.3;
a3 = 0.4;

% Pedestal Offset
ped_offset = [1 0 0 0;
          0 1 0 0;
          0 0 1 0.1;
          0 0 0 1];

% Transformation matrix: 0 - 1
Transform_01 = [cos(theta1), -sin(theta1)*cos(pi/2), sin(theta1)*sin(pi/2), 0*cos(theta1);
sin(theta1), cos(theta1)*cos(pi/2), -cos(theta1)*sin(pi/2), 0*sin(theta1);
0, sin(pi/2), cos(pi/2), d1;
0, 0, 0, 1];

% Transformation matrix: 1 - 2
Transform_12 = [cos(theta2), -sin(theta2)*cos(0), sin(theta2)*sin(0), a2*cos(theta2);
sin(theta2), cos(theta2)*cos(0), -cos(theta2)*sin(0), a2*sin(theta2);
0, sin(0), cos(0), 0;
0, 0, 0, 1];

% Transformation matrix: 2 - 3
Transform_23 = [cos(theta3), -sin(theta3)*cos(0), sin(theta3)*sin(0), a3*cos(theta3);
sin(theta3), cos(theta3)*cos(0), -cos(theta3)*sin(0), a3*sin(theta3);
0, sin(0), cos(0), 0;
0, 0, 0, 1];

% Final
End_Effector = ped_offset * Transform_01 * Transform_12 * Transform_23;
End_Effector = End_Effector(1:3, 4)
end