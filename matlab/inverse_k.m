function [theta1, theta2, theta3] = inverse_k(x, y, z)

% Link Dimensions
ped_offset = 0.1;
a1 = 0.4;
a2 = 0.3;
a3 = 0.4;

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

end