function [psi, theta, phi] = FixedAngleOrientation(theta1, theta2, theta3, theta4, robot_config)
    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    L4 = robot_config.robot_length.L4;
    L5 = robot_config.robot_length.L5;

    A_5_0 =...
    [sin(theta2 + theta3 + theta4)*cos(theta1), cos(theta2 + theta3 + theta4)*cos(theta1),  sin(theta1), cos(theta1)*(L4*sin(theta2 + theta3) + L2*cos(theta2) + L3*sin(theta2) + L5*sin(theta2 + theta3 + theta4));...
     sin(theta2 + theta3 + theta4)*sin(theta1), cos(theta2 + theta3 + theta4)*sin(theta1), -cos(theta1), sin(theta1)*(L4*sin(theta2 + theta3) + L2*cos(theta2) + L3*sin(theta2) + L5*sin(theta2 + theta3 + theta4));...
                -cos(theta2 + theta3 + theta4),             sin(theta2 + theta3 + theta4),            0,          L1 - L4*cos(theta2 + theta3) - L3*cos(theta2) + L2*sin(theta2) - L5*cos(theta2 + theta3 + theta4);...
                                             0,                                         0,            0,                                                                                                          1];
  
    theta = atan2(-A_5_0(3,1), sqrt(A_5_0(1,1)^2 + A_5_0(2,1)^2));
    psi = atan2(A_5_0(3,2) / cos(theta), A_5_0(3,3) / cos(theta));
    phi = atan2(A_5_0(2,1) / cos(theta), A_5_0(1,1) / cos(theta));
end