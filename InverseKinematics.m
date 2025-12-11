function theta_i = InverseKinematics(px, py, pz, robot_config)   
    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    L4 = robot_config.robot_length.L4;
    L5 = robot_config.robot_length.L5; 

    % syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz;
    % 
    % FK_5_0 = [r11 r12 r13 px;...
    %           r21 r22 r23 py;...
    %           r31 r32   0 pz;...
    %             0   0   0  1];
    % 
    % FK_5_1 = simplify(inv(A_1_0) * FK_5_0);
    % FK_5_2 = simplify(inv(A_2_1) * FK_5_1);
    % FK_5_3 = simplify(inv(A_3_2) * FK_5_2);
    % FK_5_4 = simplify(inv(A_4_3) * FK_5_3);
    % 
    % A_5_1 = simplify(A_2_1 * A_3_2 * A_4_3 * A_5_4);
    % A_5_2 = simplify(A_3_2 * A_4_3 * A_5_4);
    % A_5_3 = simplify(A_4_3 * A_5_4);
    % A_5_4 = simplify(A_5_4);
    
    theta_i.theta1 = atan2(py, px);
    % theta1 = atan2(-py, px);
    
    % theta234 = pi/2;
    theta234 = robot_config.pitch;
    
    px4 = px - cos(theta_i.theta1)*L5*sin(theta234);
    py4 = py - sin(theta_i.theta1)*L5*sin(theta234);
    pz4 = pz + L5*cos(theta234);
    
    A = 2*L3*L4;
    B = 2*L2*L4;
    C = py4^2/(sin(theta_i.theta1))^2 + ((pz4 - L1)^2 - (L2^2 + L3^2 + L4^2));
    % theta_i.theta3 = atan2(B, A) + atan2(sqrt(A^2 + B^2 - C^2), C);
    theta_i.theta3 = atan2(B, A) + atan2(-sqrt(A^2 + B^2 - C^2), C);
    
    A = -L4*cos(theta_i.theta3) - L3;
    B = L4*sin(theta_i.theta3) + L2;
    C = pz4 - L1;
    % theta_i.theta2 = atan2(B, A) + atan2(sqrt(A^2 + B^2 - C^2), C);
    theta_i.theta2 = atan2(B, A) + atan2(-sqrt(A^2 + B^2 - C^2), C);
    
    theta_i.theta4 = theta234 - theta_i.theta2 - theta_i.theta3;
end    

