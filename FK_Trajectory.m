function way_point = FK_Trajectory(theta_A, theta_B, robot_config)  
    [x, y, z] = ForwardKinematics(theta_A(1), theta_A(2), theta_A(3), theta_A(4), robot_config);
    way_point_A = [x, y, z];
    [x, y, z] = ForwardKinematics(theta_B(1), theta_B(2), theta_B(3), theta_B(4), robot_config);
    way_point_B = [x, y, z];
    Qk = [theta_A(1) theta_B(1);...
          theta_A(2) theta_B(2);...
          -pi/2      -pi/2     ;...
          theta_A(3) theta_B(3);...
          theta_A(4) theta_B(4)];
    tt = 0:0.01:3;
    t_knots = [0, 3];
    
    pp_joints = spline(t_knots, Qk);
    q = ppval(pp_joints, tt);
    q_row = q';
    
    pos = zeros(3, size(q_row, 1));
    for k = 1 : size(q_row, 1)
        t1 = q_row(k, 1);
        t2 = q_row(k, 2);
        t3 = q_row(k, 4);
        t4 = q_row(k, 5);
        
        [x, y, z] = ForwardKinematics(t1, t2, t3, t4, robot_config);
        pos(:, k) = [x, y, z];
    end
    
    hold on;
    way_point = plot3(pos(1,:), pos(2,:), pos(3,:), '-');
    
    P = [way_point_A' way_point_B'];
    scatter3(P(1,:), P(2,:), P(3,:), 50, 'r', 'filled');
    xlabel('X'); 
    ylabel('Y'); 
    zlabel('Z'); 
    grid on; axis equal;
    legend('FK of robot', 'waypoints');
        
    hold on; 
    robot_config.model.plot(q', 'workspace', [-0.1 0.6 -0.1 0.6 -0.3 0.3],...
                     'view', [45,30], 'delay', 0.005);
end