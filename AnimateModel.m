function handle = AnimateModel(theta_A, theta_B, robot_config, model)  
    handle.robot = model;
    waypoint_n = robot_config.waypoint_n;

    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    L4 = robot_config.robot_length.L4;
    L5 = robot_config.robot_length.L5;
         
    theta1_waypoint = linspace(theta_A(1), theta_B(1), waypoint_n);  
    theta2_waypoint = linspace(theta_A(2), theta_B(2), waypoint_n); 
    theta3_waypoint = linspace(theta_A(3), theta_B(3), waypoint_n); 
    theta4_waypoint = linspace(theta_A(4), theta_B(4), waypoint_n); 

    [pos_joint, A_i_0] = ForwardKinematics(theta1_waypoint(1), theta2_waypoint(1), theta3_waypoint(1), theta4_waypoint(1), robot_config);
    pos = zeros(3, waypoint_n);
    pos(:, 1) = pos_joint(6, :);
    handle.waypoint = plot3(robot_config.UIAxes, pos(1,1), pos(2,1), pos(3,1), '-');
    
    RotX_neg90 = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    A_i_0{1}(1:3, 1:3) = A_i_0{1}(1:3, 1:3) * RotX_neg90;
    vec_scale = 0.15;
    vecorg = pos_joint([1:2 4:6], :);
    vecx = [A_i_0{1}(1:3, 1), A_i_0{2}(1:3, 1), A_i_0{3}(1:3, 1), A_i_0{4}(1:3, 1), A_i_0{5}(1:3, 1)]' * vec_scale;
    vecy = [A_i_0{1}(1:3, 2), A_i_0{2}(1:3, 2), A_i_0{3}(1:3, 2), A_i_0{4}(1:3, 2), A_i_0{5}(1:3, 2)]' * vec_scale;
    vecz = [A_i_0{1}(1:3, 3), A_i_0{2}(1:3, 3), A_i_0{3}(1:3, 3), A_i_0{4}(1:3, 3), A_i_0{5}(1:3, 3)]' * vec_scale;
    handle_vecx = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecx(:,1), vecx(:,2), vecx(:,3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecy = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecy(:,1), vecy(:,2), vecy(:,3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecz = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecz(:,1), vecz(:,2), vecz(:,3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle.vec = [handle_vecx, handle_vecy, handle_vecz];

    if isfield(model, 'vec') && ~isempty(model.vec)
        delete(model.vec);
    end
    if robot_config.reference_frame == "on"
        set(handle.vec, 'Visible', 'on');
    else
        set(handle.vec, 'Visible', 'off');
    end

    for i = 1 : waypoint_n
        [pos_joint, A_i_0] = ForwardKinematics(theta1_waypoint(i), theta2_waypoint(i), theta3_waypoint(i), theta4_waypoint(i), robot_config);
        pos(:, i) = pos_joint(6, :);
        set(handle.waypoint, 'XData', pos(1,1:i), 'YData', pos(2,1:i), 'ZData', pos(3,1:i));
        
        t1 = theta1_waypoint(i); 
        t2 = theta2_waypoint(i); 
        t3 = theta3_waypoint(i) + pi; 
        t4 = theta4_waypoint(i); 
    
        M_trans = makehgtform('translate', [0, 0, 0]);
        M_rotate = makehgtform('zrotate', t1);
        M1 = M_trans * M_rotate;
        set(handle.robot.joint1, 'Matrix', M1);
    
        M_trans = makehgtform('translate', [0, 0, L1]);
        M_rotate = makehgtform('xrotate', t2);
        M2 = M_trans * M_rotate;
        set(handle.robot.link2, 'Matrix', M2);
    
        M_trans = makehgtform('translate', [0, 0, 0]);
        M_rotate = makehgtform('Xrotate', t3);    
        M4 = M_trans * M_rotate;
        set(handle.robot.link4, 'Matrix', M4);
    
        M_trans = makehgtform('translate', [0, 0, L4]);
        M_rotate = makehgtform('xrotate', t4);    
        M5 = M_trans * M_rotate;
        set(handle.robot.link5, 'Matrix', M5);

        RotX_neg90 = [1, 0, 0; 0, 0, 1; 0, -1, 0];
        A_i_0{1}(1:3, 1:3) = A_i_0{1}(1:3, 1:3) * RotX_neg90;
        vecorg = pos_joint([1:2 4:6], :);
        vecx = [A_i_0{1}(1:3, 1), A_i_0{2}(1:3, 1), A_i_0{3}(1:3, 1), A_i_0{4}(1:3, 1), A_i_0{5}(1:3, 1)]' * vec_scale;
        vecy = [A_i_0{1}(1:3, 2), A_i_0{2}(1:3, 2), A_i_0{3}(1:3, 2), A_i_0{4}(1:3, 2), A_i_0{5}(1:3, 2)]' * vec_scale;
        vecz = [A_i_0{1}(1:3, 3), A_i_0{2}(1:3, 3), A_i_0{3}(1:3, 3), A_i_0{4}(1:3, 3), A_i_0{5}(1:3, 3)]' * vec_scale;
        set(handle.vec(1), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecx(:,1), 'VData', vecx(:,2), 'WData', vecx(:,3));
        set(handle.vec(2), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecy(:,1), 'VData', vecy(:,2), 'WData', vecy(:,3));
        set(handle.vec(3), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecz(:,1), 'VData', vecz(:,2), 'WData', vecz(:,3));
        
        drawnow;
        pause(0.01);
    end
end

