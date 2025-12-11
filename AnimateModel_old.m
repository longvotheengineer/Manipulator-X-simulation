function handle = AnimateModel_old(theta_A, theta_B, robot_config)    
    handle = struct();
    resolution_waypoint = 50;

    pos_joint1 = linspace(theta_A(1), theta_B(1), resolution_waypoint);
    pos_joint2 = linspace(theta_A(2), theta_B(2), resolution_waypoint);
    pos_joint3 = linspace(theta_A(3), theta_B(3), resolution_waypoint);
    pos_joint4 = linspace(theta_A(4), theta_B(4), resolution_waypoint);

    [pos_joint, A_i_0] = ForwardKinematics(pos_joint1(1), pos_joint2(1), pos_joint3(1), pos_joint4(1), robot_config);
    vec_scale = 0.15;
    vecorg = pos_joint([1:2 4:6], :);

    vecx = [A_i_0{1}(1:3, 1), A_i_0{2}(1:3, 1), A_i_0{3}(1:3, 1), A_i_0{4}(1:3, 1), A_i_0{5}(1:3, 1)]' * vec_scale;
    vecy = [A_i_0{1}(1:3, 2), A_i_0{2}(1:3, 2), A_i_0{3}(1:3, 2), A_i_0{4}(1:3, 2), A_i_0{5}(1:3, 2)]' * vec_scale;
    vecz = [A_i_0{1}(1:3, 3), A_i_0{2}(1:3, 3), A_i_0{3}(1:3, 3), A_i_0{4}(1:3, 3), A_i_0{5}(1:3, 3)]' * vec_scale;

    handle_vecx = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecx(:,1), vecx(:,2), vecx(:,3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecy = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecy(:,1), vecy(:,2), vecy(:,3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecz = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecz(:,1), vecz(:,2), vecz(:,3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle.vec = [handle_vecx, handle_vecy, handle_vecz];
    
    x_data = pos_joint(:, 1);
    y_data = pos_joint(:, 2);
    z_data = pos_joint(:, 3);
    pos = zeros(3, resolution_waypoint);
    pos(:, 1) = pos_joint(6, :);
    
    handle.robot = plot3(robot_config.UIAxes, x_data, y_data, z_data, 'o-r', 'LineWidth', 10, 'MarkerSize', 1); 
    handle.waypoint = plot3(robot_config.UIAxes, pos(1,1), pos(2,1), pos(3,1), '-');

    for i = 2 : resolution_waypoint
        [pos_joint, A_i_0] = ForwardKinematics(pos_joint1(i), pos_joint2(i), pos_joint3(i), pos_joint4(i), robot_config);

        x_data = pos_joint(:, 1);
        y_data = pos_joint(:, 2);
        z_data = pos_joint(:, 3);
        pos(:, i) = pos_joint(6, :);
        
        vecorg = pos_joint([1:2 4:6], :);
        vecx = [A_i_0{1}(1:3, 1), A_i_0{2}(1:3, 1), A_i_0{3}(1:3, 1), A_i_0{4}(1:3, 1), A_i_0{5}(1:3, 1)]' * vec_scale;
        vecy = [A_i_0{1}(1:3, 2), A_i_0{2}(1:3, 2), A_i_0{3}(1:3, 2), A_i_0{4}(1:3, 2), A_i_0{5}(1:3, 2)]' * vec_scale;
        vecz = [A_i_0{1}(1:3, 3), A_i_0{2}(1:3, 3), A_i_0{3}(1:3, 3), A_i_0{4}(1:3, 3), A_i_0{5}(1:3, 3)]' * vec_scale;

        set(handle.vec(1), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecx(:,1), 'VData', vecx(:,2), 'WData', vecx(:,3));
        set(handle.vec(2), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecy(:,1), 'VData', vecy(:,2), 'WData', vecy(:,3));
        set(handle.vec(3), 'XData', vecorg(:,1), 'YData', vecorg(:,2), 'ZData', vecorg(:,3), 'UData', vecz(:,1), 'VData', vecz(:,2), 'WData', vecz(:,3));

        set(handle.waypoint, 'XData', pos(1,1:i), 'YData', pos(2,1:i), 'ZData', pos(3,1:i));
        set(handle.robot, 'XData', x_data, 'YData', y_data, 'ZData', z_data);
        drawnow;
        pause(0.005);
    end
end