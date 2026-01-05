function handle = PathPlanning(theta_A, theta_B, robot_config, model)
    handle.robot = model;
    waypoint_n = robot_config.waypoint_n;

    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    L4 = robot_config.robot_length.L4;
    L5 = robot_config.robot_length.L5;

    T = robot_config.runtime;
    N = waypoint_n;
    switch robot_config.motion_type
        case "Cubic"
            trajPVA = PathCubic(theta_A, theta_B, T, N);
        otherwise
    end

    handle.PosEE = plot(robot_config.UIAxesPosEE, nan, nan, 'LineWidth', 2);
    handle.VelEE = plot(robot_config.UIAxesVelEE, nan, nan, 'LineWidth', 2);
    handle.AccEE = plot(robot_config.UIAxesAccEE, nan, nan, 'LineWidth', 2);
    handle.PosJoint1 = plot(robot_config.UIAxesPosJoint1, nan, nan, 'LineWidth', 2);
    handle.VelJoint1 = plot(robot_config.UIAxesVelJoint1, nan, nan, 'LineWidth', 2);
    handle.AccJoint1 = plot(robot_config.UIAxesAccJoint1, nan, nan, 'LineWidth', 2);
    handle.PosJoint2 = plot(robot_config.UIAxesPosJoint2, nan, nan, 'LineWidth', 2);
    handle.VelJoint2 = plot(robot_config.UIAxesVelJoint2, nan, nan, 'LineWidth', 2);
    handle.AccJoint2 = plot(robot_config.UIAxesAccJoint2, nan, nan, 'LineWidth', 2);
    handle.PosJoint3 = plot(robot_config.UIAxesPosJoint3, nan, nan, 'LineWidth', 2);
    handle.VelJoint3 = plot(robot_config.UIAxesVelJoint3, nan, nan, 'LineWidth', 2);
    handle.AccJoint3 = plot(robot_config.UIAxesAccJoint3, nan, nan, 'LineWidth', 2);
    handle.PosJoint4 = plot(robot_config.UIAxesPosJoint4, nan, nan, 'LineWidth', 2);
    handle.VelJoint4 = plot(robot_config.UIAxesVelJoint4, nan, nan, 'LineWidth', 2);
    handle.AccJoint4 = plot(robot_config.UIAxesAccJoint4, nan, nan, 'LineWidth', 2);

    xData_PosEE = [];
    yData_PosEE = [];
    xData_VelEE = [];
    yData_VelEE = [];
    xData_AccEE = [];
    yData_AccEE = [];
    xData_PosJoint1 = [];
    yData_PosJoint1 = [];
    xData_VelJoint1 = [];
    yData_VelJoint1 = [];
    xData_AccJoint1 = [];
    yData_AccJoint1 = [];
    xData_PosJoint2 = [];
    yData_PosJoint2 = [];
    xData_VelJoint2 = [];
    yData_VelJoint2 = [];
    xData_AccJoint2 = [];
    yData_AccJoint2 = [];
    xData_PosJoint3 = [];
    yData_PosJoint3 = [];
    xData_VelJoint3 = [];
    yData_VelJoint3 = [];
    xData_AccJoint3 = [];
    yData_AccJoint3 = [];
    xData_PosJoint4 = [];
    yData_PosJoint4 = [];
    xData_VelJoint4 = [];
    yData_VelJoint4 = [];
    xData_AccJoint4 = [];
    yData_AccJoint4 = [];

    theta1_waypoint = trajPVA.theta(:,1);  
    theta2_waypoint = trajPVA.theta(:,2);  
    theta3_waypoint = trajPVA.theta(:,3);  
    theta4_waypoint = trajPVA.theta(:,4); 

    J = {};
    for i = 1 : waypoint_n
        % J{i} = [                         -sin(theta1_waypoint(i)) * (L4*sin(theta2_waypoint(i) + theta3_waypoint(i)) + L2*cos(theta2_waypoint(i)) + L3*sin(theta2_waypoint(i)) + L5*sin(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,... 
        %                                 -cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                              % -cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                                           -cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                                                                                            -cos(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
        %                                   cos(theta1_waypoint(i)) * (L4*sin(theta2_waypoint(i) + theta3_waypoint(i)) + L2*cos(theta2_waypoint(i)) + L3*sin(theta2_waypoint(i)) + L5*sin(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,... 
        %                                 -sin(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                              % -sin(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                                           -sin(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                                                                                            -sin(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
        %                                                                                                                                                                                                                                            0   ,...
        %        2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                     % 2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                  2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
        %                                                                                                                   2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
        %                                                                                                                                                                                                                                            0   ,...
        %                                                                                                                                                                                                                        sin(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                        % sin(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                        sin(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                        sin(theta1_waypoint(i)) ;...
        %                                                                                                                                                                                                                                             0  ,...
        %                                                                                                                                                                                                                       -cos(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                       % -cos(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                       -cos(theta1_waypoint(i)) ,...
        %                                                                                                                                                                                                                       -cos(theta1_waypoint(i)) ;...
        %                                                                                                                                                                                                                                             1  ,...
        %                                                                                                                                                                                                                                             0  ,...
        %                                                                                                                                                                                                                                             % 0  ,...
        %                                                                                                                                                                                                                                             0  ,...
        %                                                                                                                                                                                                                                             0  ];
        J{i} = [                         -sin(theta1_waypoint(i)) * (L4*sin(theta2_waypoint(i) + theta3_waypoint(i)) + L2*cos(theta2_waypoint(i)) + L3*sin(theta2_waypoint(i)) + L5*sin(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,... 
                                        -cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...                                                               
                                                                                                  -cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
                                                                                                                                                   -cos(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
                                          cos(theta1_waypoint(i)) * (L4*sin(theta2_waypoint(i) + theta3_waypoint(i)) + L2*cos(theta2_waypoint(i)) + L3*sin(theta2_waypoint(i)) + L5*sin(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,... 
                                        -sin(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...                                                                   
                                                                                                  -sin(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
                                                                                                                                                   -sin(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
                                                                                                                                                                                                                                                   0   ,...
               2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L3*cos(theta2_waypoint(i)) + L2*sin(theta2_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...                                        
                                                                         2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (-L4*cos(theta2_waypoint(i) + theta3_waypoint(i)) - L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ,...
                                                                                                                          2*sin(theta1_waypoint(i))*cos(theta1_waypoint(i)) * (- L5*cos(theta2_waypoint(i) + theta3_waypoint(i) + theta4_waypoint(i))) ;...
                                                                                                                                                                                                                                                   0   ,...
                                                                                                                                                                                                                               sin(theta1_waypoint(i)) ,...                                                                                                                                                                                                                             
                                                                                                                                                                                                                               sin(theta1_waypoint(i)) ,...
                                                                                                                                                                                                                               sin(theta1_waypoint(i)) ;...
                                                                                                                                                                                                                                                    0  ,...
                                                                                                                                                                                                                              -cos(theta1_waypoint(i)) ,...                                                                                                                                                                                                                             
                                                                                                                                                                                                                              -cos(theta1_waypoint(i)) ,...
                                                                                                                                                                                                                              -cos(theta1_waypoint(i)) ;...
                                                                                                                                                                                                                                                    1  ,...
                                                                                                                                                                                                                                                    0  ,...                                                                                                                                                                                                                                                    
                                                                                                                                                                                                                                                    0  ,...
                                                                                                                                                                                                                                                    0  ];
        
        jacobian_matrix{i} = J{i} * trajPVA.v(i,:)';
    end

    [pos_joint, A_i_0] = ForwardKinematics(theta1_waypoint(1), theta2_waypoint(1), theta3_waypoint(1), theta4_waypoint(1), robot_config);
    pos = zeros(3, waypoint_n);
    pos(:, 1) = pos_joint(6, :);
    handle.waypoint = plot3(robot_config.UIAxes, pos(1,1), pos(2,1), pos(3,1), '-');
    
    RotX_neg90 = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    A_i_0{1}(1:3, 1:3) = A_i_0{1}(1:3, 1:3) * RotX_neg90;
    vec_scale = 0.15;
    vecorg = pos_joint([1:2 4:6], :);
    vecx   = [A_i_0{1}(1:3, 1), A_i_0{2}(1:3, 1), A_i_0{3}(1:3, 1), A_i_0{4}(1:3, 1), A_i_0{5}(1:3, 1)]' * vec_scale;
    vecy   = [A_i_0{1}(1:3, 2), A_i_0{2}(1:3, 2), A_i_0{3}(1:3, 2), A_i_0{4}(1:3, 2), A_i_0{5}(1:3, 2)]' * vec_scale;
    vecz   = [A_i_0{1}(1:3, 3), A_i_0{2}(1:3, 3), A_i_0{3}(1:3, 3), A_i_0{4}(1:3, 3), A_i_0{5}(1:3, 3)]' * vec_scale;
    handle_vecx = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecx(:,1), vecx(:,2), vecx(:,3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecy = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecy(:,1), vecy(:,2), vecy(:,3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle_vecz = quiver3(robot_config.UIAxes, vecorg(:,1), vecorg(:,2), vecorg(:,3), vecz(:,1), vecz(:,2), vecz(:,3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    handle.vec  = [handle_vecx, handle_vecy, handle_vecz];

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
        
        xData_PosEE(end+1) = T*i/waypoint_n;
        yData_PosEE(end+1) = sqrt(pos_joint(6,1)^2 + pos_joint(6,2)^2 + pos_joint(6,3)^2);
        set(handle.PosEE, 'XData', xData_PosEE, 'YData', yData_PosEE);
        xData_VelEE(end+1) = T*i/waypoint_n;
        yData_VelEE(end+1) = sqrt(jacobian_matrix{i}(1)^2 + jacobian_matrix{i}(2)^2 + jacobian_matrix{i}(3)^2);
        set(handle.VelEE, 'XData', xData_VelEE, 'YData', yData_VelEE);
        xData_AccEE(end+1) = T*i/waypoint_n;
        yData_AccEE(end+1) = sqrt(jacobian_matrix{i}(4)^2 + jacobian_matrix{i}(5)^2 + jacobian_matrix{i}(6)^2);
        set(handle.AccEE, 'XData', xData_AccEE, 'YData', yData_AccEE);

        xData_PosJoint1(end+1) = T*i/waypoint_n;
        yData_PosJoint1(end+1) = theta1_waypoint(i);
        set(handle.PosJoint1, 'XData', xData_PosJoint1, 'YData', yData_PosJoint1);
        xData_VelJoint1(end+1) = T*i/waypoint_n;
        yData_VelJoint1(end+1) = trajPVA.v(i,1);
        set(handle.VelJoint1, 'XData', xData_VelJoint1, 'YData', yData_VelJoint1);
        xData_AccJoint1(end+1) = T*i/waypoint_n;
        yData_AccJoint1(end+1) = trajPVA.a(i,1);
        set(handle.AccJoint1, 'XData', xData_AccJoint1, 'YData', yData_AccJoint1);

        xData_PosJoint2(end+1) = T*i/waypoint_n;
        yData_PosJoint2(end+1) = theta2_waypoint(i);
        set(handle.PosJoint2, 'XData', xData_PosJoint2, 'YData', yData_PosJoint2);
        xData_VelJoint2(end+1) = T*i/waypoint_n;
        yData_VelJoint2(end+1) = trajPVA.v(i,2);
        set(handle.VelJoint2, 'XData', xData_VelJoint2, 'YData', yData_VelJoint2);
        xData_AccJoint2(end+1) = T*i/waypoint_n;
        yData_AccJoint2(end+1) = trajPVA.a(i,2);
        set(handle.AccJoint2, 'XData', xData_AccJoint2, 'YData', yData_AccJoint2);

        xData_PosJoint3(end+1) = T*i/waypoint_n;
        yData_PosJoint3(end+1) = theta3_waypoint(i);
        set(handle.PosJoint3, 'XData', xData_PosJoint3, 'YData', yData_PosJoint3);
        xData_VelJoint3(end+1) = T*i/waypoint_n;
        yData_VelJoint3(end+1) = trajPVA.v(i,3);
        set(handle.VelJoint3, 'XData', xData_VelJoint3, 'YData', yData_VelJoint3);
        xData_AccJoint3(end+1) = T*i/waypoint_n;
        yData_AccJoint3(end+1) = trajPVA.a(i,3);
        set(handle.AccJoint3, 'XData', xData_AccJoint3, 'YData', yData_AccJoint3);
        
        xData_PosJoint4(end+1) = T*i/waypoint_n;
        yData_PosJoint4(end+1) = theta4_waypoint(i);
        set(handle.PosJoint4, 'XData', xData_PosJoint4, 'YData', yData_PosJoint4);
        xData_VelJoint4(end+1) = T*i/waypoint_n;
        yData_VelJoint4(end+1) = trajPVA.v(i,4);
        set(handle.VelJoint4, 'XData', xData_VelJoint4, 'YData', yData_VelJoint4);
        xData_AccJoint4(end+1) = T*i/waypoint_n;
        yData_AccJoint4(end+1) = trajPVA.a(i,4);
        set(handle.AccJoint4, 'XData', xData_AccJoint4, 'YData', yData_AccJoint4);

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
