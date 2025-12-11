function WorkSpace_Scatter = WorkSpace(robot_config)
    joint_limit_1 = linspace(robot_config.jointlimit.j1l, robot_config.jointlimit.j1u, 10);
    joint_limit_2 = linspace(robot_config.jointlimit.j2l, robot_config.jointlimit.j2u, 10);
    joint_limit_3 = linspace(robot_config.jointlimit.j3l, robot_config.jointlimit.j3u, 10);
    joint_limit_4 = linspace(robot_config.jointlimit.j4l, robot_config.jointlimit.j4u, 10);
    
    points = [];
    
    for theta1 = joint_limit_1
        for theta2 = joint_limit_2
            for theta3 = joint_limit_3
                for theta4 = joint_limit_4
                    switch robot_config.workspace
                        case "operation-workspace"
                            req_theta4 = robot_config.pitch - theta2 - theta3;
                            if req_theta4 >= -2*pi && req_theta4 <= 2*pi
                                pos_joint = ForwardKinematics(theta1, theta2, theta3, theta4, robot_config);
                                p = [pos_joint(6,1), pos_joint(6,2), pos_joint(6,3)];
                                points = [points; p];
                            end
                        case "joint-workspace"
                            pos_joint = ForwardKinematics(theta1, theta2, theta3, theta4, robot_config);
                            p = [pos_joint(6,1), pos_joint(6,2), pos_joint(6,3)];
                            points = [points; p];
                        otherwise
                    end
                end
            end
        end
    end
    
    WorkSpace_Scatter = scatter3(robot_config.UIAxes, points(:, 1), points(:, 2), points(:, 3), 8, 'filled');
end