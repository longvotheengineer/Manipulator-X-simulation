function handle = InitModel(robot_config)    
    delete(findall(robot_config.UIAxes, 'Type', 'light'));
    light('Position',[1 -1 1], 'Style', 'infinite', 'Parent', robot_config.UIAxes);
    lighting(robot_config.UIAxes, 'gouraud'); 
    material(robot_config.UIAxes, 'dull');
    
    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    L4 = robot_config.robot_length.L4;
    L5 = robot_config.robot_length.L5; 
    L = [L1-0.03 0.07 L2 L3 L4 L5 0.3]; 
    W = [0.04 0.02 0.02 0.02 0.03 0.04 0.05]; 
    H = [0.06 0.02 0.03 0.03 0.03 0.04 0.05];

    for i = 1 : 7
        if i == 2
            vertices{i} = [-W(i)/2  -H(i)/2      0;      
                           -W(i)/2  -H(i)/2   L(i);  
                            W(i)/2  -H(i)/2   L(i);   
                            W(i)/2  -H(i)/2      0;      
                           -W(i)/2   H(i)/2      0;      
                           -W(i)/2   H(i)/2   L(i);   
                            W(i)/2   H(i)/2   L(i);   
                            W(i)/2   H(i)/2      0]; 
        elseif i == 3 
            L(i) = L(i) + 0.015;
            vertices{i} = [ -W(i)/2      0  -H(i)/2;  
                            -W(i)/2   L(i)  -H(i)/2; 
                             W(i)/2   L(i)  -H(i)/2;  
                             W(i)/2      0  -H(i)/2; 
                            -W(i)/2      0   H(i)/2; 
                            -W(i)/2   L(i)   H(i)/2; 
                             W(i)/2   L(i)   H(i)/2;  
                             W(i)/2      0   H(i)/2];
            L(i) = L(i) - 0.015;           
        else
            vertices{i} = [-W(i)/2  -H(i)/2      0;      
                           -W(i)/2  -H(i)/2   L(i);  
                            W(i)/2  -H(i)/2   L(i);   
                            W(i)/2  -H(i)/2      0;      
                           -W(i)/2   H(i)/2      0;      
                           -W(i)/2   H(i)/2   L(i);   
                            W(i)/2   H(i)/2   L(i);   
                            W(i)/2   H(i)/2      0];       
        end
        faces{i} = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    end
    
    handle.robot.base = hgtransform('Parent', robot_config.UIAxes);
    Mb = makehgtform('zrotate', -pi/2); 
    set(handle.robot.base, 'Matrix', Mb);

    handle.robot.link0 = hgtransform('Parent', handle.robot.base);
    patch('Vertices', vertices{7}, 'Faces', faces{7}, 'FaceColor', [0.2 0.2 0.2],...
          'Parent', handle.robot.link0, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    M_trans = makehgtform('translate', [0, 0, -0.345]);
    M7 = M_trans;
    set(handle.robot.link0, 'Matrix', M7);
       
    handle.robot.link11 = hgtransform('Parent', handle.robot.base);
    patch('Vertices', vertices{1}, 'Faces', faces{1}, 'FaceColor', [0.10, 0.35, 0.80],...
          'Parent', handle.robot.link11, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    
    v_plate = CreateBoxVertices(0.08, 0.08, 0.005, -0.005);
    patch('Vertices', v_plate, 'Faces', faces{1}, ...
          'FaceColor', [0.10, 0.35, 0.80], 'Parent', handle.robot.link11, ...
          'EdgeColor', 'none');
          
    r_bolt = 0.003; h_bolt = 0.006;
    offsets = [0.03 0.03; 0.03 -0.03; -0.03 0.03; -0.03 -0.03];
    for k=1:4
       DrawCylinder(handle.robot.link11, r_bolt, h_bolt, [0.8 0.8 0.8], [offsets(k,1), offsets(k,2), 0]);
    end

    handle.robot.joint1 = hgtransform('Parent', handle.robot.base);

    J1_radius = 0.018; 
    J1_height = 0.03;  
    [j1x, j1y, j1z] = cylinder(J1_radius, 20);
    j1z = j1z * J1_height - J1_height/2;
    surf(j1x, j1y, j1z+0.01, 'Parent', handle.robot.joint1,...
        'FaceColor', [0.85, 0.10, 0.10],... 
        'EdgeColor', 'none'); 
    patch(j1x(1,:), j1y(1,:), j1z(1,:)+0.01, [0.85, 0.10, 0.10], 'Parent', handle.robot.joint1, 'EdgeColor', 'none');
    patch(j1x(2,:), j1y(2,:), j1z(2,:)+0.01, [0.85, 0.10, 0.10], 'Parent', handle.robot.joint1, 'EdgeColor', 'none');

    handle.robot.link12 = hgtransform('Parent', handle.robot.joint1);
    L_bracket = 0.07; 
    W_bracket = 0.012;
    H_bracket = 0.020; 
    offset_side = 0.022; 
    v_side_L = CreateBoxVertices(W_bracket, H_bracket, L_bracket, 0);
    v_side_L(:,1) = v_side_L(:,1) - offset_side; 
    patch('Vertices', v_side_L, 'Faces', faces{1}, 'FaceColor', [0.10, 0.35, 0.80], ...
          'Parent', handle.robot.link12, 'EdgeColor', 'none');     
    v_side_R = CreateBoxVertices(W_bracket, H_bracket, L_bracket, 0);
    v_side_R(:,1) = v_side_R(:,1) + offset_side; 
    patch('Vertices', v_side_R, 'Faces', faces{1}, 'FaceColor', [0.10, 0.35, 0.80], ...
          'Parent', handle.robot.link12, 'EdgeColor', 'none');

    J2_radius = 0.02; 
    J2_height = 0.04;  
    [j2x_temp, j2y_temp, j2z_temp] = cylinder(J2_radius, 20);
    j2z_temp = j2z_temp * J2_height - J2_height/2;
    j2x = j2z_temp;
    j2y = j2y_temp;
    j2z = j2x_temp;
    surf(j2x, j2y, j2z+0.06, 'Parent', handle.robot.link12,...
        'FaceColor', [0.85, 0.10, 0.10],... 
        'EdgeColor', 'none'); 
    patch(j2x(1,:), j2y(1,:), j2z(1,:)+0.06, [0.85, 0.10, 0.10], 'Parent', handle.robot.link12, 'EdgeColor', 'none');
    patch(j2x(2,:), j2y(2,:), j2z(2,:)+0.06, [0.85, 0.10, 0.10], 'Parent', handle.robot.link12, 'EdgeColor', 'none');

    handle.robot.link2 = hgtransform('Parent', handle.robot.joint1); 
    patch('Vertices', vertices{3}, 'Faces', faces{3}, 'FaceColor', [0.10, 0.35, 0.80],...
          'Parent', handle.robot.link2, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

    handle.robot.link3 = hgtransform('Parent', handle.robot.link2); 
    patch('Vertices', vertices{4}, 'Faces', faces{4}, 'FaceColor', [0.10, 0.35, 0.80],...
          'Parent', handle.robot.link3, 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    
    J3_radius = 0.02; 
    J3_height = 0.04;  
    [j3x_temp, j3y_temp, j3z_temp] = cylinder(J3_radius, 20);
    j3z_temp = j3z_temp * J3_height - J3_height/2;
    j3x = j3z_temp;
    j3y = j3y_temp;
    j3z = j3x_temp;
    surf(j3x, j3y, j3z, 'Parent', handle.robot.link3,...
        'FaceColor', [0.85, 0.10, 0.10],... 
        'EdgeColor', 'none'); 
    patch(j3x(1,:), j3y(1,:), j3z(1,:), [0.85, 0.10, 0.10], 'Parent', handle.robot.link3, 'EdgeColor', 'none');
    patch(j3x(2,:), j3y(2,:), j3z(2,:), [0.85, 0.10, 0.10], 'Parent', handle.robot.link3, 'EdgeColor', 'none');

    handle.robot.link4 = hgtransform('Parent', handle.robot.link3); 
    patch('Vertices', vertices{5}, 'Faces', faces{5}, 'FaceColor', [0.10, 0.35, 0.80],...
          'Parent', handle.robot.link4, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

    J4_radius = 0.02; 
    J4_height = 0.06;  
    [j4x_temp, j4y_temp, j4z_temp] = cylinder(J4_radius, 20);
    j4z_temp = j4z_temp * J4_height - J4_height/2;
    j4x = j4z_temp;
    j4y = j4y_temp;
    j4z = j4x_temp;
    surf(j4x, j4y, j4z+L4, 'Parent', handle.robot.link4,...
        'FaceColor', [0.85, 0.10, 0.10],... 
        'EdgeColor', 'none'); 
    patch(j4x(1,:), j4y(1,:), j4z(1,:)+L4, [0.85, 0.10, 0.10], 'Parent', handle.robot.link4, 'EdgeColor', 'none');
    patch(j4x(2,:), j4y(2,:), j4z(2,:)+L4, [0.85, 0.10, 0.10], 'Parent', handle.robot.link4, 'EdgeColor', 'none');
    
    handle.robot.link5 = hgtransform('Parent', handle.robot.link4);     
    W_base = 0.05; H_base = 0.03; L_base = 0.04;
    
    v_base = [ -W_base/2  -H_base/2      0;      
               -W_base/2  -H_base/2   L_base;  
                W_base/2  -H_base/2   L_base;   
                W_base/2  -H_base/2      0;      
               -W_base/2   H_base/2      0;      
               -W_base/2   H_base/2   L_base;   
                W_base/2   H_base/2   L_base;   
                W_base/2   H_base/2      0];       
    f_box = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    
    patch('Vertices', v_base, 'Faces', f_box, 'Parent', handle.robot.link5, ...
          'FaceColor', [0.10, 0.35, 0.80], 'FaceAlpha', 0.7, 'EdgeColor', 'none');

    W_finger = 0.008; 
    H_finger = 0.025; 
    L_finger = 0.08;  
    Base_Gap = 0.04;  
    Grip_Angle = 15;  

    v_finger_local = [ -W_finger/2  -H_finger/2      0;      
                       -W_finger/2  -H_finger/2   L_finger;  
                        W_finger/2  -H_finger/2   L_finger;   
                        W_finger/2  -H_finger/2      0;      
                       -W_finger/2   H_finger/2      0;      
                       -W_finger/2   H_finger/2   L_finger;   
                        W_finger/2   H_finger/2   L_finger;   
                        W_finger/2   H_finger/2      0]; 
    
    h_finger_L = hgtransform('Parent', handle.robot.link5);
    M_pos_L = makehgtform('translate', [-Base_Gap/2, 0, L_base]);
    M_rot_L = makehgtform('yrotate', deg2rad(Grip_Angle));    
    set(h_finger_L, 'Matrix', M_pos_L * M_rot_L);  
    patch('Vertices', v_finger_local, 'Faces', f_box, 'Parent', h_finger_L, ...
          'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    h_finger_R = hgtransform('Parent', handle.robot.link5);
    M_pos_R = makehgtform('translate', [Base_Gap/2, 0, L_base]);
    M_rot_R = makehgtform('yrotate', deg2rad(-Grip_Angle));
    set(h_finger_R, 'Matrix', M_pos_R * M_rot_R);
    patch('Vertices', v_finger_local, 'Faces', f_box, 'Parent', h_finger_R, ...
          'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    M_trans = makehgtform('translate', [0, 0, -0.04]);
    M11 = M_trans;
    set(handle.robot.link11, 'Matrix', M11);

    M_trans = makehgtform('translate', [0, 0, 0]);
    Mj1 = M_trans;
    set(handle.robot.joint1, 'Matrix', Mj1);

    M_trans = makehgtform('translate', [0, 0, 0.01]);
    M12 = M_trans;
    set(handle.robot.link12, 'Matrix', M12);

    M_trans = makehgtform('translate', [0, 0, L1]);
    M2 = M_trans;
    set(handle.robot.link2, 'Matrix', M2);
    
    M_trans = makehgtform('translate', [0, L2, -0.02]);
    M_rotate = makehgtform('xrotate', 0);
    M3 = M_trans * M_rotate;
    set(handle.robot.link3, 'Matrix', M3);
    
    M_trans = makehgtform('translate', [0, 0, 0]);
    M_rotate = makehgtform('xrotate', pi);
    M4 = M_trans * M_rotate;
    set(handle.robot.link4, 'Matrix', M4);
    
    M_trans = makehgtform('translate', [0, 0, L4]);
    M5 = M_trans;
    set(handle.robot.link5, 'Matrix', M5);

    handle.vec = reference_frame(robot_config);
end

function handle = reference_frame(robot_config)
    [pos_joint, A_i_0] = ForwardKinematics(0, 0, 0, 0, robot_config);
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
    handle = [handle_vecx, handle_vecy, handle_vecz];
end

function v = CreateBoxVertices(w, h, l, z_offset)
    v = [-w/2 -h/2 z_offset; 
         -w/2 -h/2 l+z_offset; 
          w/2 -h/2 l+z_offset; 
          w/2 -h/2 z_offset; 
         -w/2  h/2 z_offset; 
         -w/2  h/2 l+z_offset; 
          w/2  h/2 l+z_offset; 
          w/2  h/2 z_offset];
end

function DrawCylinder(parent_handle, r, h, color, offset, axis_orient)
    if nargin < 6 
        axis_orient = 'z'; 
    end 

    [cx, cy, cz] = cylinder(r, 20);
    cz = cz * h - h/2; 
    
    if strcmp(axis_orient, 'x')
        temp = cx; cx = cz; cz = temp;
    elseif strcmp(axis_orient, 'y')
        temp = cy; cy = cz; cz = temp;
    end
    
    cx = cx + offset(1);
    cy = cy + offset(2);
    cz = cz + offset(3);
    
    surf(cx, cy, cz, 'Parent', parent_handle, ...
         'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
         
    patch(cx(1,:), cy(1,:), cz(1,:), color, 'Parent', parent_handle, 'EdgeColor', 'none');
    patch(cx(2,:), cy(2,:), cz(2,:), color, 'Parent', parent_handle, 'EdgeColor', 'none');
end