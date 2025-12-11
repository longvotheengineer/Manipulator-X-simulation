function handle = AnimateModel(theta_A, theta_B, robot_config)  

robot_config = struct();
robot_config.robot_length = struct('L1', 0.077,...
                                   'L2', 0.128,...
                                   'L3', 0.024,...
                                   'L4', 0.124,...
                                   'L5', 0.126);

figure; 
h_axes = axes('XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[-1.5 1.5]);
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; 
axis([-0.2 0.3 -0.2 0.2 -0.3 0.3]);
view(45, 30); hold on; 

L1 = robot_config.robot_length.L1;
L2 = robot_config.robot_length.L2;
L3 = robot_config.robot_length.L3;
L4 = robot_config.robot_length.L4;
L5 = robot_config.robot_length.L5; 
L = [L1 L2 L3 L4 L5]; 
W = [0.05 0.01 0.01 0.02 0.05]; 
H = [0.05 0.01 0.01 0.02 0.05];

for i = 1 : 5
    if i == 2 
        vertices{i} = [ -W(i)/2      0  -H(i)/2;  
                        -W(i)/2   L(i)  -H(i)/2; 
                         W(i)/2   L(i)  -H(i)/2;  
                         W(i)/2      0  -H(i)/2; 
                        -W(i)/2      0   H(i)/2; 
                        -W(i)/2   L(i)   H(i)/2; 
                         W(i)/2   L(i)   H(i)/2;  
                         W(i)/2      0   H(i)/2];
       
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

h_base = hgtransform('Parent', h_axes);
Mb = makehgtform('zrotate', -pi/2); 
set(h_base, 'Matrix', Mb);

h_link1 = hgtransform('Parent', h_base);
patch('Vertices', vertices{1}, 'Faces', faces{1}, 'FaceColor', 'red', ...
      'Parent', h_link1, 'FaceAlpha', 0.8);

h_link2 = hgtransform('Parent', h_link1); 
patch('Vertices', vertices{2}, 'Faces', faces{2}, 'FaceColor', 'green', ...
      'Parent', h_link2, 'FaceAlpha', 0.8);

h_link3 = hgtransform('Parent', h_link2); 
patch('Vertices', vertices{3}, 'Faces', faces{3}, 'FaceColor', 'green', ...
      'Parent', h_link3, 'FaceAlpha', 0.8);

h_link4 = hgtransform('Parent', h_link3); 
patch('Vertices', vertices{4}, 'Faces', faces{4}, 'FaceColor', 'yellow', ...
      'Parent', h_link4, 'FaceAlpha', 0.8);

h_link5 = hgtransform('Parent', h_link4); 
patch('Vertices', vertices{5}, 'Faces', faces{5}, 'FaceColor', 'yellow', ...
      'Parent', h_link5, 'FaceAlpha', 0.8);

M_trans = makehgtform('translate', [0, 0, L(1)]);
M2 = M_trans;
set(h_link2, 'Matrix', M2);

M_trans = makehgtform('translate', [0, L(2), -0.02]);
M_rotate = makehgtform('xrotate', 0);
M3 = M_trans * M_rotate;
set(h_link3, 'Matrix', M3);

M_trans = makehgtform('translate', [0, 0, 0]);
M_rotate = makehgtform('xrotate', pi);
M4 = M_trans * M_rotate;
set(h_link4, 'Matrix', M4);

M_trans = makehgtform('translate', [0, 0, L(4)]);
M5 = M_trans;
set(h_link5, 'Matrix', M5);

theta1_waypoint = linspace(theta_A(1), theta_B(1), 200);  
theta2_waypoint = linspace(theta_A(2), theta_B(2), 200); 
theta3_waypoint = linspace(theta_A(3), theta_B(3), 200); 
theta4_waypoint = linspace(theta_A(4), theta_B(4), 200); 

for i = 1:200
    t1 = theta1_waypoint(i); 

    M_trans = makehgtform('translate', [0, 0, 0]);
    M_rotate = makehgtform('zrotate', t1);
    M1 = M_trans * M_rotate;
    set(h_link1, 'Matrix', M1);

    M_trans = makehgtform('translate', [0, 0, L(1)]);
    M_rotate = makehgtform('zrotate', t2);
    M2 = M_trans * M_rotate;
    set(h_link2, 'Matrix', M2);

    M_trans = makehgtform('translate', [0, 0, 0]);
    M_rotate = makehgtform('Xrotate', t3);    
    M4 = M_trans * M_rotate;
    set(h_link4, 'Matrix', M4);

    M_trans = makehgtform('translate', [0, 0, L(4)]);
    M_rotate = makehgtform('xrotate', t4);    
    M5 = M_trans * M_rotate;
    set(h_link5, 'Matrix', M5);

    drawnow;
    pause(0.01);
end

