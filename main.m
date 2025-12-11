clear all; clc; close all;

robot_config = struct();
robot_config.robot_length = struct('L1', 0.077,...
                                   'L2', 0.128,...
                                   'L3', 0.024,...
                                   'L4', 0.124,...
                                   'L5', 0.126);

% figure; grid on; axis equal;
% axis([-0.1 0.6 -0.1 0.6 -0.3 0.3]);
% view(45, 30);

WorkSpace_Scatter = WorkSpace(robot_config);

theta_A = [0, 0, 0, 0];
theta_B = [0, pi/2, 0, 0];

AnimateModel(theta_A, theta_B, robot_config);
