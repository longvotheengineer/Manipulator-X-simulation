%==========================================================================
%   SCRIPT: main.m
%==========================================================================
%   DESCRIPTION:
%       Initializes the kinematic parameters (link lengths) for a custom
%       4-DOF robot manipulator. The script generates a scatter plot of the 
%       robot's reachable workspace and performs a graphical animation 
%       moving from configuration A to configuration B.
%
%   AUTHOR: [LvDaengineer]
%   DATE:   November 28, 2025
%
%   DEPENDENCIES:
%       - WorkSpace.m     (Generates workspace scatter plot)
%       - AnimateModel.m  (Handles visualization and motion)
%
%   INPUTS:
%       - robot_length: Struct containing L1 (0.077) to L5 (0.126)
%       - theta_A: Initial joint angles [rad]
%       - theta_B: Target joint angles [rad]
%
%==========================================================================

%% 1. INITIALIZATION
%==========================================================================
clear; clc; close all;

%% 2. PARAMETER SETUP & EXECUTION
%==========================================================================
% Define Link Lengths
robot_config.robot_length = struct('L1',0.077, ...
                                   'L2',0.128, ...
                                   'L3',0.024, ...
                                   'L4',0.124, ...
                                   'L5',0.126);

% Generate Workspace
WorkSpace_Scatter = WorkSpace(robot_config);

% Define Joint Configurations (Start & End)
theta_A = [0,    0, 0, 0];
theta_B = [0, pi/2, 0, 0];

% Run Animation
AnimateModel(theta_A, theta_B, robot_config);
