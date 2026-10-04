clc; clear; close all;

% Define the position of each leg relative to X-axis
phi1 = deg2rad(30);
phi2 = deg2rad(150);
phi3 = deg2rad(270);

% Define input parameters (desired pose)
theta_x = deg2rad(17.32);
theta_y = deg2rad(10);

%% Inverse kinematics
% Compute the bending angle and deflection angle
alpha = sqrt(theta_x^2 + theta_y^2);
phi = atan2(theta_y, theta_x);

% Bending angles of each leg
theta1 = alpha * cos(phi1 - phi);
theta2 = alpha * cos(phi2 - phi);
theta3 = alpha * cos(phi3 - phi);

% Print results
fprintf('Bending angle of the whole platform: %.2f deg\n', rad2deg(alpha));
fprintf('Deflection angle of the platform: %.2f deg\n', rad2deg(phi));
fprintf('\n');
fprintf('Bending angle of Leg 1: %.2f deg\n', rad2deg(theta1));
fprintf('Bending angle of Leg 2: %.2f deg\n', rad2deg(theta2));
fprintf('Bending angle of Leg 3: %.2f deg\n', rad2deg(theta3));
fprintf('\n');

%% Forward kinametics
% Check the calculations by forward kinematics
FK_theta_x = alpha * cos(phi);
FK_theta_y = alpha * sin(phi);

fprintf('Theta_x from FK: %.2f deg\n', rad2deg(FK_theta_x));
fprintf('Theta_y from FK: %.2f deg\n', rad2deg(FK_theta_y));