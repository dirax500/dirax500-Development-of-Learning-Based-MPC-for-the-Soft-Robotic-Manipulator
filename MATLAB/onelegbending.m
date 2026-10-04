clc; clear; close all;

% Define input parameters (bending angle of legs: MUST BE ONLY ONE LEG)
theta1 = deg2rad(20);
theta2 = 0;
theta3 = 0;

%% Inverse kinematics
% Compute the bending angle and deflection angle
if theta1 ~= 0
    alpha = theta1;
    phi = deg2rad(30);
elseif theta2 ~= 0
    alpha = theta2;
    phi = deg2rad(150);
elseif theta3 ~= 0
    alpha = theta3;
    phi = deg2rad(270);
end



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