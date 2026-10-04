clc, clear, close all;


%% Calculations
% Define the position of each leg relative to X-axis
phi1 = deg2rad(30);
phi2 = deg2rad(150);
phi3 = deg2rad(270);


% Define legs
L1 = [cos(phi1) sin(phi1) 0];
L1 = L1 / norm(L1);

L2 = [cos(phi2) sin(phi2) 0];
L2 = L2 / norm(L2);

L3 = [cos(phi3) sin(phi3) 0];
L3 = L3 / norm(L3);


% Define screw axis around what each leg rotates:
K1 = [cos(phi1 + pi/2) sin(phi1 + pi/2) 0];
K1 = K1 / norm(K1);

K2 = [cos(phi2 + pi/2) sin(phi2 + pi/2) 0];
K2 = K2 / norm(K2);

K3 = [cos(phi3 + pi/2) sin(phi3 + pi/2) 0];
K3 = K3 / norm(K3);


% Define an angle of rotation
theta1 = deg2rad(20);
theta2 = deg2rad(20);
theta3 = deg2rad(20);


% Find the rotation matrices by Rodrigues' formula
% Calculate the rotation matrix using Rodrigues' formula for each leg
R1 = Rodrigues(K1, theta1);
R2 = Rodrigues(K2, theta2);
R3 = Rodrigues(K3, theta3);


% The rotated legs will be:
L1_rotated = R1 * L1';
L2_rotated = R2 * L2';
L3_rotated = R3 * L3';


% Sum of rotated leg vectors
d_z = L1_rotated + L2_rotated + L3_rotated;


%% Visualization
figure; hold on; axis equal; grid on;

% Fixed frame (blue)
quiver3(0,0,0, 1,0,0, 'b','LineWidth', 2);
quiver3(0,0,0, 0,1,0, 'b','LineWidth', 2);
quiver3(0,0,0, 0,0,1, 'b','LineWidth', 2);
text(1,0,0,'X');
text(0,1,0,'Y');
text(0,0,1,'Z');
view(3);

% Screw axes (green)
quiver3(0,0,0, K1(1),K1(2),K1(3), 'g', 'LineWidth', 2);
text(K1(1),K1(2),K1(3), 'K1');
view(3);

quiver3(0,0,0, K2(1),K2(2),K2(3), 'g', 'LineWidth', 2);
text(K2(1),K2(2),K2(3), 'K2');
view(3);

quiver3(0,0,0, K3(1),K3(2),K3(3), 'g', 'LineWidth', 2);
text(K3(1),K3(2),K3(3), 'K3');
view(3);


% Legs L (red)
quiver3(0,0,0, L1(1),L1(2),L1(3), 'r', 'LineWidth', 2);
text(L1(1),L1(2),L1(3), 'L1');
view(3);

quiver3(0,0,0, L2(1),L2(2),L2(3), 'r', 'LineWidth', 2);
text(L2(1),L2(2),L2(3), 'L2');
view(3);

quiver3(0,0,0, L3(1),L3(2),L3(3), 'r', 'LineWidth', 2);
text(L3(1),L3(2),L3(3), 'L3');
view(3);


% Rotated legs (magenta)
quiver3(0,0,0, L1_rotated(1),L1_rotated(2),L1_rotated(3), 'm', 'LineWidth', 2);
text(L1_rotated(1),L1_rotated(2),L1_rotated(3), 'L1_{rotated}');
view(3);

quiver3(0,0,0, L2_rotated(1),L2_rotated(2),L2_rotated(3), 'm', 'LineWidth', 2);
text(L2_rotated(1),L2_rotated(2),L2_rotated(3), 'L2_{rotated}');
view(3);

quiver3(0,0,0, L3_rotated(1),L3_rotated(2),L3_rotated(3), 'm', 'LineWidth', 2);
text(L3_rotated(1),L3_rotated(2),L3_rotated(3), 'L3_{rotated}');
view(3);

P0 = [0;0;0];
P1 = L1_rotated;
P2 = P1 + L2_rotated;
P3 = P2 + L3_rotated;   % = d_z

% First vector
quiver3(P0(1),P0(2),P0(3),...
        L1_rotated(1),L1_rotated(2),L1_rotated(3),...
        'm','LineWidth',2);

% Second vector
quiver3(P1(1),P1(2),P1(3),...
        L2_rotated(1),L2_rotated(2),L2_rotated(3),...
        'k:','LineWidth',2);

% Third vector
quiver3(P2(1),P2(2),P2(3),...
        L3_rotated(1),L3_rotated(2),L3_rotated(3),...
        'k:','LineWidth',2);

% Resultant
quiver3(0,0,0,...
        P3(1),P3(2),P3(3),...
        'k','LineWidth',3);

text(P3(1),P3(2),P3(3),'d_z');


%% Helper functions
% Rodrigues's formula
function R = Rodrigues(K, theta)

    % Elements
    R11 = (K(1)^2 - 1)*(1-cos(theta)) + 1;
    R12 = K(1)*K(2)*(1-cos(theta)) - K(3)*sin(theta);
    R13 = K(1)*K(3)*(1-cos(theta)) + K(2)*sin(theta);
    R21 = K(2)*K(1)*(1-cos(theta)) + K(3)*sin(theta);
    R22 = (K(2)^2 - 1)*(1-cos(theta)) + 1;
    R23 = K(2)*K(3)*(1-cos(theta)) - K(1)*sin(theta);
    R31 = K(3)*K(1)*(1-cos(theta)) - K(2)*sin(theta);
    R32 = K(3)*K(2)*(1-cos(theta)) + K(1)*sin(theta);
    R33 = (K(3)^2 - 1)*(1-cos(theta)) + 1;

    % Versors that define the orientation of axes
    X = [R11;
         R21;
         R31];

    Y = [R12;
         R22;
         R32];

    Z = [R13;
         R23;
         R33];

    % The rotation matrix is
    R = [X Y Z];
end;