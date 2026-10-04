clc;
clear;
close all;

%% Geometry
% Position of each leg relative to X-axis
phi = deg2rad([30 150 270]);

% Rotation angles (change these)
theta = deg2rad([10 40 0]);      % Example:
                                 % [20 20 20] -> sum of 3 vectors
                                 % [20 20 0]  -> sum of 2 vectors
                                 % [20 0 0]   -> sum of 1 vector
                                 % [0 0 0]    -> no sum

tol = 1e-10;

%% Define legs and screw axes
L = zeros(3,3);
K = zeros(3,3);

for i = 1:3

    % Leg direction
    L(i,:) = [cos(phi(i)) sin(phi(i)) 0];
    L(i,:) = L(i,:)/norm(L(i,:));

    % Screw axis (90 degrees from leg)
    K(i,:) = [cos(phi(i)+pi/2) sin(phi(i)+pi/2) 0];
    K(i,:) = K(i,:)/norm(K(i,:));

end

%% Rotate active legs and compute resultant vector

L_rotated = zeros(3,3);
d_z = zeros(3,1);

active = abs(theta) > tol;

for i = find(active)

    R = Rodrigues(K(i,:), theta(i));

    L_rotated(:,i) = R * L(i,:)';

    d_z = d_z + L_rotated(:,i);

end

%% Visualization

figure;
hold on;
grid on;
axis equal;
view(3);

xlabel('X');
ylabel('Y');
zlabel('Z');

%% Fixed frame

quiver3(0,0,0,1,0,0,'b','LineWidth',2);
quiver3(0,0,0,0,1,0,'b','LineWidth',2);
quiver3(0,0,0,0,0,1,'b','LineWidth',2);

text(1,0,0,'X');
text(0,1,0,'Y');
text(0,0,1,'Z');

%% Original legs (red)

for i = 1:3

    quiver3(0,0,0,...
        L(i,1),L(i,2),L(i,3),...
        'r','LineWidth',2);

    text(L(i,1),L(i,2),L(i,3),sprintf('L%d',i));

end

%% Screw axes (green)

for i = 1:3

    quiver3(0,0,0,...
        K(i,1),K(i,2),K(i,3),...
        'g','LineWidth',2);

    text(K(i,1),K(i,2),K(i,3),sprintf('K%d',i));

end

%% Rotated legs (only active ones)

for i = find(active)

    quiver3(0,0,0,...
        L_rotated(1,i),...
        L_rotated(2,i),...
        L_rotated(3,i),...
        'm','LineWidth',2);

    text(L_rotated(1,i),...
         L_rotated(2,i),...
         L_rotated(3,i),...
         sprintf('L%d_{rot}',i));

end

%% Head-to-tail vector addition

P = zeros(3,4);

for k = 1:length(find(active))

    idx = find(active);
    i = idx(k);

    P(:,k+1) = P(:,k) + L_rotated(:,i);

    if k == 1
        color = 'm';
    else
        color = 'k:';
    end

    quiver3(P(1,k),P(2,k),P(3,k),...
            L_rotated(1,i),...
            L_rotated(2,i),...
            L_rotated(3,i),...
            color,'LineWidth',2);

end

%% Resultant vector

if any(active)

    quiver3(0,0,0,...
        d_z(1),d_z(2),d_z(3),...
        'k','LineWidth',3);

    text(d_z(1),d_z(2),d_z(3),'d_z');

else

    disp('All angles are zero.');
    disp('No resultant vector.');

end

%% Rodrigues' rotation formula
function R = Rodrigues(K,theta)

K = K(:)/norm(K);

Kx = [    0   -K(3)  K(2);
        K(3)     0  -K(1);
       -K(2)  K(1)     0];

R = eye(3) + sin(theta)*Kx + (1-cos(theta))*Kx^2;

end