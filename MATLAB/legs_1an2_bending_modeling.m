clc, clear, close all;

% Define a screw axis of the rotation
K = [-1 sqrt(3) 0];
K = K / norm(K);

% Define a Leg2 direction (L) which will rotate (leg bends);
L = [sqrt(3) 1 0];
L = L / norm(L);


% Define an angle of rotation
theta = deg2rad(45);

% Elements of a rotation matrix by Rodrigues's formula
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

% The rotated L will be:
L_rotated = R * L';

% Check if the rotation matrix is orthonormal
disp(['Determinant of the rotation matrix is: ' num2str(det(R))]);
disp(['Magnitudes of the versor X of rotation matrix is: ' num2str(norm(X))]);
disp(['Magnitudes of the versor Y of rotation matrix is: ' num2str(norm(Y))]);
disp(['Magnitudes of the versor Z of rotation matrix is: ' num2str(norm(Z))]);
disp(['Dot product of versors X and Y is:' num2str(dot(X, Y))]);
XdotZ = round(dot(X,Z), 12);
YdotZ = round(dot(Y,Z), 12);
disp(['Dot product of versors X and Z is: ' num2str(XdotZ)]);
disp(['Dot product of versors Y and Z is: ' num2str(YdotZ)]);
% For orthonormal matrix determinant must equal 1, magnitues of versors
% must equal 1 and their dot products must equal 0.
% The round function was used as the in some cases, the program outputs
% very small number instead.

X2 = R(:,1);
Y2 = R(:,2);

% Projection of L1_rotated onto X1Z1 plane
Pxz = [1 0 0;
       0 0 0;
       0 0 1];

LX_proj = Pxz * L_rotated;

% Projection of L1_rotated onto Y1Z1 plane
Pyz = [0 0 0;
       0 1 0;
       0 0 1];

LY_proj = Pyz * L_rotated;



figure; hold on; axis equal; grid on;

% Fixed frame (blue)
quiver3(0,0,0, 1,0,0, 'b','LineWidth', 2);
quiver3(0,0,0, 0,1,0, 'b','LineWidth', 2);
quiver3(0,0,0, 0,0,1, 'b','LineWidth', 2);
text(1,0,0,'X1');
text(0,1,0,'Y1');
text(0,0,1,'Z1');
view(3);

% Screw axis K (green)
quiver3(0,0,0, K(1),K(2),K(3), 'g', 'LineWidth', 2);
text(K(1),K(2),K(3), 'K');
view(3);

% Leg frame L (orange)
quiver3(0,0,0, L(1),L(2),L(3), 'r', 'LineWidth', 2);
text(L(1),L(2),L(3), 'L1');
view(3);

% Rotated leg frame L (magenta)
quiver3(0,0,0, L_rotated(1),L_rotated(2),L_rotated(3), 'm', 'LineWidth', 2);
text(L_rotated(1),L_rotated(2),L_rotated(3), 'L1_{rotated}');
view(3);

%Rotated frame (green)
% quiver3(0,0,0, R(1,1), R(2,1), R(3,1), 'g', 'LineWidth', 2);
% quiver3(0,0,0, R(1,2), R(2,2), R(3,2), 'g', 'LineWidth', 2);
% quiver3(0,0,0, R(1,3), R(2,3), R(3,3), 'g', 'LineWidth', 2);
% text(R(1,1), R(2,1), R(3,1), 'X2');
% text(R(1,2), R(2,2), R(3,2), 'Y2');
% text(R(1,3), R(2,3), R(3,3), 'Z2');
% view(3);

% L_rot projected onto X1Z1 plane
quiver3(0,0,0,...
        LX_proj(1),LX_proj(2),LX_proj(3),...
        'k--','LineWidth',2);

% L_rot projected onto Y1Z1 plane
quiver3(0,0,0,...
        LY_proj(1),LY_proj(2),LY_proj(3),...
        'm--','LineWidth',2);

% text(X2_proj(1),X2_proj(2),X2_proj(3),'X2_{YZ}');
% text(Y2_proj(1),Y2_proj(2),Y2_proj(3),'Y2_{XZ}');

% L_rot -> X1Z1 plane
plot3([L_rotated(1) LX_proj(1)],...
      [L_rotated(2) LX_proj(2)],...
      [L_rotated(3) LX_proj(3)],...
      'k:','LineWidth',1.5);

% % Y2 -> Y1Z1 plane
plot3([L_rotated(1) LY_proj(1)],...
      [L_rotated(2) LY_proj(2)],...
      [L_rotated(3) LY_proj(3)],...
      'm:','LineWidth',1.5);