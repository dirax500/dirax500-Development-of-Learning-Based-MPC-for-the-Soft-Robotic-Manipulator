%% soft_polygon_3D_elastic_offsetLoad_3points.m
% 3-sided soft platform with elastic joints (quasi-static model)


%     0°, 120°, 240° around the centre (radius 10 mm)


clear; close all; clc;

%% ----- Geometry -----
L1 = 100;      % bottom link length [mm]
L2 = 100;      % top link length [mm]

% Triangle radius: distance from center to each side anchor
R  = 60;       % [mm]  (same for bottom and desired for top)

% Desired rest "knee" inside angle ~130 deg
inside_angle_deg = 130;
theta0_deg       = (180 - inside_angle_deg)/2;   % symmetric bending each side
theta0           = deg2rad(theta0_deg);          % [rad]

% Rest joint angles (where springs have zero torque)
% th1 = qb, th2 = qb + qm
% rest: th1 = +theta0, th2 = -theta0  (S-shape)
qb_rest = +theta0;
qm_rest = -2*theta0;               % so qb_rest + qm_rest = -theta0
qt_rest = 0.0;                     % top joint neutral for now
restQ   = [qb_rest, qm_rest, qt_rest];

% For info: approximate rest side height from link geometry
th1_rest = qb_rest;
th2_rest = qb_rest + qm_rest;
z_rest = L1*cos(th1_rest) + L2*cos(th2_rest);   % [mm]
fprintf('Approx rest side height from links = %.1f mm\n', z_rest);

%% ----- Joint stiffness values (Nm/rad) -----
% These are the calibrated values you found to match 1 kg -> ~10 mm sag
k_edge = 0.150;     % stiffness of bottom & top joints
k_mid  = 0.750;     % stiffness of middle joint (thicker TPU)

% Penalty enforcing plate radius in XY (units: "energy" per mm^2)
k_plate_xy = 0.1;  % keeps top triangle radius near R

%% ----- External load -----
payload_kg_max = 3.0;       % max payload to simulate [kg]
g = 9.81;                   % [m/s^2]
Fz_total_max = payload_kg_max * g;   % [N] total downward force

nFrames = 40;    % animation frames per load position

%% ----- Base triangle (bottom plate) in XY plane -----
baseVerts = zeros(3,3);
for k = 1:3
    ang = (k-1)*2*pi/3;   % 0, 120, 240 deg
    baseVerts(k,:) = [R*cos(ang), R*sin(ang), 0];
end

triXY = baseVerts(:,1:2);          % use nominal triangle in XY

%% ----- Figure -----
figure('Color','w');
axis equal; grid on; hold on;
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3-sided soft platform – elastic response with off-centre load');
view(45,25);

opts = optimset('Display','off');

z_top_noload = [];  % to store no-load mean top height (first frame only)

%% ----- Three load positions: 0°, 120°, 240° -----
load_radius = 10;                  % 10 mm off-centre
load_angles_deg = [0, 120, 240];   % degrees

for idx = 1:3
    phi_deg = load_angles_deg(idx);
    phi     = deg2rad(phi_deg);

    % Off-centre load point at this angle
    p_load = [load_radius*cos(phi), load_radius*sin(phi)];   % [x, y] mm

    % Barycentric weights of the load point
    lambda = barycentric2D(p_load, triXY);   % 1x3
    fprintf('Angle %3.0f deg: load barycentric weights (side 1,2,3) = [%.3f  %.3f  %.3f]\n', ...
            phi_deg, lambda(1), lambda(2), lambda(3));

    for a = linspace(0,1,nFrames)    % a: 0 → no load, 1 → full payload

        cla;

        % Current payload and per-side force
        payload_kg = a * payload_kg_max;
        Fz_total   = payload_kg * g;           % [N], downward
        Fz_sides   = lambda * Fz_total;        % 1x3, distributed by barycentric weights

        % ---- Bottom plate (fixed triangle) ----
        bottomPlate = baseVerts;
        fill3(bottomPlate(:,1), bottomPlate(:,2), bottomPlate(:,3), ...
              [0.5 0.5 0.5], 'FaceAlpha', 0.3, ...
              'EdgeColor', 'k', 'LineWidth', 1.5);
        hold on;

        topJointPositions = zeros(3,3);

        % ---- For each side ----
        for k = 1:3
            ang = (k-1)*2*pi/3;
            e_r = [cos(ang); sin(ang); 0];   % radial dir in XY
            e_z = [0; 0; 1];                 % vertical

            % bottom anchor for this side
            B = bottomPlate(k,:).';          % 3x1

            % Per-side load
            Fz_side = Fz_sides(k);

            % Solve equilibrium joint angles q = [qb, qm, qt]
            q0 = restQ;                      % initial guess: rest shape

            energy_fun = @(q) side_energy(q, restQ, L1, L2, e_r, e_z, B, ...
                                         k_edge, k_mid, k_edge, Fz_side, ...
                                         k_plate_xy, R);

            q_opt = fminsearch(energy_fun, q0, opts);
            qb = q_opt(1);
            qm = q_opt(2);
            qt = q_opt(3);  % only used in energy for now

            % local link angles
            th1 = qb;
            th2 = qb + qm;

            % local geometry in (u,z)
            p0 = [0; 0];
            p1 = p0 + L1*[sin(th1); cos(th1)];
            p2 = p1 + L2*[sin(th2); cos(th2)];     % top joint pos (local)

            % map to 3D
            P0 = B;
            P1 = B + e_r*p1(1) + e_z*p1(2);
            P2 = B + e_r*p2(1) + e_z*p2(2);

            topJointPositions(k,:) = P2.';  % actual hinge position

            % draw side (links & joints)
            plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'b-', 'LineWidth', 2);
            plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'b-', 'LineWidth', 2);

            plot3(P0(1), P0(2), P0(3), 'ro', 'MarkerSize', 7, 'LineWidth', 1.5); % bottom
            plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 7, 'LineWidth', 1.5); % middle
            plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 7, 'LineWidth', 1.5); % top
        end

        % ---- Top plate: connect actual top joints ----
        topPlate = topJointPositions;
        z_top_mean = mean(topJointPositions(:,3));

        if isempty(z_top_noload)
            z_top_noload = z_top_mean;  % store no-load value (first frame)
        end
        sag = z_top_noload - z_top_mean;        % positive sag = moved downward

        fill3(topPlate(:,1), topPlate(:,2), topPlate(:,3), ...
              [0.3 0.7 1.0], 'FaceAlpha', 0.3, ...
              'EdgeColor', 'k', 'LineWidth', 1.5);

        % Draw the load point (projected at mean height just for visualization)
        plot3(p_load(1), p_load(2), z_top_mean, 'kx', 'MarkerSize', 10, 'LineWidth', 2);

        % --- Info in title ---
        txt = sprintf('Angle = %3.0f deg | Payload = %.2f kg | mean top z = %.1f mm | sag = %.1f mm', ...
                      phi_deg, payload_kg, z_top_mean, sag);
        title({'3-sided soft platform – elastic response with off-centre load', txt});

        axis equal;
        xlim([-200 200]);
        ylim([-200 200]);
        zlim([0 250]);
        view(45,25);
        drawnow;
    end

    % Pause 3 seconds before switching to the next load position
    pause(3);
end

%% ----- Helper: side elastic energy under vertical load + plate constraint -----
function E = side_energy(q, restQ, L1, L2, e_r, e_z, B, ...
                         k_bot, k_mid, k_top, Fz, k_plate_xy, R_des)
    % q     = [qb, qm, qt]
    % restQ = [qb_rest, qm_rest, qt_rest]
    qb  = q(1); qm  = q(2); qt  = q(3);
    qb0 = restQ(1); qm0 = restQ(2); qt0 = restQ(3);

    % local link angles
    th1 = qb;
    th2 = qb + qm;

    % local coordinates
    p1 = L1*[sin(th1); cos(th1)];
    p2 = p1 + L2*[sin(th2); cos(th2)];

    % global top hinge position
    P2 = B + e_r*p2(1) + e_z*p2(2);
    x_top = P2(1);
    y_top = P2(2);
    z_top = P2(3);    % [mm]

    % torsional spring energy about rest angles
    E_spr = 0.5*k_bot*(qb - qb0)^2 + ...
            0.5*k_mid*(qm - qm0)^2 + ...
            0.5*k_top*(qt - qt0)^2;

    % vertical load potential: Fz > 0 downward, we want lower z to reduce energy
    E_load =  Fz * (z_top / 1000);   % convert mm -> m

    % plate constraint: keep radial distance near R_des
    r_xy = sqrt(x_top^2 + y_top^2);
    E_plate = k_plate_xy * (r_xy - R_des)^2;

    E = E_spr + E_load + E_plate;
end

%% ----- Helper: barycentric coordinates of a point in 2D -----
function lam = barycentric2D(p, tri)
% p   : 1x2 [x y]
% tri : 3x2, triangle vertices [x y]
% lam : 1x3 barycentric coordinates, lam(1)+lam(2)+lam(3)=1

x = p(1);  y = p(2);

x1 = tri(1,1); y1 = tri(1,2);
x2 = tri(2,1); y2 = tri(2,2);
x3 = tri(3,1); y3 = tri(3,2);

den = ( (y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3) );

lam1 = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / den;
lam2 = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / den;
lam3 = 1 - lam1 - lam2;

lam = [lam1 lam2 lam3];
end
