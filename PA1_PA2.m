%  Robot: Franka Emika Panda (7-DOF)
%  Tool:  Cylindrical, 100 mm length, 5 mm diameter

%   A - Move tool tip to goal, stay within 3 mm sphere, obey joint limits
%   B - Part A + minimize change in tool shaft direction
%   C - Parts A & B with virtual wall repulsion (bonus)
%   D - Comparison plots across all parts
clear; clc; close all;
addpath("Functions")

%%  ROBOT PARAMETERS — Franka Emika Panda

% DH convention: [a, d, alpha, theta_offset]
q_min = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
q_max = [ 2.8973;  1.7628;  2.8973; -0.0698;  2.8973;  3.7525;  2.8973];

%         a        d       alpha     theta_offset
DH = [  0.000,  0.333,   0,       0;
        0.000,  0.000,  -pi/2,    0;
        0.000,  0.316,   pi/2,    0;
        0.0825, 0.000,   pi/2,    0;
       -0.0825, 0.384,  -pi/2,    0;
        0.000,  0.000,   pi/2,    0;
        0.088,  0.000,   pi/2,    0];

d_flange = 0.107;   % flange offset [m]
d_tool   = 0.100;   % tool length   [m]

%% INIT CONFIGURATION & GOAL

q0 = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];
q0 = clamp(q0, q_min, q_max);

p_goal = [0.45; 0.20; 0.50];

[T0, ~] = forward_kinematics(q0, DH, d_flange, d_tool);
p_start = T0(1:3,4);   % [0.223, 0.399, 0.249]

%% VIRTUAL WALL PARAMETERS (Part C)
%
%  GEOMETRY RATIONALE:
%  Start Y = 0.399,  Goal Y = 0.200
%
%  Wall: XZ-plane patch, normal n_wall = [0,1,0] (repels in +Y).
%  Centre at Y = 0.300 (midpoint between start and goal in Y).
%  Patch: X in [0.270, 0.430], Z in [0.295, 0.455].
%
%  The +Y repulsion opposes the natural -Y goal-seeking motion, forcing
%  the tip to arc laterally (in X) around the patch boundary.
%  The goal at X=0.450 lies 100 mm outside the patch X-extent (centre
%  0.350 ± 0.080), so the tip always converges after clearing the patch.

p_wall  = [0.350; 0.300; 0.375];   % wall centre on straight-line midpoint
n_wall  = [0; 1; 0];               % normal: repels in +Y
wall_dx = 0.08;                    % ±80 mm in X
wall_dz = 0.08;                    % ±80 mm in Z

alpha_gain = 0.06;   % small step prevents jumping over influence zone
k_wall     = 6.0;    % strong repulsion for visible detour
d_thresh   = 0.12;   % 120 mm influence half-width in Y
k_shaft    = 0.80;   

fprintf('Initial tip position:  [%.3f, %.3f, %.3f] m\n', p_start);
fprintf('Goal  position:        [%.3f, %.3f, %.3f] m\n', p_goal);
fprintf('Start distance to goal: %.1f mm\n', norm(p_start - p_goal)*1000);
fprintf('Wall centre: [%.3f, %.3f, %.3f], normal: [%.0f,%.0f,%.0f]\n\n', ...
        p_wall, n_wall);

%% RUN CONTROLLERS

n_steps = 700;   % 700 iterations

fprintf('Running Part A (goal + joint limits)...\n');
traj_a = run_ctrl(q0, p_goal, DH, d_flange, d_tool, q_min, q_max, n_steps, ...
                  false, false, p_wall, n_wall, wall_dx, wall_dz, ...
                  alpha_gain, k_wall, d_thresh, k_shaft);

fprintf('Running Part B (+ shaft orientation)...\n');
traj_b = run_ctrl(q0, p_goal, DH, d_flange, d_tool, q_min, q_max, n_steps, ...
                  true, false, p_wall, n_wall, wall_dx, wall_dz, ...
                  alpha_gain, k_wall, d_thresh, k_shaft);

fprintf('Running Part C (+ virtual wall)...\n');
traj_c = run_ctrl(q0, p_goal, DH, d_flange, d_tool, q_min, q_max, n_steps, ...
                  true, true, p_wall, n_wall, wall_dx, wall_dz, ...
                  alpha_gain, k_wall, d_thresh, k_shaft);

fprintf('Final distances:  A=%.2f mm  B=%.2f mm  C=%.2f mm\n', ...
    traj_a.dist(end), traj_b.dist(end), traj_c.dist(end));
fprintf('Final shaft errors: A=%.1f deg  B=%.2f deg  C=%.2f deg\n\n', ...
    traj_a.shaft_err(end), traj_b.shaft_err(end), traj_c.shaft_err(end));

%% PLOTS

plot_results(traj_a, traj_b, traj_c, p_goal, p_start, ...
             p_wall, n_wall, wall_dx, wall_dz, d_thresh, n_steps);

% ANIMATION 

animate_robot(traj_c, p_goal, p_wall, n_wall, wall_dx, wall_dz);


%%  CONTROLLER FUNCTION

function traj = run_ctrl(q0, p_goal, DH, d_flange, d_tool, ...
                          q_min, q_max, n_steps, use_shaft, use_wall, ...
                          p_wall, n_wall, wall_dx, wall_dz, ...
                          alpha_gain, k_wall, d_thresh, k_shaft)

%  Gradient-based constrained IK controller.
%  Wall: finite XZ-plane patch, normal [0,1,0].
%  Repulsion fires when:
%    |p_x - p_wall_x| <= wall_dx   (inside patch in X)
%    |p_z - p_wall_z| <= wall_dz   (inside patch in Z)
%    |p_y - p_wall_y| < d_thresh   (within influence zone in Y)

    q      = q0;
    shaft0 = extract_shaft(q0, DH, d_flange, d_tool);

    traj.pos       = zeros(3, n_steps);
    traj.dist      = zeros(1, n_steps);
    traj.shaft_err = zeros(1, n_steps);
    traj.q         = zeros(7, n_steps);
    traj.wall_dist = zeros(1, n_steps);   % signed Y-distance to wall [mm]

    alpha  = alpha_gain;
    lambda = 1e-4;
    k_proj = 0.50;

    for k = 1:n_steps
        %% FK
        [T, ~] = forward_kinematics(q, DH, d_flange, d_tool);
        p      = T(1:3,4);
        shaft  = T(1:3,3);

        %% Primary: DLS IK toward goal
        e  = p_goal - p;
        Jp = position_jacobian(q, DH, d_flange, d_tool);
        dq = Jp' * ((Jp*Jp' + lambda*eye(3)) \ e);

        %% Secondary A: shaft orientation (Parts B, C)
        if use_shaft
            Jw      = geometric_jacobian(q, DH, d_flange, d_tool);
            Jw      = Jw(4:6, :);
            e_shaft = cross(shaft, shaft0);
            dq      = dq + k_shaft * Jw' * ((Jw*Jw' + lambda*eye(3)) \ e_shaft);
        end

        %% Secondary B: XZ-patch wall repulsion in +Y (Part C)
        if use_wall
            dist_wall = dot(n_wall, p - p_wall);   % signed Y-distance
            dp        = p - p_wall;
            in_patch  = (abs(dp(1)) <= wall_dx) && (abs(dp(3)) <= wall_dz);

            if in_patch && abs(dist_wall) < d_thresh
                penetration = d_thresh - abs(dist_wall);
                v_rep   = penetration * n_wall;   % push in +Y
                dq_wall = Jp' * ((Jp*Jp' + lambda*eye(3)) \ v_rep);
                dq      = dq + k_wall * dq_wall;
            end
        end

        %% integrate & clamp
        q = q + alpha * dq;
        q = clamp(q, q_min, q_max);

        %% 3 mm sphere constraint
        [T_new, ~] = forward_kinematics(q, DH, d_flange, d_tool);
        p_new = T_new(1:3,4);
        if norm(p_new - p_goal) > 0.003
            p_proj  = project_to_sphere(p_new, p_goal, 0.003);
            e_proj  = p_proj - p_new;
            Jp_new  = position_jacobian(q, DH, d_flange, d_tool);
            dq_proj = Jp_new' * ((Jp_new*Jp_new' + lambda*eye(3)) \ e_proj);
            q       = q + k_proj * dq_proj;
            q       = clamp(q, q_min, q_max);
        end

        %% Log
        [T_log, ~]        = forward_kinematics(q, DH, d_flange, d_tool);
        shaft_cur         = T_log(1:3,3);
        traj.pos(:,k)     = T_log(1:3,4);
        traj.dist(k)      = norm(T_log(1:3,4) - p_goal) * 1000;
        traj.shaft_err(k) = acosd(min(1, max(-1, dot(shaft_cur, shaft0))));
        traj.q(:,k)       = q;
        traj.wall_dist(k) = dot(n_wall, T_log(1:3,4) - p_wall) * 1000;
    end
end


%% UTIL

function q = clamp(q, qmin, qmax)
    q = max(qmin, min(qmax, q));
end

function p_proj = project_to_sphere(p, center, r)
    dir = p - center;
    d   = norm(dir);
    if d < 1e-9; p_proj = center + r*[1;0;0]; return; end
    p_proj = center + r*(dir/d);
end

function d = extract_shaft(q, DH, d_flange, d_tool)
    [T, ~] = forward_kinematics(q, DH, d_flange, d_tool);
    d = T(1:3,3);
end


%% PLOTS

function plot_results(a, b, c, p_goal, p_start, p_wall, n_wall, ...
                      wall_dx, wall_dz, d_thresh, n_steps)

    colors = [0.122 0.471 0.706;   % Part A — blue
              0.200 0.627 0.173;   % Part B — green
              0.890 0.102 0.110];  % Part C — red
    lw    = 2.0;
    iters = 1:n_steps;

    %% Fig1: Distance to Goal & Shaft Deviation
    figure('Color','w','Name','Performance Metrics','Position',[50 50 1300 520]);

    subplot(1,2,1)
    plot(iters, a.dist, '-',  'Color',colors(1,:), 'LineWidth',lw); hold on;
    plot(iters, b.dist, '--', 'Color',colors(2,:), 'LineWidth',lw);
    plot(iters, c.dist, ':',  'Color',colors(3,:), 'LineWidth',lw);
    yline(3,'k--','3 mm limit','LineWidth',1.5,'LabelHorizontalAlignment','left');
    legend('Part A','Part B','Part C','Location','northeast');
    xlabel('Iteration','FontSize',12);
    ylabel('Distance to Goal (mm)','FontSize',12);
    title('Distance to Goal vs. Iteration','FontSize',13,'FontWeight','bold');
    grid on; box on;
    xlim([1 n_steps]);

    subplot(1,2,2)
    % Dual y-axis: Part A on left (large scale), B & C on right (zoomed)
    yyaxis left
    plot(iters, a.shaft_err, '-', 'Color',colors(1,:), 'LineWidth',lw);
    ylabel('Part A Shaft Error (deg)','FontSize',11,'Color',colors(1,:));
    ylim([0 55]);
    ax = gca; ax.YColor = colors(1,:);

    yyaxis right
    plot(iters, b.shaft_err, '--', 'Color',colors(2,:), 'LineWidth',lw); hold on;
    plot(iters, c.shaft_err, ':',  'Color',colors(3,:), 'LineWidth',lw);
    ylabel('Parts B & C Shaft Error (deg)','FontSize',11,'Color',[0.15 0.15 0.15]);
    ylim([0 10]);
    ax.YColor = [0.15 0.15 0.15];

    legend({'Part A (no shaft ctrl)','Part B','Part C (shaft+wall)'}, ...
           'Location','northeast','FontSize',10);
    xlabel('Iteration','FontSize',12);
    title('Tool Shaft Orientation Deviation','FontSize',13,'FontWeight','bold');
    grid on; box on;
    xlim([1 n_steps]);

    sgtitle('Controller Performance Comparison', ...
            'FontSize',14,'FontWeight','bold');
    saveas(gcf,'Fig1_Performance_Metrics.png');

    %% Fie 2: 3D Trajectories
    figure('Color','w','Name','3D Trajectories','Position',[50 600 880 660]);
    hold on; grid on; box on;

    h1 = plot3(a.pos(1,:), a.pos(2,:), a.pos(3,:), '-',  'Color',colors(1,:), 'LineWidth',2.5);
    h2 = plot3(b.pos(1,:), b.pos(2,:), b.pos(3,:), '--', 'Color',colors(2,:), 'LineWidth',2.5);
    h3 = plot3(c.pos(1,:), c.pos(2,:), c.pos(3,:), ':',  'Color',colors(3,:), 'LineWidth',3.0);

    % True start marker
    h4 = plot3(p_start(1), p_start(2), p_start(3), ...
               'ks','MarkerSize',12,'MarkerFaceColor','k');

    % Goal sphere
    [xs,ys,zs] = sphere(30);
    surf(p_goal(1)+0.010*xs, p_goal(2)+0.010*ys, p_goal(3)+0.010*zs, ...
         'FaceAlpha',0.40,'EdgeColor','none','FaceColor',[0 0.75 0]);
    h5 = plot3(p_goal(1),p_goal(2),p_goal(3),'g*','MarkerSize',14,'LineWidth',2);

    % Virtual wall — XZ-plane patch at y = p_wall(2)
    [Xw,Zw] = meshgrid(linspace(p_wall(1)-wall_dx, p_wall(1)+wall_dx, 12), ...
                        linspace(p_wall(3)-wall_dz, p_wall(3)+wall_dz, 12));
    Yw = p_wall(2) * ones(size(Xw));
    surf(Xw, Yw, Zw, 'FaceAlpha',0.45, 'EdgeColor',[0.7 0.1 0.1], ...
         'EdgeAlpha',0.5, 'FaceColor',[1 0.4 0.4]);

    % Normal arrow (+Y direction)
    quiver3(p_wall(1), p_wall(2), p_wall(3), ...
            n_wall(1)*0.06, n_wall(2)*0.06, n_wall(3)*0.06, ...
            0, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 2);
    text(p_wall(1), p_wall(2)+0.08, p_wall(3)+0.01, '\bf n_{wall}', ...
         'Color','r','FontSize',11);

    legend([h1 h2 h3 h4 h5],'Part A','Part B','Part C','Start','Goal', ...
           'Location','best','FontSize',10);
    xlabel('X (m)','FontSize',12);
    ylabel('Y (m)','FontSize',12);
    zlabel('Z (m)','FontSize',12);
    title({'3D Tool-Tip Trajectories', ...
           'Part C deflects around wall; A & B cross freely'}, ...
           'FontSize',12,'FontWeight','bold');

    % Axis bounds including true start
    all_x = [a.pos(1,:),b.pos(1,:),c.pos(1,:),p_goal(1),p_wall(1),p_start(1)];
    all_y = [a.pos(2,:),b.pos(2,:),c.pos(2,:),p_goal(2),p_wall(2),p_start(2)];
    all_z = [a.pos(3,:),b.pos(3,:),c.pos(3,:),p_goal(3),p_wall(3),p_start(3)];
    pad = 0.06;
    xlim([min(all_x)-pad, max(all_x)+pad]);
    ylim([min(all_y)-pad, max(all_y)+pad]);
    zlim([min(all_z)-pad, max(all_z)+pad]);
    view(30, 20);   % angle chosen to show lateral detour and start marker clearly
    saveas(gcf,'Fig2_3D_Trajectories.png');

    %% Fie 3: Signed Wall Distance (Y-direction)
    % The wall repulsion fires while the tip is inside the patch AND
    % within d_thresh of Y=0.300.
    d_thresh_mm = d_thresh * 1000;

    figure('Color','w','Name','Wall Proximity','Position',[780 600 800 500]);
    hold on; grid on; box on;

    plot(iters, a.wall_dist, '-',  'Color',colors(1,:), 'LineWidth',lw);
    plot(iters, b.wall_dist, '--', 'Color',colors(2,:), 'LineWidth',lw);
    plot(iters, c.wall_dist, ':',  'Color',colors(3,:), 'LineWidth',lw+0.5);

    % Influence zone boundaries
    yline( d_thresh_mm, 'k--', 'LineWidth',1.5);
    yline(-d_thresh_mm, 'k--', 'LineWidth',1.5);
    yline(0, 'r-', 'LineWidth',1.5);

    % Labels — placed away from curves
    text(n_steps*0.55,  d_thresh_mm+7, sprintf('+%d mm influence boundary', d_thresh_mm), ...
         'FontSize',10,'Color','k','HorizontalAlignment','center');
    text(n_steps*0.55, -d_thresh_mm-13, sprintf('-%d mm influence boundary', d_thresh_mm), ...
         'FontSize',10,'Color','k','HorizontalAlignment','center');
    text(10, 8, 'Wall surface (Y = 0.30 m)','FontSize',10,'Color','r');

    % Shade influence zone
    xf = [1 n_steps n_steps 1];
    yf = [-d_thresh_mm -d_thresh_mm d_thresh_mm d_thresh_mm];
    fill(xf, yf, [1 0.85 0.85], 'FaceAlpha',0.20, 'EdgeColor','none');
    text(n_steps*0.70, d_thresh_mm*0.5, 'Wall influence zone', ...
         'FontSize',10,'Color',[0.7 0.2 0.2],'HorizontalAlignment','center');

    legend('Part A (no wall)','Part B (no wall)','Part C (wall active)', ...
           'Location','southeast','FontSize',10);
    xlabel('Iteration','FontSize',12);
    ylabel('Signed Distance to Wall in Y (mm)','FontSize',12);
    title({'Tool-Tip Signed Distance from Virtual Wall (Y-direction)', ...
           'Part C repelled in +Y during crossing — exits patch laterally in X'}, ...
           'FontSize',12,'FontWeight','bold');
    xlim([1 n_steps]);

    ylim([-d_thresh_mm-30, d_thresh_mm+30]);

    ax_inset = axes('Position',[0.55 0.55 0.32 0.30]);
    hold(ax_inset,'on'); grid(ax_inset,'on'); box(ax_inset,'on');
    zoom_end = min(80, n_steps);
    plot(ax_inset, 1:zoom_end, a.wall_dist(1:zoom_end), '-',  'Color',colors(1,:), 'LineWidth',1.5);
    plot(ax_inset, 1:zoom_end, b.wall_dist(1:zoom_end), '--', 'Color',colors(2,:), 'LineWidth',1.5);
    plot(ax_inset, 1:zoom_end, c.wall_dist(1:zoom_end), ':',  'Color',colors(3,:), 'LineWidth',2.0);
    yline(ax_inset, 0,  'r-', 'LineWidth',1.2);
    yline(ax_inset,  d_thresh_mm, 'k--', 'LineWidth',1.0);
    yline(ax_inset, -d_thresh_mm, 'k--', 'LineWidth',1.0);
    fill(ax_inset, [1 zoom_end zoom_end 1], ...
         [-d_thresh_mm -d_thresh_mm d_thresh_mm d_thresh_mm], ...
         [1 0.85 0.85],'FaceAlpha',0.25,'EdgeColor','none');
    title(ax_inset, 'Zoom: iters 1–80','FontSize',9,'FontWeight','bold');
    xlabel(ax_inset,'Iter','FontSize',9);
    ylabel(ax_inset,'mm','FontSize',9);
    xlim(ax_inset,[1 zoom_end]);
    ylim(ax_inset,[-d_thresh_mm-20, d_thresh_mm+20]);
    set(ax_inset,'FontSize',8);

    saveas(gcf,'Fig3_Wall_Distance.png');

    %% igure 4: Joint Angle Histories (Part C)
    figure('Color','w','Name','Joint Angles Part C','Position',[50 100 1200 500]);
    joint_names = {'q_1','q_2','q_3','q_4','q_5','q_6','q_7'};
    q_min_r = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
    q_max_r = [ 2.8973; 1.7628; 2.8973;-0.0698; 2.8973; 3.7525; 2.8973];

    for j = 1:7
        subplot(2,4,j);
        plot(iters, rad2deg(c.q(j,:)), 'Color',colors(3,:), 'LineWidth',1.5); hold on;
        yline(rad2deg(q_min_r(j)), 'r--', 'LineWidth',1);
        yline(rad2deg(q_max_r(j)), 'r--', 'LineWidth',1);
        xlabel('Iter'); ylabel('deg');
        title(joint_names{j},'FontWeight','bold');
        grid on; box on;
        xlim([1 n_steps]);   % explicit xlim — fixes MATLAB autoscale artifact
        ylim([rad2deg(q_min_r(j))-5, rad2deg(q_max_r(j))+5]);
    end
    sgtitle('Part C — Joint Angle Histories with Limits (red dashed)', ...
            'FontSize',13,'FontWeight','bold');
    saveas(gcf,'Fig4_Joint_Angles.png');

    fprintf('\nAll figures saved as PNG files.\n');
end


%%  ANIMATION — Tool-Tip Trajectory (Part C)

function animate_robot(traj, p_goal, p_wall, n_wall, wall_dx, wall_dz)

    figure('Color','w','Name','Animation — Part C','Position',[800 50 700 600]);
    hold on; grid on; box on;

    [xs,ys,zs] = sphere(30);
    n_frames   = length(traj.dist);

    % XZ-plane patch at y = p_wall(2)
    [Xw,Zw] = meshgrid(linspace(p_wall(1)-wall_dx, p_wall(1)+wall_dx, 8), ...
                        linspace(p_wall(3)-wall_dz, p_wall(3)+wall_dz, 8));
    Yw = p_wall(2) * ones(size(Xw));

    for k = 1:n_frames
        cla;
        plot3(traj.pos(1,1:k), traj.pos(2,1:k), traj.pos(3,1:k), ...
              'b-','LineWidth',2);
        surf(p_goal(1)+0.003*xs, p_goal(2)+0.003*ys, p_goal(3)+0.003*zs, ...
             'FaceAlpha',0.25,'EdgeColor','none','FaceColor',[0 0.8 0]);
        surf(Xw,Yw,Zw,'FaceAlpha',0.30,'EdgeColor',[0.6 0.2 0.2], ...
             'EdgeAlpha',0.4,'FaceColor','r');
        plot3(traj.pos(1,k), traj.pos(2,k), traj.pos(3,k), ...
              'ro','MarkerFaceColor','r','MarkerSize',9);
        text(0.05,0.95,sprintf('Iter %d/%d   Dist=%.2f mm',k,n_frames,traj.dist(k)), ...
             'Units','normalized','FontSize',11,'FontWeight','bold', ...
             'BackgroundColor','w','EdgeColor','k');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('Part C — Tool-Tip Trajectory (shaft + virtual wall)', ...
              'FontSize',12,'FontWeight','bold');
        view(30,20);
        axis([0.15 0.60 0.10 0.50 0.20 0.60]);
        drawnow;
    end
end