clear; close all; clc;
addpath("Functions")

%% Franka Emika Set Up from THA2

% link lengths, m
L = [0.333 0.316 0.384 0.107];

% flange offset, m
a = 0.088;

% home position, https://frankarobotics.github.io/docs/robot_specifications.html#kinematic-configuration
M = [1 0 0 a;
    0 -1 0 0;
    0 0 -1 L(1)+L(2)+L(3)-L(4);
    0 0 0 1];

% Screw axis in space frame

ws = {[0;0;1], [0;-1;0], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1]};
qs = {[0;0;0], [0;0;L(1)], [0;0;L(1)], [a;0;L(1)+L(2)], [0;0;L(1)+L(2)+L(3)], [0;0;L(1)+L(2)+L(3)], [a;0;L(1)+L(2)+L(3)-L(4)]};

S_space = zeros(6,7);

for i=1:7
    wi = ws{i};
    if norm(wi) == 0
        vi = qs{i};
    else
        vi = cross(-wi, qs{i});
    end

    S_space(:, i) = [wi; vi];
end

% Screw axis in body frame

wb = {[0;0;-1], [0;1;0], [0;0;-1], [0;-1;0], [0;0;-1], [0;-1;0], [0;0;-1]};
qb = {[-a;0;-L(4)+L(3)+L(2)], [-a;0;-L(4)+L(3)+L(2)], [-a;0;-L(4)+L(3)+L(2)], [0;0;-L(4)+L(3)], [-a;0;-L(4)], [-a;0;-L(4)], [0;0;0]};

S_body = zeros(6,7);

for i=1:7
    wi = wb{i};
    if norm(wi) == 0
        vi = qb{i};
    else
        vi = cross(-wi, qb{i});
    end

    S_body(:, i) = [wi; vi];
end

% Joint Limits
qL_deg = [-166; -101; -166; -176; -166; -1; -166]; % from product manual, deg
qU_deg = [166; 101; 166; -4; 166; 215; 166];
qL = deg2rad(qL_deg); % convert to radians
qU = deg2rad(qU_deg);

%% Robot Setup for Animation
robot = loadrobot('frankaEmika','DataFormat','column');
robot.Gravity = [0 0 -9.81];

% Remove the hand and fingers
% Remove 'panda_hand' which removes 'panda_leftfinger' and 'panda_rightfinger'
removeBody(robot, 'panda_hand');

%% 100 mm x 5 mm Cylinder Tool
toolL = 0.10;
toolR = 0.0025;
toolBody = rigidBody('tool');
toolJoint = rigidBodyJoint('toolFix','fixed');
setFixedTransform(toolJoint, eye(4));
toolBody.Joint = toolJoint;

addVisual(toolBody, "Cylinder", [toolR toolL], trvec2tform([0 0 toolL/2]));
addCollision(toolBody, collisionCylinder(toolR, toolL));

% Attach directly to link 8 (the last flange)
addBody(robot, toolBody, 'panda_link8');
toolName = 'tool';

%% Configuration
% homeConfiguration(robot) returns a 7x1 vector
q = homeConfiguration(robot);
q_initial = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];

%% Goal & Tool Geometry
% pGoal = [0.10; 0.10; 0.40];
pGoal = [0.30;0.00;0.50];

% tip is at the end of the 100mm cylinder
toolTipOffset = [0; 0; toolL];

%% Sim Params
dt = 0.05;
T  = 15;
N  = round(T/dt);
dMax  = 0.003; % here is the 3 mm circle
lambda = 0.05;  % regularization weight
dq_clamp = 0.08; % max joint dispacement per step
margin = deg2rad(5); % soft joint limit bugfer
k_null = 0.05; % null space center gain


%% goal cases
goals = {
    [0.30;  0.00; 0.50], ...
    [0.20;  0.20; 0.45], ...
    [0.40; -0.10; 0.35]
    };
%% Figure Setup
figure('Color','k','Position',[100 80 1200 700]);

% 1: Robot Simulation with Path and Goal
ax1 = subplot(2,2,1);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
view(ax1,[45 25]);
xlim(ax1,[-0.8 0.8]); ylim(ax1,[-0.8 0.8]); zlim(ax1,[0 1.1]);
title(ax1,'Franka Panda + Tool','Color','w');
set(ax1,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w','ZColor','w');

% Goal Sphere
[sx,sy,sz] = sphere(24);
hGoalSurf = surf(ax1, sx*0.05, sy*0.05, sz*0.05, ...
    'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
traj = plot3(ax1, NaN, NaN, NaN, 'c-',  'LineWidth', 2);
tipP = plot3(ax1, NaN, NaN, NaN, 'ro',  'MarkerFaceColor', 'r');


% 2: Goal Error (error between tool tip and pGoal)
ax2 = subplot(2,2,2);
hold(ax2,'on'); grid(ax2,'on');
set(ax2,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w');
title(ax2,'Goal Error','Color','w');
xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Error (m)','Color','w');
errLine = plot(ax2,NaN,NaN,'c-','LineWidth',2);
plot(ax2,[0 T],[dMax dMax],'g--'); %error threshold 3mm
xlim(ax2,[0 T]); ylim(ax2,[0 0.6]);

% 3: Tool Orientation (for plotting/comparison with PA2)
ax3 = subplot(2,2,3);
hold(ax3,'on'); grid(ax3,'on');
set(ax3, 'Color', [0.08 0.08 0.12], 'XColor', 'w', 'YColor', 'w');
title(ax3, 'Tool Z-Axis Orientation', 'Color', 'w');
xlabel(ax3, 'Time (s)', 'Color', 'w');
ylabel(ax3, 'Component',  'Color', 'w');
oriLines(1) = plot(ax3, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Zx');
oriLines(2) = plot(ax3, NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Zy');
oriLines(3) = plot(ax3, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Zz');
legend(ax3, 'TextColor', 'w', 'Location', 'southeast');
ylim(ax3, [-1.1 1.1]);

% 4: Joint Angles
ax4 = subplot(2,2,4);
hold(ax4, 'on');
set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
title(ax4, 'Live Joint Angles vs. Limits', 'Color', 'w');

% --- STATIONARY SHADOW BARS ---
% Visually showing the joint range and limits

% Calculate the 'height' of the allowable range
jointRangeDeg = rad2deg(qU - qL);
% Position the shadow bars so they start at the Lower Limit
hShadow = bar(ax4, rad2deg(qL) + jointRangeDeg/2, 'FaceColor', [0.3 0.3 0.3], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.2, 'BarWidth', 0.8);
% Adjust the shadow bar to cover the full range from qL to qU
hShadow.YData = jointRangeDeg;
hShadow.BaseValue = min(rad2deg(qL));
% Set the shadow bar property to 'stacked' or simply offset the bottom

for i = 1:7
    % Drawing a grey rectangle for the limit range
    rectangle(ax4, 'Position', [i-0.4, rad2deg(qL(i)), 0.8, jointRangeDeg(i)], ...
        'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
end

% --- ACTIVE BARS ---
% shows the joint limit values as the robot moves
uniformColor = [0 0.7 0.7];
hBar = bar(ax4, rad2deg(q), 'FaceColor', 'flat', 'EdgeColor', 'w', 'BarWidth', 0.5);
hBar.CData = repmat(uniformColor, 7, 1);
hBar.BaseValue = 0;

ylabel(ax4, 'Degrees', 'Color', 'w');
ylim(ax4, [-190 190]);
xticks(ax4, 1:7);
xticklabels(ax4, {'q1','q2','q3','q4','q5','q6','q7'});

% Initialize Text Labels
hTexts = gobjects(7,1);
for i = 1:7
    hTexts(i) = text(ax4, i, 0, '0.0', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'Color', 'w', 'FontWeight', 'bold');
end

%% Storage (for plots)
pHist = zeros(3,N); % position
eHist = zeros(1,N); % error
zHist = zeros(3,N); % Z-axis
qHist = zeros(7, N); % joint limits


%% PA1 Algorithm (W14-L1 Slides 10-12)

% VARIABLES
% q: current joint angles [1x7]
% qL: lower joint limits [1x7]
% qU: upper joint limits [1x7]
% p_tip: tool tip in {b} [3x1]
% pGoal: goal position in {s} [3x1]

for gIdx = 1:numel(goals)

    pGoal = goals{gIdx};
    fprintf('\n=== Goal %d: [%.2f, %.2f, %.2f] ===\n', gIdx, pGoal(1), pGoal(2), pGoal(3));

    % Update goal sphere
    set(hGoalSurf, ...
        'XData', sx*0.05 + pGoal(1), ...
        'YData', sy*0.05 + pGoal(2), ...
        'ZData', sz*0.05 + pGoal(3));

    % Reset storage and joint config for this goal
    pHist(:) = 0;  eHist(:) = 0;  zHist(:) = 0;  qHist(:) = 0;
    q = q_initial;

    % Null-space target: midpoint of each joint range
    q_mid = (qU + qL) / 2;

    % lsqlin options (suppress console output)
    opts = optimoptions('lsqlin', 'Display', 'off');

    p_tip = [0; 0; toolL];   % tip offset in body frame

    for k = 1:N

        % FK and Jacobians of current joint angle
        F = FK_space_no_plot(M, S_space, q); % T from {b} to {s}
        Js = J_space(S_space, q);
        J_alpha = Js(1:3, :);
        J_eps = Js(4:6,:);

        % define tool axis along z axis of body frame - make sure this is right
        R = F(1:3,1:3);
        z_s = R(:, 3);
        zHist(:, k) = z_s; % store for animation

        % tip position in space frame
        t_h = F*[p_tip; 1]; % 4x4*4x1 calculation from {b} to {s}
        p_current = t_h(1:3); % 3x1 tip poition

        % Form Ax-b problem (W14-L1-SL12)
        A_pos = skewSym(-p_current)*J_alpha+J_eps;
        b_pos = p_current-pGoal;

        % Position error
        b_pos = p_current - pGoal;
        dist  = norm(b_pos);

        %% --- Convergence check ---
        if dist < dMax
            fprintf('  Goal %d reached at step k=%d (t=%.2fs), final error = %.4f m\n', ...
                gIdx, k, (k-1)*dt, dist);
            pHist(:, k:end) = repmat(p_current, 1, N-k+1);
            eHist(k:end)    = dist;
            zHist(:, k:end) = repmat(z_s, 1, N-k+1);
            qHist(:, k:end) = repmat(q,   1, N-k+1);
            break
        end
        if k == N
            fprintf('  Goal %d: did not converge. Final error = %.4f m\n', gIdx, dist);
        end

        %%  Build task-space matrix 
        A_pos = skewSym(-p_current) * J_alpha + J_eps;   % 3x7

        %%  Soft joint limit bounds (with buffer margin) 
        qMin = qL - q + margin;
        qMax = qU - q - margin;

        %%  Linearized ball constraint ( sign was wrong before) 
        %  ||e + A*dq|| <= r  ->  (e/||e||)' A dq <= r - ||e||
        e_hat  = b_pos / dist;
        A_ineq = e_hat' * A_pos;          % 1x7
        b_ineq = dMax - dist;             % negative when outside ball (active)

        %% --- Regularized least-squares (prevents joint railing) 
        %  Augment with lambda*I to penalize large dq
        W     = diag([1, 1, 1, 1, 1, 1, 1]);   % per-joint weights (tune if needed)
        A_aug = [A_pos; sqrt(lambda) * W];
        b_aug = [-b_pos; zeros(7,1)];

        dq = lsqlin(A_aug, b_aug, A_ineq, b_ineq, [], [], qMin, qMax, [], opts);

        if isempty(dq)
            disp('  Solver failed — stopping.');
            break
        end

        %%  Clamp step size (prevents large single-step excursions) 
        dq = max(min(dq, dq_clamp), -dq_clamp);

        %%  Null-space joint centering (pulls joints away from limits) 
        %  Project a centering signal into the null space of A_pos
        J_pinv  = pinv(A_pos);
        N_proj  = eye(7) - J_pinv * A_pos;       % null-space projector
        dq_null = k_null * (q_mid - q);           % centering direction
        dq      = dq + N_proj * dq_null;

        %%  Update joints 
        q = q + dq*dt;
        %%  Recompute FK with updated q for display (dot/robot in sync) 
        F_new     = FK_space_no_plot(M, S_space, q);
        p_display = F_new * [p_tip; 1];
        p_display = p_display(1:3);

        %%  Store 
        pHist(:, k) = p_display;
        eHist(k)    = dist;
        qHist(:, k) = q;
        timeVec     = (0:k-1) * dt;

         %%  Update Robot Simulation (ax1) 
        light(ax1, 'Position', [ 2  2  3], 'Style', 'infinite', 'Color', [1.00 0.97 0.92]);
        light(ax1, 'Position', [-2 -1  2], 'Style', 'infinite', 'Color', [0.30 0.40 0.60]);
        light(ax1, 'Position', [ 0  3 -1], 'Style', 'infinite', 'Color', [0.15 0.15 0.20]);
        lighting(ax1, 'gouraud');
        material(ax1, 'metal');
        show(robot, q, 'Parent', ax1, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
        set(traj, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
        set(tipP, 'XData', p_display(1),  'YData', p_display(2),  'ZData', p_display(3));
 
        %%  Update Error Plot (ax2) 
        set(errLine, 'XData', (0:k-1)*dt, 'YData', eHist(1:k));
 
        %%  Update Orientation Plot (ax3) 
        set(oriLines(1), 'XData', timeVec, 'YData', zHist(1, 1:k));
        set(oriLines(2), 'XData', timeVec, 'YData', zHist(2, 1:k));
        set(oriLines(3), 'XData', timeVec, 'YData', zHist(3, 1:k));
 
        %%  Update Joint Limit Plot (ax4) 
        qDeg      = rad2deg(q);
        threshold = 0.05;
        set(hBar, 'YData', qDeg);
        for i = 1:7
            yOffset = sign(qDeg(i)) * 15;
            if qDeg(i) == 0; yOffset = 15; end
            set(hTexts(i), 'Position', [i, qDeg(i) + yOffset, 0], ...
                'String', sprintf('%.1f°', qDeg(i)));
 
            rangeRad  = qU(i) - qL(i);
            nearLimit = (qU(i) - q(i) < rangeRad * threshold) || ...
                        (q(i) - qL(i) < rangeRad * threshold);
            if nearLimit
                hBar.CData(i,:) = [1 0 0];
                set(hTexts(i), 'Color', 'r');
            else
                hBar.CData(i,:) = uniformColor;
                set(hTexts(i), 'Color', 'w');
            end
        end
 
        drawnow limitrate;
 
    end % k loop
 
    pause(1.5);   %  pause between goals
 
end % goal loop