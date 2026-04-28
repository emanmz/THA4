clear; close all; clc;
addpath("Functions")

%% Franka Emika Set Up from THA2

% link lengths, m
L = [0.333 0.316 0.384 0.107];

% flange offset, m
A = 0.088;

% home position, https://frankarobotics.github.io/docs/robot_specifications.html#kinematic-configuration
M = [1 0 0 A;
    0 -1 0 0;
    0 0 -1 L(1)+L(2)+L(3)-L(4);
    0 0 0 1];

% Screw axis in space frame

ws = {[0;0;1], [0;-1;0], [0;0;1], [0;1;0], [0;0;1], [0;1;0], [0;0;1]};
qs = {[0;0;0], [0;0;L(1)], [0;0;L(1)], [A;0;L(1)+L(2)], [0;0;L(1)+L(2)+L(3)], [0;0;L(1)+L(2)+L(3)], [A;0;L(1)+L(2)+L(3)-L(4)]};

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
qb = {[-A;0;-L(4)+L(3)+L(2)], [-A;0;-L(4)+L(3)+L(2)], [-A;0;-L(4)+L(3)+L(2)], [0;0;-L(4)+L(3)], [-A;0;-L(4)], [-A;0;-L(4)], [0;0;0]};

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
q(1:7) = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4]; % ready position, intial configuration

%% Goal & Tool Geometry
% pGoal = [0.10; 0.10; 0.40];
pGoal = [0.30;0.00;0.50];

dMax  = 0.003; % 3 mm circle constriant 
% tip is at the end of the 100mm cylinder
toolTipOffset = [0; 0; toolL];

%% Sim Params
dt = 0.05;
T  = 15;
N  = round(T/dt);
gain = 1.5; %do we need this variable? it doesn't seem like we use it?

%% Figure Setup
figure('Color','k','Position',[100 80 1200 700]);

% 1: Robot simulation with path and goal
ax1 = subplot(2,2,1);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
view(ax1,[45 25]);
xlim(ax1,[-0.8 0.8]); ylim(ax1,[-0.8 0.8]); zlim(ax1,[0 1.1]);
title(ax1,'Franka Panda + Tool','Color','w');
set(ax1,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w','ZColor','w');

% Goal Sphere
[sx,sy,sz] = sphere(24);
% surf(ax1, sx*dMax+pGoal(1), sy*dMax+pGoal(2), sz*dMax+pGoal(3), ...
%     'FaceColor','g','EdgeColor','none','FaceAlpha',0.4);
surf(ax1, sx*0.05 + pGoal(1), sy*0.05 + pGoal(2), sz*0.05 + pGoal(3), ...
    'FaceColor','g','EdgeColor','none','FaceAlpha',0.4); %made this 5cm for debugging bc i couldnt see shit. maybe we should keep it big. 

% tip trajectory and end point
traj = plot3(ax1,NaN,NaN,NaN,'c-','LineWidth',2);
tipP = plot3(ax1,NaN,NaN,NaN,'ro','MarkerFaceColor','r', 'MarkerSize', 3);

% 2: Goal error (error between tool tip and pGoal)
ax2 = subplot(2,2,2);
hold(ax2,'on'); grid(ax2,'on');
set(ax2,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w');
title(ax2,'Goal Error','Color','w');
xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Error (m)','Color','w');
errLine = plot(ax2,NaN,NaN,'c-','LineWidth',2);
plot(ax2,[0 T],[dMax dMax],'g--'); % error threshold, 3mm
xlim(ax2,[0 T]); ylim(ax2,[0 0.6]);

% 3: Tool Orientation
ax3 = subplot(2,2,3); hold(ax3,'on'); grid(ax3,'on');
title(ax3,'Tool Z-Axis Orientation','Color','w');
set(ax3,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w');
oriLine = plot(ax3, zeros(N,3), 'LineWidth', 1.5); 
legend(ax3, {'Zx','Zy','Zz'}, 'TextColor', 'k', 'Location', 'southeast');

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

%% PA2 Algorithm (W14-L1 Slides 13-15)

% VARIABLES
% q: current joint angles [1x7] 
% qL: lower joint limits [1x7]
% qU: upper joint limits [1x7]
% p_tip: tool tip in {b} [3x1]
% pGoal: goal position in {s} [3x1]

p_tip = [0;0;toolL]; 
q_initial = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4]; % intital configuration from animation, 'ready' pose
q = q_initial;

for k =1:N

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

% Form Ax-b problem for position (W14-L1-SL12)
A_pos = skewSym(-p_current)*J_alpha+J_eps;
b_pos = p_current-pGoal;

% Form Ax-b for tool orientation (W14-L1-SL15)
A_ori = skewSym(-z_s)*J_alpha;
b_ori = zeros(3,1); % purely rotational objective

% check positional error
if norm(b_pos) < 0.001
    disp('Goal Reached');
    break
end

% ====== CONSTRAINTS ====== %

% joint limits
qMin = qL- q; 
qMax = qU - q;

% 3mm distance (linearized version of ||(skewSym(-t)*J_alpha+J_eps)*deltaq +t-p_goal||
% <= 3 W14-L1-SL12)
A_3mm = (b_pos' * A_pos) / norm(b_pos); 
b_3mm = 0.003 + norm(b_pos);

% weighting factors
zeta = 1.0;
eta = 0.5; 

% ===== SOLVE ===== %

% combine objective functions with weighting factors
A = [zeta*A_pos;
    eta*A_ori];

b = [zeta*(b_pos);
    eta*(b_ori)];

% solve with MATLAB built in 
dq = lsqlin(A, -b, A_3mm, b_3mm, [], [], qMin, qMax);

% fail safe 
if isempty(dq)
    disp('Solver Failed');
    break
end 

% update q
q = q + dq*dt; 


% ===== ANIMATION AND PLOTTING ===== %

% update animation
pHist(:,k) = p_current;
eHist(k) = norm(b_pos);
qHist(:,k) = q;
timeVec = (0:k-1)*dt;


% Update Robot Sim (1)
show(robot, q, 'Parent', ax1, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
set(traj, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
set(tipP, 'XData', p_current(1), 'YData', p_current(2), 'ZData', p_current(3));

% Update Error Plot (2)
set(errLine, 'XData', (0:k-1)*dt, 'YData', eHist(1:k));

% Update Orientation Plot (3)
set(oriLine(1), 'XData', timeVec, 'YData', zHist(1, 1:k)); % Zx
set(oriLine(2), 'XData', timeVec, 'YData', zHist(2, 1:k)); % Zy
set(oriLine(3), 'XData', timeVec, 'YData', zHist(3, 1:k)); % Zz

% Update Joint Limit PLot (4)
qDeg = rad2deg(q);
set(hBar, 'YData', qDeg);

threshold = 0.05; 
for i = 1:7
    % Update Text position
    yOffset = sign(qDeg(i)) * 15;
    set(hTexts(i), 'Position', [i, qDeg(i) + yOffset, 0], 'String', sprintf('%.1f°', qDeg(i)));
    
    % Limit Proximity Logic
    rangeRad = qU(i) - qL(i);
    if (qU(i) - q(i) < rangeRad * threshold) || (q(i) - qL(i) < rangeRad * threshold)
        hBar.CData(i,:) = [1 0 0]; % Red Alert (when joint is near limit)
        set(hTexts(i), 'Color', 'r');
    else
        hBar.CData(i,:) = uniformColor; 
        set(hTexts(i), 'Color', 'w');
    end
end


drawnow limitrate;
end 

