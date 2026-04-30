clear; close all; clc;
addpath("Functions")

%% Objectives
% 1) Respect joint limits
% 2) Move CYLINDER TIP to pGoal
% AND THEN
% 3) Stay within 3 mm of pGoal

%% Joint Limits
qL = deg2rad([-166; -101; -166; -176; -166; -1;  -166]); % lower lim
qU = deg2rad([ 166;  101;  166;   -4;  166; 215; 166]); % upper lim

%% Load Robot
robot = loadrobot('frankaEmika','DataFormat','column');
removeBody(robot,'panda_hand');

%% TOOL CYLINDER
toolL = 0.10;       % 100 mm
toolR = 0.0025;     % radius = 2.5 mm

tool = rigidBody('tool');
jnt  = rigidBodyJoint('fix1','fixed');
setFixedTransform(jnt,eye(4));
tool.Joint = jnt;

% cylinder starts at flange and extends +Z
addVisual(tool,"Cylinder",[toolR toolL],trvec2tform([0 0 toolL/2]));
addBody(robot,tool,'panda_link8');

toolName = 'tool';

% tip point in local tool frame
tipLocal = [0;0;toolL;1];

%% 1. Extract the Perfect M (Home Configuration of the Tip)
% Get the transform from base to 'tool' when all joints are 0
homeConfig = zeros(7,1); 
T_home_tip = getTransform(robot, homeConfig, 'tool');
M = T_home_tip; % perfect Home Matrix fomr the toolbox

%% 2. Extract the Perfect Sn (Space Screw Axes)
% The Space Jacobian at q=0 is EXACTLY the list of Screw Axes [w; v]
Js_home = geometricJacobian(robot, homeConfig, 'tool');

% geometricJacobian returns [v; w], so we swap them to match our [w; v]
% format :P
Sn = [Js_home(4:6, :); Js_home(1:3, :)];
%% Init Pose
q = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];

%% Goal
%pGoal = [0.30;0.00;0.50]; % Test 1: this one is right next to the initial pose
%pGoal = [0.0; 0.0; 0.85]; % Test 2: ABOVE base (lowkey singular
pGoal = [0.4; -0.4; 0.2]; % Test 3: Joint limits test
%pGoal = [0.6; 0.0; 0.1]; % Test 4: Testing workplace boundary 

dMax  = 0.003; % the 3 mm thing

%% Simulation
dt = 0.05;
T  = 20;
N  = round(T/dt);

Kp_far  = 1.2; % Higher value → faster motion, more aggressive correction (basically when further away we go faster towardd pgoal)
Kp_near = 0.35; % Used when the tool tip is within 3 mm of the goal: 
% Reduces motion aggressiveness near the target & Improves stability and reduces oscillations
lambda  = 0.01; % This is damped least squares inverse kinematics. It prevents numerical instability when: Jacobian becomes ill-conditioned,  
% approaches singular configurations joints align causing loss of rank

%% FIGURE
figure('Color','k','Position',[100 80 1300 750]);

ax1 = subplot(2,2,[1 3]);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
view(ax1,[45 25]);

xlim(ax1,[-0.8 0.8]);
ylim(ax1,[-0.8 0.8]);
zlim(ax1,[0 1.1]);

set(ax1,'Color',[0.08 0.08 0.12], ...
        'XColor','w','YColor','w','ZColor','w');

title(ax1,'Cylinder Tip Targets Goal','Color','w');

[sx,sy,sz] = sphere(30);

% visible goal
surf(ax1,0.02*sx+pGoal(1), ...
         0.02*sy+pGoal(2), ...
         0.02*sz+pGoal(3), ...
         'FaceColor','g','EdgeColor','none','FaceAlpha',0.7);

% 3 mm hold sphere
surf(ax1,dMax*sx+pGoal(1), ...
         dMax*sy+pGoal(2), ...
         dMax*sz+pGoal(3), ...
         'FaceColor','g','EdgeColor','none','FaceAlpha',0.2);

traj = plot3(ax1,nan,nan,nan,'c-','LineWidth',2);
tipP = plot3(ax1,nan,nan,nan,'ro','MarkerFaceColor','r','MarkerSize',5);

show(robot,q,'Parent',ax1,'PreservePlot',false,'Frames','off');

%% Error Plot
ax2 = subplot(2,2,2);
hold(ax2,'on'); grid(ax2,'on');

set(ax2,'Color',[0.08 0.08 0.12], ...
        'XColor','w','YColor','w');

title(ax2,'Tip Error','Color','w');
xlabel(ax2,'Time (s)','Color','w');
ylabel(ax2,'m','Color','w');

plot(ax2,[0 T],[dMax dMax],'g--');
errLine = plot(ax2,nan,nan,'c-','LineWidth',2);

xlim(ax2,[0 T]);
ylim(ax2,[0 0.6]);

%% Joint Plot
ax3 = subplot(2,2,4);
hold(ax3,'on'); grid(ax3,'on');

set(ax3,'Color',[0.08 0.08 0.12], ...
        'XColor','w','YColor','w');

title(ax3,'Joint Angles (deg)','Color','w');

hBar = bar(ax3,rad2deg(q),'FaceColor',[0 0.7 0.7]);

ylim(ax3,[-190 190]);
xticks(ax3,1:7);

%% FIGURE 2: Surgical Microscope (The Zoomed View)
fig2 = figure(2);
set(fig2, 'Color', 'k', 'Position', [860 200 600 600]);
axDetail = axes('Parent', fig2);
hold(axDetail, 'on'); grid(axDetail, 'on'); axis(axDetail, 'equal');
view(axDetail, [45 25]);
% Zoomed in 1cm around the goal
set(axDetail, 'Color', [0.05 0.05 0.05], 'XColor', 'c', 'YColor', 'c', 'ZColor', 'c');
title(axDetail, 'Surgical Microscope: 3mm Zone', 'Color', 'c');
xlabel(axDetail, 'X (m)'); ylabel(axDetail, 'Y (m)'); zlabel(axDetail, 'Z (m)');

[sx, sy, sz] = sphere(30);

% Global Objects
surf(ax1, dMax*sx+pGoal(1), dMax*sy+pGoal(2), dMax*sz+pGoal(3), ...
    'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
trajMain = plot3(ax1, nan, nan, nan, 'c-', 'LineWidth', 1);
tipMain  = plot3(ax1, nan, nan, nan, 'ro', 'MarkerFaceColor', 'r');

% Detail Objects
surf(axDetail, dMax*sx+pGoal(1), dMax*sy+pGoal(2), dMax*sz+pGoal(3), ...
    'FaceColor', 'g', 'EdgeColor', 'g', 'FaceAlpha', 0.1);
trajDetail = plot3(axDetail, nan, nan, nan, 'c-', 'LineWidth', 2);
tipDetail  = plot3(axDetail, nan, nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
% Reference crosshair for pGoal
plot3(axDetail, pGoal(1), pGoal(2), pGoal(3), 'g+', 'MarkerSize', 15);
%% Storage
pHist = zeros(3,N);
eHist = zeros(1,N);

%% Task Parameters
dMax  = 0.003; 

shaveRadius = 0.002; % 2mm milling circle
shaveSpeed = 2.0;    % Rad/s (how fast it spins)
theta = 0;           % Accumulator for the circle
taskState = 0;       % 0: Approach, 1: Shaving
pGoal_orig = pGoal;

%% MAIN LOOP
for k = 1:N
   % Toolbox Forward Kinematics at the TIP
    % T_flange gets us to the base of the cylinder
    T_flange = getTransform(robot, q, 'tool'); % built in
    
    % manually calculate the tip position p by moving toolL along the flange's Z-axis
    % p is at the END of the cylinder
    p_local = [0; 0; toolL; 1];
    p_world = T_flange * p_local;
    p = p_world(1:3); 
    
    % Toolbox Jacobian shifted to the TIP
    % Get Jacobian at the 'tool' body origin
    J_full = geometricJacobian(robot, q, 'tool'); % built in 
    Jw = J_full(1:3, :);
    Jv = J_full(4:6, :);
    
    %  Jacobian from the flange to the tip point 'p'
    % Formula: V_tip = V_flange + omega x (p_tip - p_flange)
    p_flange = T_flange(1:3, 4);
    r = p - p_flange; %  is the vector along the cylinder
    
    Jv_tip = zeros(3,7);
    for j = 1:7
        Jv_tip(:,j) = Jv(:,j) + cross(Jw(:,j), r);
    end
    
    % tool axes for the shaving circle orientation
    xAxis = T_flange(1:3,1); 
    yAxis = T_flange(1:3,2);
    
    if taskState == 0
        current_target = pGoal_orig;
        e = current_target - p;
        if norm(e) < 0.001
            taskState = 1;
            fprintf('tip reached pgoal.  shaving...\n');
        end
    else
        % SHAVING PHASE
        theta = theta + shaveSpeed * dt;
        current_target = pGoal_orig + (shaveRadius * cos(theta) * xAxis) + ...
                                     (shaveRadius * sin(theta) * yAxis);
        e = current_target - p;
    end
    err = norm(e);
    
    % DLS Solver
    Kp = (taskState == 1) * Kp_near + (taskState == 0) * Kp_far;
    v_desired = Kp * e;
    
    % Damped Least Squares using the Tip Jacobian
    dq = Jv_tip' * ((Jv_tip*Jv_tip' + lambda*eye(3)) \ v_desired);
    
    % Constraints & Integration
    for i = 1:7
        margin = min(q(i)-qL(i), qU(i)-q(i));
        if margin < deg2rad(8), dq(i) = 0.25*dq(i); end
    end
    q = q + dq*dt;
    q = min(max(q,qL),qU);

    % uppdate Visuals
    pHist(:,k) = p;
    eHist(k)   = err;
    
    show(robot,q,'Parent',ax1,'PreservePlot',false,'Frames','off');
    
    % the specific p calculated for all plots
    set(trajMain,'XData',pHist(1,1:k), 'YData',pHist(2,1:k), 'ZData',pHist(3,1:k));
    set(tipMain, 'XData',p(1), 'YData',p(2), 'ZData',p(3));
    set(tipP,    'XData',p(1), 'YData',p(2), 'ZData',p(3)); % The red dot in Global View
    
    set(errLine,'XData',(0:k-1)*dt, 'YData',eHist(1:k));
    set(hBar,'YData',rad2deg(q));

    % microscope View
    set(trajDetail, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
    set(tipDetail,  'XData', p(1), 'YData', p(2), 'ZData', p(3));
    
    % microscope centered even if the goal were to move
    xlim(axDetail, [pGoal_orig(1)-0.005, pGoal_orig(1)+0.005]);
    ylim(axDetail, [pGoal_orig(2)-0.005, pGoal_orig(2)+0.005]);
    zlim(axDetail, [pGoal_orig(3)-0.005, pGoal_orig(3)+0.005]);

    drawnow limitrate
end