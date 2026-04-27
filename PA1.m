clear; close all; clc;

%% Robot Setup
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
q(1:7) = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];

%% Goal & Tool Geometry
pGoal = [0.10; 0.10; 0.40];
dMax  = 0.003; % here is the 3 mm circle 
% tip is at the end of the 100mm cylinder
toolTipOffset = [0; 0; toolL];

%% Sim Params
dt = 0.05;
T  = 15;
N  = round(T/dt);
gain = 1.5;

%% Figure Setup
figure('Color','k','Position',[100 80 1200 700]);
ax1 = subplot(1,2,1);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
view(ax1,[45 25]);
xlim(ax1,[-0.8 0.8]); ylim(ax1,[-0.8 0.8]); zlim(ax1,[0 1.1]);
title(ax1,'Franka Panda + Tool','Color','w');
set(ax1,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w','ZColor','w');

% Goal Sphere
[sx,sy,sz] = sphere(24);
surf(ax1, sx*dMax+pGoal(1), sy*dMax+pGoal(2), sz*dMax+pGoal(3), ...
    'FaceColor','g','EdgeColor','none','FaceAlpha',0.4);

traj = plot3(ax1,NaN,NaN,NaN,'c-','LineWidth',2);
tipP = plot3(ax1,NaN,NaN,NaN,'ro','MarkerFaceColor','r');

ax2 = subplot(1,2,2);
hold(ax2,'on'); grid(ax2,'on');
set(ax2,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w');
title(ax2,'Goal Error','Color','w');
xlabel(ax2,'Time (s)','Color','w'); ylabel(ax2,'Error (m)','Color','w');
errLine = plot(ax2,NaN,NaN,'c-','LineWidth',2);
plot(ax2,[0 T],[dMax dMax],'g--');
xlim(ax2,[0 T]); ylim(ax2,[0 0.6]);

%% Storage
pHist = zeros(3,N);
eHist = zeros(1,N);

%% MAIN LOOP
for k = 1:N
    % get Tool Tip Position
    Ttool = getTransform(robot, q, toolName);
    % transform the local tip offset [0,0,0.1] into world coordinates
    p = Ttool(1:3,1:3) * toolTipOffset + Ttool(1:3,4);

    err = pGoal - p;
    dist = norm(err);

    % jacobian (7 joints)
    J = geometricJacobian(robot, q, toolName);
    Jv = J(4:6, :); % Linear velocity part for 7 joints

    % controller
    if dist > dMax
        v = gain * err;
    else
        v = zeros(3,1);
    end

    % Simple IK (Pseudo-inverse) rn
    dq = pinv(Jv) * v;
    q = q + dq * dt;

    % store & Update
    pHist(:,k) = p;
    eHist(k) = dist;
    l1 = light(ax1, 'Position',[2  2  3], 'Style','infinite', 'Color',[1 0.97 0.92]);
    l2 = light(ax1, 'Position',[-2 -1  2], 'Style','infinite', 'Color',[0.3 0.4 0.6]);
    l3 = light(ax1, 'Position',[0   3 -1], 'Style','infinite', 'Color',[0.15 0.15 0.2]);
    lighting(ax1, 'gouraud');   % smooth shading across faces
    material(ax1, 'metal');     % higher specular highlight → shiny robot links
    show(robot, q, 'Parent', ax1, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);

    set(traj, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
    set(tipP, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
    set(errLine, 'XData', (0:k-1)*dt, 'YData', eHist(1:k));

    drawnow limitrate;
end