clear; close all; clc;
addpath("Functions")


%% THINGS TO FIX
% 1. animation doesnt follow line and dot when using the algorithm
% 2. redundancy resolution to avoid joint limits. our redundancy works by
% maximixing ellipsoid. maybe it could work somehow? 
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
% pGoal = [0.10; 0.10; 0.40];
pGoal = [0.30;0.00;0.50];
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
% surf(ax1, sx*dMax+pGoal(1), sy*dMax+pGoal(2), sz*dMax+pGoal(3), ...
%     'FaceColor','g','EdgeColor','none','FaceAlpha',0.4);
surf(ax1, sx*0.05 + pGoal(1), sy*0.05 + pGoal(2), sz*0.05 + pGoal(3), ...
    'FaceColor','g','EdgeColor','none','FaceAlpha',0.4); %made this 5cm for debugging bc i couldnt see shit 

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

% %% MAIN LOOP- Animation Test
% for k = 1:N
%     % get Tool Tip Position
%     Ttool = getTransform(robot, q, toolName);
%     % transform the local tip offset [0,0,0.1] into world coordinates
%     p = Ttool(1:3,1:3) * toolTipOffset + Ttool(1:3,4);
% 
%     err = pGoal - p;
%     dist = norm(err);
% 
%     % jacobian (7 joints)
%     J = geometricJacobian(robot, q, toolName);
%     Jv = J(4:6, :); % Linear velocity part for 7 joints
% 
%     % controller
%     if dist > dMax
%         v = gain * err;
%     else
%         v = zeros(3,1);
%     end
% 
%     % Simple IK (Pseudo-inverse) rn
%     dq = pinv(Jv) * v;
%     q = q + dq * dt;
% 
%     % store & Update
%     pHist(:,k) = p;
%     eHist(k) = dist;
%     l1 = light(ax1, 'Position',[2  2  3], 'Style','infinite', 'Color',[1 0.97 0.92]);
%     l2 = light(ax1, 'Position',[-2 -1  2], 'Style','infinite', 'Color',[0.3 0.4 0.6]);
%     l3 = light(ax1, 'Position',[0   3 -1], 'Style','infinite', 'Color',[0.15 0.15 0.2]);
%     lighting(ax1, 'gouraud');   % smooth shading across faces
%     material(ax1, 'metal');     % higher specular highlight → shiny robot links
%     show(robot, q, 'Parent', ax1, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
% 
%     set(traj, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
%     set(tipP, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
%     set(errLine, 'XData', (0:k-1)*dt, 'YData', eHist(1:k));
% 
%     drawnow limitrate;
% end


%% Franka Emika Set Up from THA2
% lifted this section from THA2 

% link lengths, m
L = [0.333 0.316 0.384 0.107];

% flange offset, m
A = 0.088;


% home position, https://frankarobotics.github.io/docs/robot_specifications.html#kinematic-configuration
% THIS IS A DIFFERENT HOME THAN ABOVE BUT I DONT WANT TO FIND THE M MATRIX
% WHEN ITS ALL SCRUNCHED LIKE ABOVE. SO MAYBE THE THE CONFIG ABOVE IS JUST
% THE STARTING?
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



%% PA1 Algorithm- W14-L1 Slides 10-12

% VARIABLES - check these later
% q: current joint angles [1x7] 
% qL: lower joint limits
% qU: upper joint limits
% p_tip: tool tip in {b} [3x1]
% pGoal: goal position in {s} [3x1]


% joint limits
qL_deg = [-166; -101; -166; -176; -166; -1; -166];
qU_deg = [166; 101; 166; -4; 166; 215; 166]; 
qL = deg2rad(qL_deg);
qU = deg2rad(qU_deg);

p_tip = [0;0;toolL]; %GEM
q_initial = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4]; %this is the home config you have above
q = q_initial;

for k =1:N

% FK and Jacobians of current joint angle
F = FK_space_no_plot(M, S_space, q); % T from {b} to {s}
Js = J_space(S_space, q); 
J_alpha = Js(1:3, :);
J_eps = Js(4:6,:);

% tip position in space frame
t_h = F*[p_tip; 1]; % 4x4*4x1 calculation from {b} to {s}
p_current = t_h(1:3); % 3x1 tip poition

% Form Ax-b problem (W14-L1-SL12)
A = skewSym(-p_current)*J_alpha+J_eps;
b = p_current-pGoal;

% check positional error
if norm(b) < 0.001
    disp('Goal Reached');
    break
end

% ====== Constraints ======

% joint limits
qMin = qL- q; %is this the right way to define this?
qMax = qU - q;

% 3mm distance 
% but it's not linear and i dont know how to make it linear... 
% this is what gemini said but i dont know and i can't find a clear source
% or maybe im dumb

% linearized version of ||(skewSym(-t)*J_alpha+J_eps)*deltaq +t-p_goal||
% <= 3 
A_ineq = (b' * A) / norm(b); 
b_ineq = 0.003 + norm(b);

% solve with MATLAB built in 
dq = lsqlin(A, -b, A_ineq, b_ineq, [], [], qMin, qMax);

% fail safe 
if isempty(dq)
    disp('Solver Failed');
    break
end 

% update q
q = q + dq*dt; 

% update animation
pHist(:,k) = p_current;
    eHist(k) = norm(b);
    
    % Update Visuals
    show(robot, q, 'Parent', ax1, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
    set(traj, 'XData', pHist(1,1:k), 'YData', pHist(2,1:k), 'ZData', pHist(3,1:k));
    set(tipP, 'XData', p_current(1), 'YData', p_current(2), 'ZData', p_current(3));
    set(errLine, 'XData', (0:k-1)*dt, 'YData', eHist(1:k));
    
    drawnow limitrate;
end 

