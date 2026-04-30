clear; close all; clc;
addpath("Functions")

%% Franka Parameters
L = [0.333 0.316 0.384 0.107];
A = 0.088;

M = [1 0 0 A;
     0 -1 0 0;
     0 0 -1 L(1)+L(2)+L(3)-L(4);
     0 0 0 1];

%% Screw Axes (Space Frame)
ws = {[0;0;1],[0;-1;0],[0;0;1],[0;1;0],[0;0;1],[0;1;0],[0;0;1]};
qs = {[0;0;0],[0;0;L(1)],[0;0;L(1)], ...
      [A;0;L(1)+L(2)],[0;0;L(1)+L(2)+L(3)], ...
      [0;0;L(1)+L(2)+L(3)], ...
      [A;0;L(1)+L(2)+L(3)-L(4)]};

S_space = zeros(6,7);
for i=1:7
    wi = ws{i};
    vi = cross(-wi, qs{i});
    S_space(:,i) = [wi; vi];
end

%% Joint Limits
qL = deg2rad([-166; -101; -166; -176; -166; -1; -166]);
qU = deg2rad([ 166;  101;  166;   -4;  166; 215; 166]);

%% Initial Config
q = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];

%% Goal
pGoal = [0.30;0.00;0.50];
toolL = 0.10;
p_tip = [0;0;toolL];

%% Sim Params
dt = 0.05;
T  = 15;
N  = round(T/dt);

lambda = 0.01;   % damping
Kp = 2.5;        % strong convergence

%% Storage
pHist = zeros(3,N);
eHist = zeros(1,N);
qHist = zeros(7,N);

%% MAIN LOOP
for k = 1:N

    % Forward Kinematics
    Tcurr = FK_space_no_plot(M, S_space, q);
    Js = J_space(S_space, q);

    Jw = Js(1:3,:);
    Jv = Js(4:6,:);

    % Tip position
    tip = Tcurr * [p_tip;1];
    p = tip(1:3);

    % Error
    e = pGoal - p;
    err = norm(e);

    % Stop if converged
    if err < 1e-3
        fprintf('✅ Converged in %.2f sec\n', k*dt);
        pHist(:,k:end) = repmat(p,1,N-k+1);
        eHist(k:end) = err;
        qHist(:,k:end) = repmat(q,1,N-k+1);
        break
    end

    % Position Jacobian
    J = -skewSym(p)*Jw + Jv;

    % Desired velocity
    v = Kp * e;

    % Damped Least Squares (stable!!)
    dq = J' * ((J*J' + lambda^2*eye(3)) \ v);

    % Joint limit soft scaling
    for i=1:7
        margin = min(q(i)-qL(i), qU(i)-q(i));
        if margin < deg2rad(10)
            dq(i) = 0.3 * dq(i);
        end
    end

    % Integrate
    q = q + dq*dt;
    q = min(max(q,qL),qU);

    % Log
    pHist(:,k) = p;
    eHist(k) = err;
    qHist(:,k) = q;
end

time = (0:N-1)*dt;

%% =======================
%% 📊 PLOTS ONLY
%% =======================

figure;

% 3D Trajectory
subplot(1,3,1)
plot3(pHist(1,:),pHist(2,:),pHist(3,:),'b','LineWidth',2); hold on;
scatter3(pGoal(1),pGoal(2),pGoal(3),80,'r','filled');
grid on; axis equal;
title('End-Effector Trajectory');
xlabel('X'); ylabel('Y'); zlabel('Z');

% Error Plot
subplot(1,3,2)
plot(time,eHist,'LineWidth',2); hold on;
yline(0.003,'r--','3 mm');
grid on;
title('Convergence to Goal');
xlabel('Time (s)');
ylabel('Error (m)');

% Joint Angles
subplot(1,3,3)
plot(time, rad2deg(qHist)','LineWidth',1.5);
grid on;
title('Joint Angles');
xlabel('Time (s)');
ylabel('Degrees');
legend('q1','q2','q3','q4','q5','q6','q7');