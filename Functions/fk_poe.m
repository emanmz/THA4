function T = fk_poe(Sn, M, q)
% FK_POE  Forward kinematics via the Space-frame Product of Exponentials.
%
%   T = fk_poe(Sn, M, q)
%
%   Inputs
%   ------
%   Sn : 6×n  Space screw axes [w; v] at the zero (home) configuration,
%              one column per joint.  Pre-computed once at start-up from
%              the toolbox Jacobian at q=0 and stored in the workspace.
%   M  : 4×4  Home configuration of the end-effector (q = 0).
%              Pre-computed once at start-up with getTransform at q=0.
%   q  : n×1  Current joint angles (radians).
%
%   Output
%   ------
%   T  : 4×4  Homogeneous transform from base to end-effector.
%
%   Formula (Modern Robotics convention):
%       T(q) = e^[S1]q1 · e^[S2]q2 · ... · e^[Sn]qn · M
%
%   Each matrix exponential e^([w;v]*theta) is computed analytically via
%   the Rodrigues / screw-axis formula (no matrix expm() needed).

n = length(q);
T = eye(4);

for i = 1:n
    T = T * screw_exp(Sn(:,i), q(i));
end

T = T * M;
end


%% ── helpers ──────────────────────────────────────────────────────────────

function T = screw_exp(S, theta)
% SCREW_EXP  Closed-form matrix exponential of a screw twist.
%
%   T = screw_exp([w; v], theta)
%
%   Uses the Rodrigues formula.  Handles the pure-translation case
%   (||w|| == 0) separately.

w = S(1:3);
v = S(4:6);

w_norm = norm(w);

if w_norm < 1e-10
    % Pure translation: e^[S]θ = [I | v*θ; 0 0 0 1]
    T = eye(4);
    T(1:3,4) = v * theta;
else
    % Normalise so that ||w|| == 1 (standard PoE form already assumes this,
    % but guard anyway)
    w  = w  / w_norm;
    v  = v  / w_norm;
    th = theta * w_norm;

    % Rodrigues rotation  R = I + sin(θ)[w] + (1-cos(θ))[w]²
    W = skew(w);
    R = eye(3) + sin(th)*W + (1 - cos(th))*(W*W);

    % Translation part   p = (I·θ + (1-cos θ)[w] + (θ-sin θ)[w]²) v
    p = (eye(3)*th + (1 - cos(th))*W + (th - sin(th))*(W*W)) * v;

    T = [R, p; 0 0 0 1];
end
end


function W = skew(w)
% SKEW  3×3 skew-symmetric matrix of a 3-vector.
W = [   0,  -w(3),  w(2);
      w(3),     0, -w(1);
     -w(2),  w(1),    0 ];
end