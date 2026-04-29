function Js = space_jacobian(Sn, q)
% SPACE_JACOBIAN  Space Jacobian built from the screw axes via the adjoint.
%
%   Js = space_jacobian(Sn, q)
%
%   Inputs
%   ------
%   Sn : 6×n  Space screw axes [w; v] at the zero configuration.
%   q  : n×1  Current joint angles (radians).
%
%   Output
%   ------
%   Js : 6×n  Space Jacobian with rows ordered [w; v]  (same convention
%              as Modern Robotics).  This matches what the main script
%              expects: Jw = Js(1:3,:),  Jv = Js(4:6,:).
%
%   Formula (Modern Robotics §5.1):
%       J_s1 = S1
%       J_si = Ad( e^[S1]q1 · ... · e^[S_{i-1}]q_{i-1} ) · Si
%
%   The adjoint of a transform T = [R p; 0 1] is
%       Ad(T) = [ R,      0 ]
%               [ [p]R,   R ]
%   where [p] is the 3×3 skew-symmetric matrix of p.

n = length(q);
Js = zeros(6, n);

T_acc = eye(4);   % accumulates e^[S1]q1 · ... · e^[S_{i-1}]q_{i-1}

for i = 1:n
    % i-th column: adjoint of accumulated transform applied to i-th screw axis
    Js(:, i) = adjoint_of(T_acc) * Sn(:, i);

    % update the accumulated transform by multiplying in e^[Si]qi
    T_acc = T_acc * screw_exp(Sn(:, i), q(i));
end
end


%% ── helpers ──────────────────────────────────────────────────────────────

function AdT = adjoint_of(T)
% ADJOINT_OF  6×6 adjoint representation of a 4×4 homogeneous transform.
%
%   Modern Robotics convention:
%       Ad(T) = [ R,    0  ]
%               [ pR,   R  ]   where p = skew(T(1:3,4))
R  = T(1:3, 1:3);
p  = T(1:3, 4);
pR = skew(p) * R;

AdT = [  R,  zeros(3);
        pR,  R        ];
end


function T = screw_exp(S, theta)
% SCREW_EXP  Closed-form matrix exponential of a screw twist.
w = S(1:3);
v = S(4:6);

w_norm = norm(w);

if w_norm < 1e-10
    T = eye(4);
    T(1:3,4) = v * theta;
else
    w  = w  / w_norm;
    v  = v  / w_norm;
    th = theta * w_norm;

    W = skew(w);
    R = eye(3) + sin(th)*W + (1 - cos(th))*(W*W);
    p = (eye(3)*th + (1 - cos(th))*W + (th - sin(th))*(W*W)) * v;

    T = [R, p; 0 0 0 1];
end
end


function W = skew(w)
W = [   0,  -w(3),  w(2);
      w(3),     0, -w(1);
     -w(2),  w(1),    0 ];
end