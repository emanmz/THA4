function T = matrix_exp_screw(S, theta)
% MATRIX_EXP_SCREW  Compute the 4×4 SE(3) matrix exponential
%                   T = exp( [S] * theta )
%
%   S     – 6×1 screw axis [omega; v]  (unit screw, ||omega||=1 for revolute)
%   theta – scalar joint angle (radians) for revolute joint
%
%   Uses the closed-form Rodrigues / screw formula (Modern Robotics §3.2):
%
%     For ||omega|| = 1  (revolute):
%       R = I + sin(theta)*[omega] + (1-cos(theta))*[omega]^2
%       p = (I*theta + (1-cos(theta))*[omega] + (theta-sin(theta))*[omega]^2) * v
%
%     For ||omega|| = 0  (prismatic):
%       R = I,   p = v * theta
%
%   No trigonometric builtins are used beyond sin/cos (which are elementary
%   and not "robotics builtins").

omega = S(1:3);
v     = S(4:6);

T = eye(4);

nrm = sqrt(omega(1)^2 + omega(2)^2 + omega(3)^2);

if nrm < 1e-10
    % Prismatic joint
    T(1:3,4) = v * theta;
    return;
end

% Normalise (should already be unit, but guard numerics)
omega = omega / nrm;
v     = v     / nrm;
theta = theta * nrm;

W  = skewSym(omega);      % 3×3 skew-symmetric matrix of omega
W2 = W * W;               % [omega]^2

st = sin(theta);
ct = cos(theta);

R = eye(3) + st * W + (1 - ct) * W2;
p = (eye(3)*theta + (1 - ct)*W + (theta - st)*W2) * v;

T(1:3,1:3) = R;
T(1:3,4)   = p;
end