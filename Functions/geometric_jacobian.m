function J = geometric_jacobian(q, ~, d_flange, d_tool)
% GEOMETRIC_JACOBIAN  6×7 spatial Jacobian for Franka Panda via PoE.
%
%   J = geometric_jacobian(q, [], d_flange, d_tool)
%
%   Uses the space-Jacobian formula:
%     J_k = Ad(T_{0,k-1}) * S_k
%   where T_{0,k-1} = exp(S1*q1)*...*exp(S_{k-1}*q_{k-1})
%   and Ad(T) is the 6×6 adjoint of T in SE(3).
%
%   Second argument ignored (API compatibility).
%
%   OUTPUT
%     J  –  6×7 matrix.  Rows 1:3 = linear (angular in PoE space form),
%                         rows 4:6 = linear velocity part.
%           Layout matches geometric_jacobian convention used by
%           the controller: J(1:3,:) → linear, J(4:6,:) → angular.
%
%   NOTE: The space Jacobian gives the body twist in world coords.
%   For a revolute joint:  J_k = [Ad(T_{0,k-1})]_spatial * S_k

%% Screw axes at q=0 (world frame) — identical to forward_kinematics.m
p = [0,       0,       0.333;
     0,       0,       0.333;
     0,       0,       0.649;
     0.0825,  0,       0.649;
     0,       0,       1.033;
     0,       0,       1.033;
     0.088,   0,       1.033];

w = [0,  0,  1;
     0,  1,  0;
     0,  0,  1;
     0, -1,  0;
     0,  0,  1;
     0, -1,  0;
     0,  0, -1];

S = zeros(6,7);
for k = 1:7
    wk = w(k,:)';
    pk = p(k,:)';
    S(:,k) = [wk; -skewSym(wk)*pk];
end

%% Build Jacobian column by column
J     = zeros(6,7);
T_cur = eye(4);   % accumulates exp(S1*q1)*...*exp(S_{k-1}*q_{k-1})

for k = 1:7
    % J_k = Ad(T_cur) * S_k
    J(:,k) = adjoint(T_cur) * S(:,k);
    % Advance accumulator
    T_cur = T_cur * matrix_exp_screw(S(:,k), q(k));
end

% Reorder rows so J(1:3,:) = linear velocity, J(4:6,:) = angular velocity
% PoE space-Jacobian convention: row 1:3 = angular, row 4:6 = linear.
% Swap to match the controller expectation (linear first).
J = J([4 5 6 1 2 3], :);
end