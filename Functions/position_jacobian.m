function Jp = position_jacobian(q, ~, d_flange, d_tool)
% POSITION_JACOBIAN  3×7 Jacobian mapping joint velocities to tool-tip
%                    linear velocity, computed via PoE.
%
%   Jp = position_jacobian(q, [], d_flange, d_tool)
%
%   This is the linear (top 3) block of the geometric Jacobian,
%   accounting for the rigid tool offset on the last link.
%
%   For a revolute joint k with accumulated transform T_{0,k-1}:
%     Jp_k = J_linear_k + (w_k_world) × (p_tip - p_k_world)
%
%   where p_k_world = T_{0,k-1}(1:3,4) and w_k_world is the rotated
%   axis from the spatial Jacobian.

J  = geometric_jacobian(q, [], d_flange, d_tool);

% Tip position
[T, ~] = forward_kinematics(q, [], d_flange, d_tool);
p_tip  = T(1:3,4);

% Build Jp via the geometric relation:
%   Jp_k = Jv_k + omega_k × (p_tip - p_k)
% The spatial Jacobian already encodes p_k implicitly through the
% v = -w × p term.  For the linear-velocity block we can also just
% use the cross-product form directly.

% Rebuild screw axis origins and world-frame axes at configuration q
p_home = [0,       0,       0.333;
          0,       0,       0.333;
          0,       0,       0.649;
          0.0825,  0,       0.649;
          0,       0,       1.033;
          0,       0,       1.033;
          0.088,   0,       1.033];

w_home = [0,  0,  1;
          0,  1,  0;
          0,  0,  1;
          0, -1,  0;
          0,  0,  1;
          0, -1,  0;
          0,  0, -1];

S_home = zeros(6,7);
for k = 1:7
    wk = w_home(k,:)';
    pk = p_home(k,:)';
    S_home(:,k) = [wk; -skewSym(wk)*pk];
end

Jp    = zeros(3,7);
T_cur = eye(4);

for k = 1:7
    % World-frame axis of joint k
    Sk_world  = adjoint(T_cur) * S_home(:,k);
    w_k       = Sk_world(1:3);   % angular part (world frame)
    p_k       = T_cur(1:3,4);    % joint origin in world frame

    % Linear velocity contribution of joint k at the tool tip
    Jp(:,k) = skewSym(w_k) * (p_tip - p_k);   % = w_k × (p_tip - p_k)

    T_cur = T_cur * matrix_exp_screw(S_home(:,k), q(k));
end
end