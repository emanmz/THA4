function [T, T_joints] = forward_kinematics(q, ~, d_flange, d_tool)
% FORWARD_KINEMATICS  Franka Emika Panda FK via Product of Exponentials.
%
%   T = forward_kinematics(q, [], d_flange, d_tool)
%
%   Uses the PoE formula:  T(q) = exp(S1*q1)*...*exp(S7*q7) * M_tool
%   where M_tool = M_flange * T_tool is the zero-configuration
%   home frame of the tool tip.
%
%   OUTPUTS
%     T        – 4x4 SE(3) transform of tool tip in world frame
%     T_joints – 4x4x8 array:  T_joints(:,:,k) = pose of joint k after
%                first k exponentials (T_joints(:,:,8) = T)
%
%
%   Coordinate convention: world frame = base frame (right-hand, Z up).

%% Screw axes S_k = [w_k; v_k] with v_k = -w_k × p_k
% (revolute joint: linear part = -omega × q_point)

% Joint origins at q=0 (world frame)
p = [0,       0,       0.333;   % joint 1
     0,       0,       0.333;   % joint 2
     0,       0,       0.649;   % joint 3
     0.0825,  0,       0.649;   % joint 4
     0,       0,       1.033;   % joint 5
     0,       0,       1.033;   % joint 6
     0.088,   0,       1.033];  % joint 7

% Joint rotation axes at q=0 (world frame)
w = [0,  0,  1;   % joint 1  +Z
     0,  1,  0;   % joint 2  +Y  (note: DH alpha=-pi/2 → rotates Y into Z)
     0,  0,  1;   % joint 3  +Z
     0, -1,  0;   % joint 4  -Y
     0,  0,  1;   % joint 5  +Z
     0, -1,  0;   % joint 6  -Y  (DH alpha=+pi/2 then -pi/2 net)
     0,  0, -1];  % joint 7  -Z  (wrist rotation)

% Build 6×7 screw matrix  [w; v]  where v = -w × p
S = zeros(6,7);
for k = 1:7
    wk = w(k,:)';
    pk = p(k,:)';
    S(:,k) = [wk; -skewSym(wk)*pk];
end

%% Home frame of flange (q=0)
M_flange        = eye(4);
M_flange(1:3,4) = [0.088; 0; 0.333 + 0.316 + 0.384 + d_flange];
% Full z-stack: 0.333+0.316+0.384 = 1.033 + 0.107 flange = 1.140
M_flange(1:3,4) = [0.088; 0; 1.033 + d_flange];

% Tool tip: translate d_tool along local Z of flange (which equals world Z at q=0)
M_tool        = eye(4);
M_tool(3,4)   = d_tool;
M0            = M_flange * M_tool;   % home configuration of tool tip

%% Product of Exponentials
T_joints = zeros(4,4,8);
T_cur    = eye(4);

for k = 1:7
    T_cur          = T_cur * matrix_exp_screw(S(:,k), q(k));
    T_joints(:,:,k) = T_cur;
end

T               = T_cur * M0;
T_joints(:,:,8) = T;
end