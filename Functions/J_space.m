% % PA pt. e
function Js = J_space(Sn, theta) % W7-L1-SL6

% n = number of joints
% Sn = 6xn matrix of screw axis in space frame
% theta = 1xn matrix of joint angles

n = length(theta);
Js = zeros(6,n);
T = eye(4);
% first column is screw
Js(:,1) = Sn(:,1);
T = eye(4);
for i = 2:n
    T = T * screw_to_exp(Sn(:, i-1), theta(i-1));
    Js(:, i) = Adjoint(T) * Sn(:, i);
end
end

