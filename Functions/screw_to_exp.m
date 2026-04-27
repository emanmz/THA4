function expS = screw_to_exp(S, theta) % W5-L1 slide 6 & W5-L1 slide 10
% converts a screw axis and angle to a transformation matrix.
% S: 6x1 screw axis [w; v]
% theta: scalar joint angle

    w = S(1:3);
    v = S(4:6);
    % Skew-symmetric matrix of w
    w_sk = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    
    if norm(w) == 0     % Prismatic joint
        expS = [eye(3), v * theta; 0 0 0 1];
    else                % Revolute joint
        % Rodrigues' formula for Rotation
        R = eye(3) + sin(theta)*w_sk + (1-cos(theta))*w_sk^2;
        % Translation component
        p = (eye(3)*theta + (1-cos(theta))*w_sk + (theta-sin(theta))*w_sk^2)*v;
        expS = [R, p; 0 0 0 1];
    end
end