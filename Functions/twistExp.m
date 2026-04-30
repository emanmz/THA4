function T = twistExp(S, theta)
w = S(1:3);
v = S(4:6);

if norm(w) < 1e-8
    % prismatic
    R = eye(3);
    p = v * theta;
else
    w_hat = skew(w);
    R = eye(3) + sin(theta)*w_hat + (1-cos(theta))*(w_hat*w_hat);

    A = eye(3)*theta + (1-cos(theta))*w_hat + (theta - sin(theta))*(w_hat*w_hat);
    p = A * v;
end

T = [R p; 0 0 0 1];
end