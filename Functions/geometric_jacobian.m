function J = geometric_jacobian(q, DH, d_flange, d_tool)
    [T_ee, T] = forward_kinematics(q, DH, d_flange, d_tool);
    p = T_ee(1:3,4);
    J = zeros(6,7);
    for i = 1:7
        z = T(1:3,3,i);
        o = T(1:3,4,i);
        J(1:3,i) = cross(z, p - o);
        J(4:6,i) = z;
    end
end