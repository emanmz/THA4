function Jp = position_jacobian(q, DH, d_flange, d_tool)
    J  = geometric_jacobian(q, DH, d_flange, d_tool);
    Jp = J(1:3,:);
end