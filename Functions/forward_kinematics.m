function [T_ee, T_all] = forward_kinematics(q, DH, d_flange, d_tool)
    T = eye(4);
    T_all(:,:,1) = T;
    for i = 1:7
        T = T * dh(DH(i,1), DH(i,2), DH(i,3), q(i) + DH(i,4));
        T_all(:,:,i+1) = T;
    end
    T_flange = T * [eye(3), [0;0;d_flange]; 0 0 0 1];
    T_ee     = T_flange * [eye(3), [0;0;d_tool]; 0 0 0 1];
end