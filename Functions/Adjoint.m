function Ad = Adjoint(T) % W5-L1 slide 1 
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    p_sk = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    Ad = [R, zeros(3); p_sk*R, R];
end