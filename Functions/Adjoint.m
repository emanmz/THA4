function Ad = adjoint(T)
R = T(1:3,1:3);
p = T(1:3,4);

Ad = [R, zeros(3,3);
      skewSym(p)*R, R];
end