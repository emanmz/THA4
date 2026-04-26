
function smat = skewSym(X)
% X = 3x1 vector
smat = [0 -X(3) X(2);
    X(3) 0 -X(1);
    -X(2) X(1) 0];
end 