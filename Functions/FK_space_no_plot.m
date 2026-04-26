function T = FK_space_no_plot(M, Sn, theta)
    % T = e^[S1]th1 * e^[S2]th2 * ... * e^[Sn]thn * M
    T = eye(4);
    for i = 1:length(theta)
        T = T * screw_to_exp(Sn(:,i), theta(i));
    end
    T = T * M;
end