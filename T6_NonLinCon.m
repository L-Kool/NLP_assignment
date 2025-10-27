function [c, ceq] = T6_NonLinCon(u, x0, params, W_max)
% This function returns the matrices defining the nonlinear inequality and
% equality constraints

    stateHist = simulation(u, x0, params, 1).stateHist;

    % Extracting history of w_r
    Wr_hist = stateHist(13, :); 

    % Calculating inequality constraints
    % W_r - W_max <= 0
    % fmincon requires c(u) <= 0
    c = Wr_hist(2:end)' - W_max;
    
    % No nonlinear equality constraints
    ceq = [];
end