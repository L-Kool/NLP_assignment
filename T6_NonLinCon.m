function [c, ceq] = T6_NonLinCon(u, x0, params, W_max)
        X_hist = simulation(u, x0, params, 1).stateHist;

        % --- Extract Queue History ---
        % X_hist is (13 x N_sim+1), Wr is the 13th row
        Wr_hist = X_hist(13, :); % Includes Wr(0) at index 1

        % --- Calculate Inequality Constraints ---
        % Constraint is wr(k) <= W_max for k = 1, ..., N_sim
        % This corresponds to Wr_hist indices 2 through N_sim+1
        % fmincon requires c(u) <= 0
        c = Wr_hist(2:end)' - W_max; % Vector of size N_sim x 1
        
    % --- Equality Constraints ---
    ceq = []; % No nonlinear equality constraints

end