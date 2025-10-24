function [V, Rho, Wr, r_sim, VSL_sim] = extract_data(state_hist, u_opt_vec)
    V = state_hist(1:6, :);             % Speed history [km/h]
    Rho = state_hist(7:12, :);          % Density history [veh/(km*lane)]
    Wr = state_hist(13, :);             % Queue length history [veh]

    r_control = u_opt_vec(1:20);
    VSL_control = u_opt_vec(21:end);

    r_sim = repelem(r_control, 6);
    VSL_sim = repelem(VSL_control, 6);
end