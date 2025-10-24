function total_tts = objective_tts(u_control, x0, N_sim, params)
%   OBJECTIVE_TTS Calculates the total TTS for the freeway simulation.
%   total_tts = OBJECTIVE_TTS(u_control, x0, N_sim, params) takes the
%   control vector u_control (containing r_control and VSL_control),
%   the initial state x0, number of simulation steps N_sim, and the
%   parameters struct params. It runs the simulation and returns the
%   scalar sum of the instantaneous TTS values.

    % --- Input Validation ---
    if length(u_control) ~= 2 * (N_sim * params.T / params.Tc) || mod(length(u_control), 2) ~= 0
         error('u_control vector has incorrect length.');
    end

    % --- Extract Control Sequences ---
    Nc = length(u_control) / 2; % Number of control steps
    r_control = u_control(1:Nc);
    VSL_control = u_control(Nc+1:end);

    % --- Ensure Bounds (redundant if fmincon bounds are set, but safe) ---
    r_control = max(0, min(1, r_control));
    VSL_control = max(60, min(120, VSL_control));

    % --- Run Simulation ---
    try
        % Assuming simulate_freeway.m is in the MATLAB path
        [~, Y_hist] = simulate_freeway(x0, r_control, VSL_control, N_sim, params);

        % --- Calculate Total TTS ---
        % Sum can sometimes yield NaN if simulation fails or produces NaNs
        if any(isnan(Y_hist)) || any(isinf(Y_hist))
             warning('Simulation produced NaN or Inf values. Returning Inf for objective.');
             total_tts = inf;
        else
            total_tts = sum(Y_hist);
        end

    catch
        % Handle potential errors during simulation (e.g., numerical issues)
        warning('Error during simulation: %s. Returning Inf for objective.', ME.message);
        total_tts = inf; % Return a large value if simulation fails
    end

end