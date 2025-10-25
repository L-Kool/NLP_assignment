function totalTTS = CostFunctionQ7(x, x0, params)
% Cost function for Task 7 (Discrete Optimization)
%
% INPUTS:
%   x:      A 40x1 vector of INTEGERS (1-4) from the GA solver
%   x0:     Initial state vector
%   params: Struct of model parameters

    Nc = 20; % 20 control intervals

    % --- 1. Define the Discrete Value Maps ---
    % Map integers 1, 2, 3, 4 to your discrete sets
    r_map = [0.2, 0.4, 0.6, 0.8];
    vsl_map = [60, 80, 100, 120];

    % --- 2. Translate the Integer Vector 'x' ---
    % Get the integer choices from ga
    x_r_indices = x(1:Nc);       % Integers (1-4) for ramp
    x_vsl_indices = x(Nc+1:end); % Integers (1-4) for VSL
    
    % Map integers to actual control values
    u_r = r_map(x_r_indices)';     % Transpose to make column vector
    u_vsl = vsl_map(x_vsl_indices)'; % Transpose to make column vector
    
    % Combine into the 40x1 control vector 'u' that simulation.m expects
    u_vec = [u_r; u_vsl];

    % --- 3. Run the Simulation ---
    try
        % We use simType = 1, as Task 7 builds on Task 3(b)
        [~, outputHist] = simulation(u_vec, x0, params, 1);
        
        % Calculate the total TTS
        totalTTS = sum(outputHist);
        
        % Handle simulation failures (NaNs or Infs)
        if isnan(totalTTS) || isinf(totalTTS)
            totalTTS = 1e10; % Return a large penalty cost
        end
        
    catch
        % Catch any other errors during simulation
        totalTTS = 1e10; % Return a large penalty cost
    end
end