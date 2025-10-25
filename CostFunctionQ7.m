function totalTTS = CostFunctionQ7(x, x0, params)

    % Number of control intervals
    Nc = 20;

    % Defining the discrete value maps
    r_map = [0.2, 0.4, 0.6, 0.8];
    vsl_map = [60, 80, 100, 120];

    % Extracting the integer values (1-4) for r and VSL
    r_indices = x(1:Nc);       
    vsl_indices = x(Nc+1:end); 
    
    % Map integers to actual control values
    u_r = r_map(r_indices)';     
    u_vsl = vsl_map(vsl_indices)'; 
    
    % Combining into 40x1 vector
    u_vec = [u_r; u_vsl];

    % Using simType = 1 because of settings from Task 3.b
    totalTTS = simulation(u_vec, x0, params, 1).totalTTS;

end