function simResults = simulation(u, x_0, params, simType)

% Control signal
u = [u(1:20)' 
    u(21:end)'];
u_control = repelem(u, 1, 6);
sim_steps = 120;

% State init
state = zeros(13, sim_steps + 1);
state(:, 1) = x_0;  
output = zeros(sim_steps + 1, 1);
output(1) = params.T * x_0(13) + params.T * params.L * params.lambda * sum(x_0(7:12));
    
    % Simulation
    for k = 1:sim_steps
        state_current = state(:, k);
        u_control_current = u_control(:, k);
    
        v_next = update_velocity(state_current, u_control_current, params);
        rho_next = update_density(state_current, u_control_current, params, k, simType);
        w_r_next = update_wr(state_current, u_control_current, params);
    
        state(:, k+1) = [v_next ; rho_next ; w_r_next];
        output(k) = params.T * state(13,k) + params.T * params.L * params.lambda * sum(state(7:12, k));
    
        % Calculate the instantaneous TTS
        if k == sim_steps
        output(k+1) = params.T * state(13,k+1) + params.T * params.L * params.lambda * sum(state(7:12, k+1));
        end 

    end 

simResults.stateHist = state;
simResults.outputHist = output;
simResults.totalTTS = sum(output);

end