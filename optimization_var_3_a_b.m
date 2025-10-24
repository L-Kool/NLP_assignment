function totalTTS = optimization_var_3_a_b(u, x_0)

E1 = 5;
E2 = 13;
E3 = 6;

params.tau = 19 / 3600;     % model parameter [h]
params.mu = 60;             % model parameter [km^2/h]
params.C_r = 2000;          % on-ramp capacity [veh/h]
params.rho_m = 120;         % maximum density[veh/km lane]
params.alpha = 0.1;         % non-compliance of drivers to speed limit shown [-]
params.K = 40;              % model parameter [veh/km lane]
params.a = 1.867;           % model parameter [-]
params.v_f = 120;           % free-flow speed that cars reach in steady state in low density freeway [km/h]
params.rho_c = 33 + E1/3;   % critical density [veh/kmlane]
params.T = 10 / 3600;       % Sampling time for r(k) [h]
params.T1 = 10;             % Sampling time for r(k) [s]
params.T_c = 60;            % Control signal sampling time [s]
params.D_r = 1500;          % ramp demand [veh/h]
params.L = 1;               % length of road segment [km]
params.lambda = 2;          % number of lanes

% Control signal
u = [u(1:20)'; 
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
    rho_next = update_density_var_b(state_current, u_control_current, params, k, E2);
    w_r_next = update_wr(state_current, u_control_current, params);

    state(:, k+1) = [v_next ; rho_next ; w_r_next];
    output(k+1) = params.T * state(13,k) + params.T * params.L * params.lambda * sum(state(7:12, k));
end 

totalTTS = sum(output);
end