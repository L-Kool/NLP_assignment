clear 
clc

% Specific numbers based on student IDs
E1 = 5;
E2 = 13;
E3 = 6;

% System parameters
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
params.T_c = 60;            % Control signal sampling time [s]
params.D_r = 1500;          % ramp demand [veh/h]
params.L = 1;               % length of road segment [km]
params.lambda = 2;          % number of lanes

% Initial conditions
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x_0 = [rho_0 ; v_0 ; w_r_0];

% Simulation setup
sim_steps = 120;
state = zeros(13, sim_steps + 1);
state(:, 1) = x_0;  
output = zeros(sim_steps + 1, 1);
output(1) = params.T * x_0(13) + params.T * params.L * params.lambda * sum(x_0(7:12));

% Control signal
% r_k first row
% vsl second row
u_control = zeros(2, sim_steps);

% Simulation
for k = 1:sim_steps
    state_current = state(:, k);
    u_control_current = u_control(:, k);

    v_next = update_velocity(state_current, u_control_current, params);
    rho_next = update_density(state_current, u_control_current, params, k, E2);
    w_r_next = update_wr(state_current, u_control_current, params);

    state(:, k+1) = [v_next ; rho_next ; w_r_next];
    output(k+1) = params.T * state(13,k) + params.T * params.L * params.lambda * sum(state(7:12, k));
end 

% Post-processing
state_v = state(1:6, :);
state_rho = state(7:12, :);
state_w_r = state(end, :);

% Visualisations
time = 0:sim_steps;
close all

figure(1)
plot(time, state_v);
xlabel('Time step')
ylabel('Velocity')
title('Velocities in different segments')
legend('v_1', 'v_2', 'v_3', 'v_4', 'v_5', 'v_6')

figure(2)
plot(time, state_rho);
xlabel('Time step')
ylabel('Traffic density')
title('Traffic densities in different segments')
legend('\rho1', '\rho2', '\rho3', '\rho4', '\rho5', '\rho6')

figure(3)
plot(time, state_w_r);
xlabel('Time step')
ylabel('Ramp flow')
title('Ramp flow over time')
legend('w_r')

figure(4)
plot(time, output);
xlabel('Time step')
ylabel('Total Time Spent (veh * h)')
title('Total Time Spent')
legend('TTS')

