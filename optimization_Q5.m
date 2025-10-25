%% Optimization problem Question 5

% Defining parameters
params.E1 = 5;
params.E2 = 13;
params.E3 = 6;
params.tau = 19 / 3600;     % model parameter [h]
params.mu = 60;             % model parameter [km^2/h]
params.C_r = 2000;          % on-ramp capacity [veh/h]
params.rho_m = 120;         % maximum density[veh/km lane]
params.alpha = 0.1;         % non-compliance of drivers to speed limit shown [-]
params.K = 40;              % model parameter [veh/km lane]
params.a = 1.867;           % model parameter [-]
params.v_f = 120;           % free-flow speed that cars reach in steady state in low density freeway [km/h]
params.rho_c = 33 + params.E1/3;   % critical density [veh/kmlane]
params.T = 10 / 3600;       % Sampling time for r(k) [h]
params.T1 = 10;             % Sampling time for r(k) [s]
params.T_c = 60;            % Control signal sampling time [s]
params.D_r = 1500;          % ramp demand [veh/h]
params.L = 1;               % length of road segment [km]
params.lambda = 2;          % number of lanes

% Sim steps
Nc = 20;

% Bounds
lb_r = zeros(Nc, 1);
ub_r = ones(Nc, 1);
lb_vsl = 60 * ones(Nc, 1);
ub_vsl = 120 * ones(Nc, 1);
lb = [lb_r; lb_vsl]; % Lower bounds for u_control
ub = [ub_r; ub_vsl]; % Upper bounds for u_control

% Initial state conditions 
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x0 = [v_0 ; rho_0 ; w_r_0];

% Initial control conditions
% A. Ramp closed (r=0), Min speed limit (VSL=60)
u_control0_a = [zeros(Nc, 1); 60 * ones(Nc, 1)];

% Weights for cost terms
W_vsl = 6.3e-3;
W_ramp = 6.3e-6;

% Objective function
costQ5 = @(u) CostFunctionQ5(u, x0, params, 0, W_vsl, W_ramp).cost;

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...          % Use Sequential Quadratic Programming
    'Display', 'iter', ...           % Show output for each iteration
    'MaxFunctionEvaluations', 50000, ... % Increase limit (simulation is expensive)
    'MaxIterations', 500, ...        % Default is 400, might need more
    'OptimalityTolerance', 1e-6, ... % Default
    'StepTolerance', 1e-6, ...       % Default
    'ConstraintTolerance', 1e-6);    % Default

tic; 
% Run optimization_3_a_b.m before to have u_opt_a available for warm start
[u_opt_5, f_opt_5, exitflag_5, output_5] = fmincon(costQ5, u_opt_a, [], [], [], [], lb, ub, [], options);
time_5 = toc; 

fprintf('Task 5 optimization complete. Time: %.2f s\n', time_5);
fprintf('Optimal combined cost: %.4f\n', f_opt_5);

%% Post-processing
% Obtaining optimal history
[stateHist_opt5, outputHist_opt5, totalTTS_opt5] = simulation(u_opt_5, x0, params, 0);
costQ5_opt = CostFunctionQ5(u_opt_5, x0, params, 0, W_vsl, W_ramp);
fprintf('Cost TTS: %e\n', costQ5_opt.J_TTS);
fprintf('Cost vsl: %e\n', costQ5_opt.J_vsl);
fprintf('Cost ramp: %e\n', costQ5_opt.J_ramp);

% Plotting optimal ramp metering rates and VSL
[V_opt_5, Rho_opt_5, Wr_opt_5, r_opt_5, VSL_opt_5] = extract_data(stateHist_opt5, u_opt_5);

figure('Name', 'Task 5: Control Input Comparison');
subplot(2,1,1);
stairs(r_a, 'b:', 'LineWidth', 2, 'DisplayName', 'Task 3a (TTS-Only)');
hold on;
stairs(r_opt_5, 'b-', 'LineWidth', 3, 'DisplayName', 'Task 5 (Multi-Obj)');
title('Ramp Metering Rate Comparison', 'FontSize', 14);
ylabel('Rate', 'FontSize', 14); 
ylim([-0.1 1.1]);
legend('show', 'FontSize', 14); 
grid on;
hold off;

subplot(2,1,2);
stairs(VSL_a, 'r:', 'LineWidth', 2, 'DisplayName', 'Task 3a (TTS-Only)');
hold on;
stairs(VSL_opt_5, 'r-', 'LineWidth', 3, 'DisplayName', 'Task 5 (Multi-Obj)');
title('Variable Speed Limit Comparison', 'FontSize', 14);
ylabel('[km/h]'); 
xlabel('Time [s]', 'FontSize', 14); 
ylim([50 130]);
legend('show', 'FontSize', 14); 
grid on;
hold off;


