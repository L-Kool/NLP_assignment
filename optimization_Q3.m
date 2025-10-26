%% Optimization problem Task 3a
close all; clear;

% Importing parameters
params = ImportParameters();

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

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...          % Use Sequential Quadratic Programming
    'Display', 'iter', ...           % Show output for each iteration
    'MaxFunctionEvaluations', 50000, ... % Increase limit (simulation is expensive)
    'MaxIterations', 400, ...        % Default is 400, might need more
    'OptimalityTolerance', 1e-6, ... % Default
    'StepTolerance', 1e-6, ...       % Default
    'ConstraintTolerance', 1e-6);    % Default


% Initial control conditions
% A. Ramp closed (r=0), Min speed limit (VSL=60)
u_control0_a = [zeros(Nc, 1); 60 * ones(Nc, 1)];

% B. Ramp fully open (r=1), Max speed limit (VSL=120) - "No Control"
u_control0_b = [ones(Nc, 1); 120 * ones(Nc, 1)];

%% Question 3.a, 3.b 

% Objective function
totTTS = @(u) simulation(u, x0, params, 0).totalTTS;

%% Optimization for starting point (a)
tic; % Start timer
[u_opt_a, f_opt_a, exitflag_a, output_opt_a] = fmincon(totTTS, u_control0_a, [], [], [], [], lb, ub, [], options);
time_a = toc; % End timer
% Print the cost function from the optimization
fprintf('Exit flag of optimization 3 for starting point(a): %d\n', exitflag_a)
fprintf('Optimization time for starting point (a): %.4f\n', time_a)
fprintf('Optimal cost function value for starting point (a): %.4f\n', f_opt_a)

%% Optimization for starting point (b)
tic; % Start timer
[u_opt_b, f_opt_b, exitflag_b, output_opt_b] = fmincon(totTTS, u_control0_b, [], [], [], [], lb, ub, [], options);
time_b = toc; % End timer

%% Question 3.a, 3.b with q_0(k) 50% higher

% Objective function
totTTS_1 = @(u) simulation(u, x0, params, 1).totalTTS;

% Optimization for starting point (a)
tic; % Start timer
[u_opt1_a, f_opt1_a, exitflag1_a, output1_opt_a] = fmincon(totTTS_1, u_control0_a, [], [], [], [], lb, ub, [], options);
time1_a = toc; % End timer

% Optimization for starting point (b)
tic; % Start timer
[u_opt1_b, f_opt1_b, exitflag1_b, output1_opt_b] = fmincon(totTTS_1, u_control0_b, [], [], [], [], lb, ub, [], options);
time1_b = toc; % End timer

%% Task 4
% For each given optimal control input, we run a new simulation again for
% each case that returns us plots.

% 0 --> condition (a)
simResults_a = simulation(u_opt_a, x0, params, 0);
state_a = simResults_a.stateHist;
output_a = simResults_a.outputHist;

simResults_b = simulation(u_opt_b, x0, params, 0);
state_b = simResults_b.stateHist;
output_b = simResults_b.outputHist;

% 1 --> condition (b)
simResults1_a = simulation(u_opt1_a, x0, params, 1);
state1_a = simResults1_a.stateHist;
output1_a = simResults1_a.outputHist;

simResults1_b = simulation(u_opt1_b, x0, params, 1);
state1_b = simResults1_b.stateHist;
output1_b = simResults1_b.outputHist;

% Extract relevant states
[V_a, Rho_a, Wr_a, r_a, VSL_a] = extract_data(state_a, u_opt_a);
[V_b, Rho_b, Wr_b, r_b, VSL_b] = extract_data(state_b, u_opt_b);
[V_a1, Rho_a1, Wr_a1, r_a1, VSL_a1] = extract_data(state1_a, u_opt1_a);
[V_b1, Rho_b1, Wr_b1, r_b1, VSL_b1] = extract_data(state1_b, u_opt1_b);



%% Plotting
% Task 3 Comparison (Different Starting Points, Original Inflow)
fprintf('Plotting Task 3 comparison (Start (a) vs Start (b))\n');
num_segments = 6;

segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:num_segments, 'UniformOutput', false);

sim_time_s = (1:121).*10;

% Speeds 3
figure('Name', 'Task 3 Speeds Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_a'); title('Speeds - Start (a) (r=0, VSL=60)'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_b'); title('Speeds - Start (b) (r=1, VSL=120)'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3
figure('Name', 'Task 3 Densities Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_a'); title('Densities - Start (a) (r=0, VSL=60)'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_b'); title('Densities - Start (b) (r=1, VSL=120)'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3
figure('Name', 'Task 3 Queue Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_a, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start (a) (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_b, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start (b) (r=1, VSL=120)');
hold off;
title('Queue Length Comparison - Task 3', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14);
ylabel('Queue [veh]', 'FontSize', 14);
xlim([0 1210]);
legend('show');
grid on;

output_time_s = (1:120).*10;

% Inputs 3
figure('Name', 'Task 3 Inputs Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_a, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start (a)'); hold on;
stairs(output_time_s, r_b, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start (b)'); hold off;
title('Ramp Metering Rate (Applied)'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_a, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start (a)'); hold on;
stairs(output_time_s, VSL_b, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start (b)'); hold off;
title('Variable Speed Limit (Applied)'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');


%% Task 3_ext Comparison (Different Starting Points, Increased Inflow)

% Speeds 3_ext
figure('Name', 'Task 3_{ext} Speeds Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_a1'); title('Speeds - Start (a) (r=0, VSL=60), Increased Inflow'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_b1'); title('Speeds - Start (b) (r=1, VSL=120), Increased Inflow'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3_ext
figure('Name', 'Task 3_{ext} Densities Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_a1'); title('Densities - Start (a) (r=0, VSL=60), Increased Inflow'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_b1'); title('Densities - Start (b) (r=1, VSL=120), Increased Inflow'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3_ext
figure('Name', 'Task 3_{ext} Queue Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start (a) (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_b1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start (b) (r=1, VSL=120)');
hold off;
title('Queue Length Comparison - Task 3_ext (Increased Inflow)');
xlabel('Time [s]');
ylabel('Queue [veh]');
xlim([0 1210]);
legend('show');
grid on;

% Inputs 3_ext
figure('Name', 'Task 3_{ext} Inputs Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_b1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Ramp Metering Rate (Applied) - Increased Inflow'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_b1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Variable Speed Limit (Applied) - Increased Inflow'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');

