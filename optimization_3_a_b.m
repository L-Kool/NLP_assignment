%% Optimization problem Task 3a
% parameters

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
% 1. Ramp closed (r=0), Min speed limit (VSL=60)
u_control0_1 = [zeros(Nc, 1); 60 * ones(Nc, 1)];

% 2. Ramp fully open (r=1), Max speed limit (VSL=120) - "No Control"
u_control0_2 = [ones(Nc, 1); 120 * ones(Nc, 1)];

% Objective function
f = @(u) simulation(u, x0);

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...          % Use Sequential Quadratic Programming
    'Display', 'iter', ...           % Show output for each iteration
    'MaxFunctionEvaluations', 50000, ... % Increase limit (simulation is expensive)
    'MaxIterations', 400, ...        % Default is 400, might need more
    'OptimalityTolerance', 1e-6, ... % Default
    'StepTolerance', 1e-6, ...       % Default
    'ConstraintTolerance', 1e-6);    % Default

tic; % Start timer
[u_opt1, f_opt1, exitflag1, output1] = fmincon(f, u_control0_1, [], [], [], [], lb, ub, [], options);
time1 = toc; % End timer

tic; % Start timer
[u_opt2, f_opt2, exitflag2, output2] = fmincon(f, u_control0_2, [], [], [], [], lb, ub, [], options);
time2 = toc; % End timer

%% Task 3b

% objective function
f_b = @(u) optimization_var_3_a_b(u, x0);

tic; % Start timer
[u_opt1_b, f_opt1_b, exitflag1_b, output1_b] = fmincon(f_b, u_control0_1, [], [], [], [], lb, ub, [], options);
time1_b = toc; % End timer

tic; % Start timer
[u_opt2_b, f_opt2_b, exitflag2_b, output2_b] = fmincon(f_b, u_control0_2, [], [], [], [], lb, ub, [], options);
time2_b = toc; % End timer

%% Task 4
% For each given optimal control input, we run a new simulation again for
% each case that returns us plots.

state_a_1 = simulation_for_plots(u_opt1, x0);
state_a_2 = simulation_for_plots(u_opt2, x0);
state_b_1 = simulation_var_b_for_plots(u_opt1_b, x0);
state_b_2 = simulation_var_b_for_plots(u_opt2_b, x0);

function [V, Rho, Wr, r_sim, VSL_sim] = extract_data(state_hist, u_opt_vec)
    V = state_hist(1:6, :);             % Speed history [km/h]
    Rho = state_hist(7:12, :);          % Density history [veh/(km*lane)]
    Wr = state_hist(13, :);             % Queue length history [veh]

    r_control = u_opt_vec(1:20);
    VSL_control = u_opt_vec(21:end);

    r_sim = repelem(r_control, 6);
    VSL_sim = repelem(VSL_control, 6);
end

[V_a1, Rho_a1, Wr_a1, r_a1, VSL_a1] = extract_data(state_a_1, u_opt1);
[V_a2, Rho_a2, Wr_a2, r_a2, VSL_a2] = extract_data(state_a_2, u_opt2);
[V_b1, Rho_b1, Wr_b1, r_b1, VSL_b1] = extract_data(state_b_1, u_opt1_b);
[V_b2, Rho_b2, Wr_b2, r_b2, VSL_b2] = extract_data(state_b_2, u_opt2_b);



%% Plotting
% Task 3a Comparison (Different Starting Points, Original Inflow)
fprintf('Plotting Task 3a comparison (Start 1 vs Start 2)\n');
num_segments = 6;

segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:num_segments, 'UniformOutput', false);

sim_time_s = (1:121).*10;

% Speeds 3a
figure('Name', 'Task 3a Speeds Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_a1'); title('Speeds - Start 1 (r=0, VSL=60)'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_a2'); title('Speeds - Start 2 (r=1, VSL=120)'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3a
figure('Name', 'Task 3a Densities Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_a1'); title('Densities - Start 1 (r=0, VSL=60)'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_a2'); title('Densities - Start 2 (r=1, VSL=120)'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3a
figure('Name', 'Task 3a Queue Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1 (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2 (r=1, VSL=120)');
hold off;
title('Queue Length Comparison - Task 3a');
xlabel('Time [s]');
ylabel('Queue [veh]');
xlim([0 1210]);
legend('show');
grid on;

output_time_s = (1:120).*10;

% Inputs 3a
figure('Name', 'Task 3a Inputs Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Ramp Metering Rate (Applied)'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Variable Speed Limit (Applied)'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');


% Task 3b Comparison (Different Starting Points, Increased Inflow)
fprintf('Plotting Task 3b comparison (Start 1 vs Start 2)\n');

% Speeds 3b
figure('Name', 'Task 3b Speeds Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_b1'); title('Speeds - Start 1 (r=0, VSL=60), Increased Inflow'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_b2'); title('Speeds - Start 2 (r=1, VSL=120), Increased Inflow'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3b
figure('Name', 'Task 3b Densities Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_b1'); title('Densities - Start 1 (r=0, VSL=60), Increased Inflow'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_b2'); title('Densities - Start 2 (r=1, VSL=120), Increased Inflow'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3b
figure('Name', 'Task 3b Queue Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_b1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1 (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_b2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2 (r=1, VSL=120)');
hold off;
title('Queue Length Comparison - Task 3b (Increased Inflow)');
xlabel('Time [s]');
ylabel('Queue [veh]');
xlim([0 1210]);
legend('show');
grid on;

% Inputs 3b
figure('Name', 'Task 3b Inputs Comparison', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_b1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_b2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Ramp Metering Rate (Applied) - Increased Inflow'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_b1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_b2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Variable Speed Limit (Applied) - Increased Inflow'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');

%%
% Question 5
VSL_i_a = VSL_a1; %120x1
scaledVSL = params.alpha .* VSL_i_a;
expTerm_seg2 = params.v_f.*exp(-(1/params.a).*(Rho_a1(2,:) ./  params.rho_c).^params.a);
expTerm_seg3 = params.v_f.*exp(-(1/params.a).*(Rho_a1(3,:) ./  params.rho_c).^params.a);

V_i_k_seg2 = zeros(size(Rho_a1(2)));
V_i_k_seg3 = zeros(size(Rho_a1(3)));

for i = length(Rho_a1)-1
    V_i_k_seg2(i) = min([scaledVSL(i) expTerm_seg2(i)]);
end

figure()
plot(scaledVSL, 'DisplayName', '(1+\alpha) V_{SL,i}(k)');
hold on
plot(V_i_k_seg2, 'DisplayName', '(V_i(k)');
title('(1+\alpha)V_{SL,i}(k) vs. V_i(k)');
legend;
