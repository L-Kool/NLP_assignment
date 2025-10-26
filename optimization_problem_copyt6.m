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

sim_time_s = (0:120).*10;
output_time_s = (1:120).*10;
%%

% Sim steps
Nc = 20;

% Bounds
lb_r = zeros(Nc, 1);
ub_r = ones(Nc, 1);
lb_vsl = 60 * ones(Nc, 1);
ub_vsl = 120 * ones(Nc, 1);
lb = [lb_r; lb_vsl]; % Lower bounds for u_control
ub = [ub_r; ub_vsl]; % Upper bounds for u_control

% Initial conditions
% 1. Ramp closed (r=0), Min speed limit (VSL=60)
u_control0_1 = [zeros(Nc, 1); 60 * ones(Nc, 1)];

% 2. Ramp fully open (r=1), Max speed limit (VSL=120) - "No Control"
u_control0_2 = [ones(Nc, 1); 120 * ones(Nc, 1)];

% Initial conditions (from the sheet)
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x0 = [v_0 ; rho_0 ; w_r_0];


% objective function
f = @(u) simulation(u, x0);

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...         % Use SQP
    'Display', 'final', ...           % default
    'MaxFunctionEvaluations', 50000, ... % Increase limit
    'MaxIterations', 400, ...        % Default
    'OptimalityTolerance', 1e-6, ... % Default
    'StepTolerance', 1e-6, ...       % Default
    'ConstraintTolerance', 1e-6);    % Default
%%
tic; % Start timer
[u_opt1, f_opt1, exitflag1, output1] = fmincon(f, u_control0_1, [], [], [], [], lb, ub, [], options);
time1 = toc; % End timer

tic; % Start timer
[u_opt2, f_opt2, exitflag2, output2] = fmincon(f, u_control0_2, [], [], [], [], lb, ub, [], options);
time2 = toc; % End timer

%% Task 3b

% objective function
f_b = @(u) simulation_var_b(u, x0);
%% 
tic; % Start timer
[u_opt1_b, f_opt1_b, exitflag1_b, output1_b] = fmincon(f_b, u_control0_1, [], [], [], [], lb, ub, [], options);
time1_b = toc; % End timer

tic; % Start timer
[u_opt2_b, f_opt2_b, exitflag2_b, output2_b] = fmincon(f_b, u_control0_2, [], [], [], [], lb, ub, [], options);
time2_b = toc; % End timer

%% Task 4
% For each given optimal control input, we run a new simulation again for
% each case that returns us plots.

[state_a_1, output_a_1] = simulation_for_plots(u_opt1, x0);
[state_a_2, output_a_2] = simulation_for_plots(u_opt2, x0);
[state_b_1, output_b_1] = simulation_var_b_for_plots(u_opt1_b, x0);
[state_b_2, output_b_2] = simulation_var_b_for_plots(u_opt2_b, x0);

cumTTS_a_1 = sum(output_a_1);
cumTTS_a_2 = sum(output_a_2);
cumTTS_b_1 = sum(output_b_1);
cumTTS_b_2 = sum(output_b_2);

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


% Speeds 3a
figure('Name', 'Speeds Comparison - 3a', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_a1'); title('Speeds - Start 1 (r=0, VSL=60)'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_a2'); title('Speeds - Start 2 (r=1, VSL=120)'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3a
figure('Name', 'Densities Comparison - 3a', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_a1'); title('Densities - Start 1 (r=0, VSL=60)'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_a2'); title('Densities - Start 2 (r=1, VSL=120)'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3a
figure('Name', 'Queue Comparison - 3a', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1 (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2 (r=1, VSL=120)');
hold off;
title('Queue Length Comparison - 3a');
xlabel('Time [s]');
ylabel('Queue [veh]');
xlim([0 1210]);
legend('show');
grid on;



% Inputs 3a
figure('Name', 'Inputs Comparison - 3a', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Applied Ramp Metering Rate'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_a1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_a2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Applied Variable Speed Limit'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');

% Outputs 3a
figure('Name', 'Output Comparison - 3a', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, output_a_1); hold on; plot(sim_time_s, output_a_2); title('TTS over time Comparison'); ylabel('[veh*h]'); xlim([0 1210]); grid on; legend('Start 1', 'Start 2');

%% Plots for 3b
% Task 3b Comparison (Different Starting Points, Increased Inflow)
fprintf('Plotting Task 3b comparison (Start 1 vs Start 2)\n');

% Speeds 3b
figure('Name', 'Speeds Comparison - 3b (Increased Inflow)', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_b1'); title('Speeds - Start 1 (r=0, VSL=60)'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_b2'); title('Speeds - Start 2 (r=1, VSL=120)'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3b
figure('Name', 'Densities Comparison - 3b (Increased Inflow)', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_b1'); title('Densities - Start 1 (r=0, VSL=60)'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_b2'); title('Densities - Start 2 (r=1, VSL=120)'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3b
figure('Name', 'Queue Comparison  - 3b (Increased Inflow)', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
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
figure('Name', 'Inputs Comparison  - 3b (Increased Inflow)', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_b1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_b2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Ramp Metering Rate (Applied) - Increased Inflow'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_b1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_b2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Variable Speed Limit (Applied) - Increased Inflow'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');

% Outputs 3b
figure('Name', 'Output Comparison - 3b', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, output_b_1); hold on; plot(sim_time_s, output_b_2); title('TTS over time Comparison'); ylabel('[veh*h]'); xlim([0 1210]); grid on; legend('Start 1', 'Start 2');

%% Task. 6
clc
% Constraint on w_r
W_max_initial = (23 + E1 / 6);
W_max_current = W_max_initial;

% Define Nonlinear Constraint Handle
nonlcon = @(u) T6_NonLinCon(u, x0, W_max_current); % Update W_max in handle

% Run Optimization Loop (Handles feasibility adjustments)
maxAttempts = 200;
increaseCounter = 0;
decreaseCounter = 0;

for attempt = 1:maxAttempts
nonlcon = @(u) T6_NonLinCon(u, x0, W_max_current); % Update W_max in handle
[u_opt_t6, J_opt_t6, exitflag_t6, out_t6] = fmincon(f_b, u_control0_1, [], [], [], [], lb, ub, nonlcon, options);


% Check if optimization is feasible
    if exitflag_t6 > 0 || exitflag_t6 == 0
        % Optimization is feasible
        % Run the simulation to find w_r relative to optimal point found
            [state_t6, ~] = simulation_var_b_for_plots(u_opt_t6, x0);
            % Find max w_r of such state
            max_Wr_achieved = max(state_t6(13,:));
            % If the error between the max w_r value and the W_max set is
            % less than the constraint tolerance
            if max_Wr_achieved > W_max_current + 1e-6
                % The optimizer reported an optimum point that violates the
                % constraints
                if max_Wr_achieved > W_max_current * 1.01 % If violation > 1%
                    % The constraint was violated. Increase w_max
                    W_max_current = W_max_current * 1.10;
                    increaseCounter = increaseCounter + 1;
                else
                    % Solution acceptable. Minor constraint violation
                    % likely due to tolerances.
                    break
                end
            elseif W_max_current - max_Wr_achieved > 0.1 * W_max_current
                % Constraint not active (more than 10% slack)
                W_max_current = W_max_current * 0.90;
                decreaseCounter = decreaseCounter + 1;
            else
                break % Solution feasible
            end
            if abs(max_Wr_achieved - W_max_current) < 1e-6
                % The constraint was active, exit the loop
                break
            else % The constraint was not active, reduce the W_max of 10%
                W_max_current = W_max_current * 0.9;
                decreaseCounter = decreaseCounter + 1;
            end
        break
    elseif exitflag_t6 == -2
        W_max_current = W_max_current * 1.1;
        increaseCounter = increaseCounter + 1;
    end
end
%% Start with initial guess 2
W_max_current_b = W_max_initial;
for attempt = 1:maxAttempts
nonlcon = @(u) T6_NonLinCon(u, x0, W_max_current_b); % Update W_max in handle
tic;
[u_opt_t6_b, J_opt_t6_b, exitflag_t6_b, out_t6_b] = fmincon(f_b, u_control0_2, [], [], [], [], lb, ub, nonlcon, options);
time_t6_b = toc;

% Check if optimization is feasible
    if exitflag_t6_b > 0 || exitflag_t6_b == 0
        % Optimization is feasible
        % Run the simulation to find w_r relative to optimal point found
            [state_t6_b, ~] = simulation_var_b_for_plots(u_opt_t6_b, x0);
            % Find max w_r of such state
            max_Wr_achieved_b = max(state_t6_b(13,:));
            % If the error between the max w_r value and the W_max set is
            % less than the constraint tolerance
            if max_Wr_achieved_b > W_max_current_b + 1e-6
                % The optimizer reported an optimum point that violates the
                % constraints
                if max_Wr_achieved_b > W_max_current_b * 1.01 % If violation > 1%
                    % The constraint was violated. Increase w_max
                    W_max_current_b = W_max_current_b * 1.10;
                else
                    % Solution acceptable. Minor constraint violation
                    % likely due to tolerances.
                    break
                end
            elseif W_max_current_b - max_Wr_achieved_b > 0.1 * W_max_current_b
                % Constraint not active (more than 10% slack)
                W_max_current_b = W_max_current_b * 0.90;
            else
                break % Solution feasible
            end
            if abs(max_Wr_achieved_b - W_max_current_b) < 1e-6
                % The constraint was active, exit the loop
                break
            else % The constraint was not active, reduce the W_max of 10%
                W_max_current_b = W_max_current_b * 0.9;
            end
        break
    elseif exitflag_t6_b == -2
        W_max_current_b = W_max_current_b * 1.1;
    end
end
%% Optimal solution plot

[state_t6, output_t6] = simulation_var_b_for_plots(u_opt_t6, x0);
[V_t6, Rho_t6, Wr_t6, r_t6, VSL_t6] = extract_data(state_t6, u_opt_t6);
cumTTS_t6 = sum(output_t6);

[state_t6_2, output_t6_2] = simulation_var_b_for_plots(u_opt_t6_b, x0);
[V_t6_2, Rho_t6_2, Wr_t6_2, r_t6_2, VSL_t6_2] = extract_data(state_t6_2, u_opt_t6_b);
cumTTS_t6_2 = sum(output_t6_2);

%% Plots T6
close all
num_segments = 6;

W_max_current_plot = W_max_current * ones(121,1);
W_max_current_b_plot = W_max_current_b * ones(121,1);
W_max_initial_plot = W_max_initial * ones(121,1);


segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:num_segments, 'UniformOutput', false);

% Speeds 3b
figure('Name', 'Speeds Comparison T6', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, V_t6'); title('Speeds - Start 1 (r=0, VSL=60)'); ylabel('[km/h]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');
subplot(2,1,2); plot(sim_time_s, V_t6_2'); title('Speeds - Start 2 (r=1, VSL=120)'); ylabel('[km/h]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside');

% Densities 3b
figure('Name', 'Densities Comparison T6)', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1); plot(sim_time_s, Rho_t6'); title('Densities - Start 1 (r=0, VSL=60)'); ylabel('[veh/km/lane]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;
subplot(2,1,2); plot(sim_time_s, Rho_t6_2'); title('Densities - Start 2 (r=1, VSL=120)'); ylabel('[veh/km/lane]'); xlabel('Time [s]'); xlim([0 1210]); grid on; legend(segment_legends, 'Location','eastoutside'); ylim([0, params.rho_m * 1.1]); hold on; plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); hold off;

% Queue 3b
figure('Name', 'Queue Comparison T6', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, Wr_t6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1 (r=0, VSL=60)');
hold on;
plot(sim_time_s, Wr_t6_2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2 (r=1, VSL=120)');
hold on;
plot(sim_time_s, W_max_current_plot, 'k:', 'LineWidth', 1.5, 'DisplayName', 'W_{max(1)}');
hold on;
plot(sim_time_s, W_max_current_b_plot, 'm:', 'LineWidth', 1.5, 'DisplayName', 'W_{max(2)}');
hold on;
plot(sim_time_s, W_max_initial_plot, 'g:', 'LineWidth', 1.5, 'DisplayName', 'W_{max} (initial)');
hold off;
title('Queue Length Comparison');
xlabel('Time [s]');
ylabel('Queue [veh]');
xlim([0 1210]);
legend('show');
grid on;

% Inputs 3b
figure('Name', 'Inputs Comparison  T6', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
subplot(2,1,1);
stairs(output_time_s, r_t6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, r_t6_2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Ramp Metering Rate (Applied)'); ylabel('Rate'); ylim([-0.1 1.1]); xlim([0 1200]); grid on; legend('show');
subplot(2,1,2);
stairs(output_time_s, VSL_t6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Start 1'); hold on;
stairs(output_time_s, VSL_t6_2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Start 2'); hold off;
title('Variable Speed Limit (Applied)'); ylabel('[km/h]'); xlabel('Time [s]'); ylim([50 130]); xlim([0 1200]); grid on; legend('show');

% Outputs 3b
figure('Name', 'Output Comparison T6', 'Units', 'pixels', 'Position', [100, 100, 1600, 800]);
plot(sim_time_s, output_t6); hold on; plot(sim_time_s, output_t6_2); title('TTS over time Comparison'); ylabel('[veh*h]'); xlim([0 1210]); grid on; legend('Start 1', 'Start 2');
