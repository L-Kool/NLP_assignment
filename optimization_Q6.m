%% Optimization problem Task 6

% Importing parameters
params = ImportParameters();
sim_time_s = (0:120).*10;
output_time_s = (1:120).*10;

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


%% Setting up the optimization
clc
% Constraint on w_r
W_max_initial = (23 + params.E1 / 6);
W_max_current = W_max_initial;

% Defining the objective function
f_6 = @(u) simulation(u, x0, params, 1).totalTTS;

% Defining nonlinear constraint handle
nonlcon = @(u) T6_NonLinCon(u, x0, params, W_max_current); 

% Options for optimization algorithm using SQP
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...         
    'Display', 'final', ...          
    'MaxFunctionEvaluations', 50000, ... 
    'MaxIterations', 400, ...        % Default
    'OptimalityTolerance', 1e-6, ... % Default
    'StepTolerance', 1e-6, ...       % Default
    'ConstraintTolerance', 1e-6);    % Default


%% Running the optimization
maxAttempts = 200;
increaseCounter = 0;
decreaseCounter = 0;

tic;
for attempt = 1:maxAttempts

% Update W_max in handle
nonlcon = @(u) T6_NonLinCon(u, x0, params, W_max_current); 
[u_opt_t6, J_opt_t6, exitflag_t6, out_t6] = fmincon(f_6, u_control0_1, [], [], [], [], lb, ub, nonlcon, options);


% Check if optimization is feasible
    if exitflag_t6 > 0 || exitflag_t6 == 0

        % Optimization is feasible
        % Run the simulation to find w_r relative to optimal point found
            state_t6 = simulation(u_opt_t6, x0, params, 1).stateHist;

            % Find max w_r 
            max_Wr_achieved = max(state_t6(13,:));

            % If the error between the max w_r value and the W_max set is
            % less than the constraint tolerance
            if max_Wr_achieved > W_max_current + 1e-6

                % The constraints are violated by more than 1%
                if max_Wr_achieved > W_max_current * 1.01 
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
                break % Feasible solution found
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
time_t6_a = toc;

%% Start with initial guess 2
W_max_current_b = W_max_initial;


tic;
for attempt = 1:maxAttempts
nonlcon = @(u) T6_NonLinCon(u, x0, params, W_max_current_b); % Update W_max in handle

[u_opt_t6_b, J_opt_t6_b, exitflag_t6_b, out_t6_b] = fmincon(f_6, u_control0_2, [], [], [], [], lb, ub, nonlcon, options);

% Check if optimization is feasible
    if exitflag_t6_b > 0 || exitflag_t6_b == 0

        % Optimization is feasible
        % Run the simulation to find w_r relative to optimal point found
            state_t6_b = simulation(u_opt_t6_b, x0, params, 1).stateHist;

            % Find max w_r 
            max_Wr_achieved_b = max(state_t6_b(13,:));

            % If the error between the max w_r value and the W_max set is
            % less than the constraint tolerance
            if max_Wr_achieved_b > W_max_current_b + 1e-6

                % The constraint is violated by more than 1%
                if max_Wr_achieved_b > W_max_current_b * 1.01 
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
time_t6_b = toc;

%% Post-processing data

simResults6 = simulation(u_opt_t6, x0, params, 1);
state_t6 = simResults6.stateHist;
output_t6 = simResults6.outputHist;


[V_t6, Rho_t6, Wr_t6, r_t6, VSL_t6] = ExtractData(state_t6, u_opt_t6);
cumTTS_t6 = sum(output_t6);

simResults6_b = simulation(u_opt_t6_b, x0, params, 1);
state_t6_2 = simResults6_b.stateHist;
output_t6_2 = simResults6_b.outputHist;
cumTTS_t6_2 = simResults6_b.totalTTS;

[V_t6_2, Rho_t6_2, Wr_t6_2, r_t6_2, VSL_t6_2] = ExtractData(state_t6_2, u_opt_t6_b);

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
