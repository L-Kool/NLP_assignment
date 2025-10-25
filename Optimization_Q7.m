%% Optimization problem Task 7
close all; clear; clc;

% Import parameters
params = ImportParameters():

% Initial state conditions 
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x0 = [v_0 ; rho_0 ; w_r_0];

% --- 2. Set up the Genetic Algorithm ---
Nc = 20;            % 20 control intervals
nvars = 2 * Nc;     % 40 variables in total (20 for r, 20 for VSL)

% Bounds: All variables are integers from 1 to 4
lb = ones(nvars, 1);
ub = 4 * ones(nvars, 1);

% Integer Constraints: All 40 variables are integers
IntCon = 1:nvars;

% Objective Function:
% Pass x0 and params to the cost function using an anonymous function
obj_fcn_Q7 = @(x) CostFunctionQ7(x, x0, params);

% Options
% Note: 'ga' can be slow. 'UseParallel' helps immensely if you have
% the Parallel Computing Toolbox.
fprintf('Setting GA options...\n');
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 200, ...  % Increase population for a better search
    'MaxGenerations', 100, ...  % (Default is 100 * nvars, which is too long)
    'CrossoverFraction', 0.8, ...
    'PlotFcn', @gaplotbestf, ... % Show a plot of the best cost
    'UseParallel', true);       % Set to false if you don't have the toolbox

% --- 3. Run the Optimization ---
fprintf('Starting GA optimization (this may take a while)...\n');
tic;
[x_opt_7, f_opt_7, exitflag_7, output_7] = ga(obj_fcn_Q7, ...
    nvars, ...    % Number of variables
    [], [], ...   % No linear inequality constraints
    [], [], ...   % No linear equality constraints
    lb, ub, ...   % Bounds (1 to 4)
    [], ...       % No nonlinear constraints
    IntCon, ...   % Integer variables
    options);
time_7 = toc;

fprintf('Task 7 optimization complete. Time: %.2f s\n', time_7);
fprintf('Optimal discrete TTS cost: %.4f\n', f_opt_7);

%% --- 4. Post-processing and Plotting ---

% The output 'x_opt_7' is a vector of integers. We must translate it
% back to the real control values to get the final plots.

fprintf('Translating optimal solution and running final simulation...\n');
r_map = [0.2, 0.4, 0.6, 0.8];
vsl_map = [60, 80, 100, 120];

x_r_indices_opt = x_opt_7(1:Nc);
x_vsl_indices_opt = x_opt_7(Nc+1:end);

u_r_opt_7 = r_map(x_r_indices_opt)';
u_vsl_opt_7 = vsl_map(x_vsl_indices_opt)';

u_opt_7 = [u_r_opt_7; u_vsl_opt_7]; % This is the final 40x1 control vector

% Run the final simulation using simType = 1 (increased inflow)
[stateHist_opt7, outputHist_opt7] = simulation(u_opt_7, x0, params, 1);
[V_opt_7, Rho_opt_7, Wr_opt_7, r_opt_7, VSL_opt_7] = extract_data(stateHist_opt7, u_opt_7);

% --- Plot the results (Task 8) ---
fprintf('Plotting Task 7/8 results...\n');
time_axis_s = (1:120) .* 10;
sim_time_s = (1:121) .* 10;
segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:6, 'UniformOutput', false);

% Plot optimal discrete inputs
figure('Name', 'Task 7: Optimal Discrete Control Inputs');
subplot(2,1,1);
stairs(time_axis_s, r_opt_7, 'b-', 'LineWidth', 2);
title('Optimal Discrete Ramp Metering Rate (Task 7)');
ylabel('Rate');
ylim([-0.1 1.1]); % Set Y-axis to see discrete levels clearly
yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0]);
grid on;

subplot(2,1,2);
stairs(time_axis_s, VSL_opt_7, 'r-', 'LineWidth', 2);
title('Optimal Discrete Variable Speed Limit (Task 7)');
ylabel('[km/h]'); xlabel('Time [s]');
ylim([50 130]); % Set Y-axis to see discrete levels clearly
yticks([60, 80, 100, 120]);
grid on;

% Plot corresponding states (Speeds, Densities, Queue)
figure('Name', 'Task 7: Optimal States (Speeds)');
plot(sim_time_s, V_opt_7'); 
title('Speeds - Task 7 Optimal'); 
ylabel('[km/h]'); xlabel('Time [s]');
xlim([0 1210]); grid on; 
legend(segment_legends, 'Location','eastoutside');

figure('Name', 'Task 7: Optimal States (Densities)');
plot(sim_time_s, Rho_opt_7'); 
title('Densities - Task 7 Optimal'); 
ylabel('[veh/km/lane]'); xlabel('Time [s]');
xlim([0 1210]); grid on; 
legend(segment_legends, 'Location','eastoutside');
ylim([0, params.rho_m * 1.1]);
hold on; 
plot(sim_time_s([1 end]), [params.rho_c params.rho_c], 'k--', 'DisplayName','\rho_c'); 
plot(sim_time_s([1 end]), [params.rho_m params.rho_m], 'r:', 'DisplayName','\rho_m'); 
hold off;

figure('Name', 'Task 7: Optimal States (Queue)');
plot(sim_time_s, Wr_opt_7, 'b-', 'LineWidth', 2);
title('Queue Length - Task 7 Optimal');
xlabel('Time [s]'); ylabel('Queue [veh]');
xlim([0 1210]);
grid on;