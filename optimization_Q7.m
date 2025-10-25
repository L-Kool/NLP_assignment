%% Optimization problem Task 7
close all; clear;

% Import parameters
params = ImportParameters();

% Initial state conditions 
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x0 = [v_0 ; rho_0 ; w_r_0];

% Defining settings for genetic algorithm
Nc = 20;        % Number of control steps      
nvars = 2 * Nc; % Number of control variables

% Defining lower and upper bounds (1-4)
lb = ones(nvars, 1);
ub = 4 * ones(nvars, 1);

% Integer Constraints: All 40 variables are integers
IntCon = 1:nvars;

% Objective Function:
cost_Q7 = @(x) CostFunctionQ7(x, x0, params);

% Defining options for the genetic algorithm
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 200, ...  % Increase population for a better search
    'MaxGenerations', 100, ...  % (Default is 100 * nvars, which is too long)
    'CrossoverFraction', 0.8, ...
    'PlotFcn', @gaplotbestf, ... % Show a plot of the best cost
    'UseParallel', false);       % Set to false if you don't have the toolbox

%% Running the optimization
tic;
close all
[x_opt_7, f_opt_7, exitflag_7, output_7] = ga(cost_Q7, nvars, [], [], [], [], ...
    lb, ub, ...
    [], ...       
    IntCon, ...  
    options);
time_7 = toc;

fprintf('Task 7 optimization complete. Time: %.2f s\n', time_7);
fprintf('Optimal discrete TTS cost: %.4f\n', f_opt_7);

%% Post-processing and plotting

% The output 'x_opt_7' is a vector of integers. We must translate it
% back to the real control values to get the final plots.

% Mappings of r and VSL
r_map = [0.2, 0.4, 0.6, 0.8];
vsl_map = [60, 80, 100, 120];

% Extracting optimal integers for r and VSL
r_opt_indices = x_opt_7(1:Nc);
vsl_opt_indices = x_opt_7(Nc+1:end);

% Mapping integers to values in sets
u_r_opt_7 = r_map(r_opt_indices)';
u_vsl_opt_7 = vsl_map(vsl_opt_indices)';

% Optimal control vector
u_opt_7 = [u_r_opt_7; u_vsl_opt_7]; 

% Running the simulation to extract states
stateHist_opt7 = simulation(u_opt_7, x0, params, 1).stateHist;
[V_opt_7, Rho_opt_7, Wr_opt_7, r_opt_7, VSL_opt_7] = extract_data(stateHist_opt7, u_opt_7);

%% Plotting results (Task 8)
time_axis_s = (1:120) .* 10;
sim_time_s = (1:121) .* 10;
segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:6, 'UniformOutput', false);

% Optimal discrete ramp metering rate
close all
figure('Name', 'Task 7: Optimal Discrete Control Inputs');
subplot(2,1,1);
stairs(time_axis_s, r_opt_7, 'b-', 'LineWidth', 2);
title('Optimal Discrete Ramp Metering Rate');
ylabel('Rate');
ylim([-0.1 1.1]); % Set Y-axis to see discrete levels clearly
yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0]);
grid on;

% Optimal VSL
subplot(2,1,2);
stairs(time_axis_s, VSL_opt_7, 'r-', 'LineWidth', 2);
title('Optimal Discrete Variable Speed Limit');
ylabel('[km/h]'); xlabel('Time [s]');
ylim([50 130]); % Set Y-axis to see discrete levels clearly
yticks([60, 80, 100, 120]);
grid on;

%% Plotting states
close all
% Speeds
figure('Name', 'Task 7: Optimal States (Speeds)');
plot(sim_time_s, V_opt_7'); 
title('Speeds for the different segments', 'FontSize', 14); 
ylabel('[km/h]'); xlabel('Time [s]', 'FontSize', 14);
xlim([0 1210]);
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);

% Densities
figure('Name', 'Task 7: Optimal States (Densities)');
plot(sim_time_s, Rho_opt_7'); 
title('Densities for the different segments', 'FontSize', 14); 
ylabel('[veh/km/lane]'); xlabel('Time [s]', 'FontSize', 14);
xlim([0 1210]); 
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);

% Queue length
figure('Name', 'Task 7: Optimal States (Queue)');
plot(sim_time_s, Wr_opt_7, 'b-', 'LineWidth', 2);
title('Queue Length for the different segments'); 
xlabel('Time [s]'); ylabel('Queue [veh]');
xlim([0 1210]);
grid on;

%% TODO: ADD INSTANTANEOUS AND CUMULATIVE TTS VALUES