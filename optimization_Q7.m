%% Optimization problem Task 7
close all;

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
    'PopulationSize', 200, ...  
    'MaxGenerations', 100, ...  
    'CrossoverFraction', 0.8, ...
    'PlotFcn', @gaplotbestf, ... 
    'UseParallel', false);       

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
% Output of optimization is set of integers which need to be mapped to
% their respective control input values

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
simResults7 = simulation(u_opt_7, x0, params, 1);
stateHist_opt7 = simResults7.stateHist;
outputHist_opt7 = simResults7.outputHist;
[V_opt_7, Rho_opt_7, Wr_opt_7, r_opt_7, VSL_opt_7] = extract_data(stateHist_opt7, u_opt_7);

%% Plotting results (Task 8)
time_axis_s = (1:120) .* 10;
sim_time_s = (1:121) .* 10;
segment_legends = arrayfun(@(i) sprintf('Seg %d', i), 1:6, 'UniformOutput', false);

% Optimal discrete ramp metering rate compared to ones from 3.b
close all
figure('Name', 'Task 7: Optimal Discrete Control Inputs');
subplot(2,1,1);
stairs(time_axis_s, r_opt_7, 'b-', 'LineWidth', 2);
hold on;
stairs(time_axis_s, r_b, 'r--', 'LineWidth', 2);
title('Optimal Discrete vs. Continuous Ramp Metering Rate', 'FontSize', 14);
ylabel('Rate', 'FontSize', 14);
ylim([-0.1 1.5]); 
yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0]);
legend('Discrete control input (Task 7)', 'No control case (Task 3.b)', 'FontSize', 14);
grid on;

% Optimal discrete VSL compared to ones from 3.b
subplot(2,1,2);
stairs(time_axis_s, VSL_opt_7, 'b-', 'LineWidth', 2);
hold on;
stairs(time_axis_s, VSL_b, 'r--', 'LineWidth', 2);
title('Optimal Discrete vs. Continuous Variable Speed Limit', 'FontSize', 14);
ylabel('[km/h]', 'FontSize', 14); 
xlabel('Time [s]', 'FontSize', 14);
ylim([50 150]); 
yticks([60, 80, 100, 120]);
legend('Discrete control input (Task 7)', 'No control case (Task 3.b)', 'FontSize', 14);
grid on;

%% Plotting states and compare to ones from 3.b
close all

% Speeds
figure('Name', 'Task 7: Optimal States (Speeds)');
subplot(2,1,1);
plot(sim_time_s, V_opt_7'); 
title('Speeds for the different segments using discrete control inputs', 'FontSize', 14); 
ylabel('[km/h]', 'FontSize', 14); 
xlim([0 1210]);
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);

subplot(2,1,2)
plot(sim_time_s, V_b);
title('Speeds for the different segments in no control case', 'FontSize', 14); 
ylabel('[km/h]','FontSize', 14); 
xlabel('Time [s]', 'FontSize', 14);
xlim([0 1210]);
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);


% Densities
figure('Name', 'Task 7: Optimal States (Densities)');
subplot(2,1,1);
plot(sim_time_s, Rho_opt_7'); 
title('Densities for the different segments using discrete control inputs', 'FontSize', 14); 
ylabel('[veh/km/lane]', 'FontSize', 14);
xlim([0 1210]); 
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);

subplot(2,1,2);
plot(sim_time_s, Rho_b);
title('Densities for the different segments in no control case', 'FontSize', 14); 
ylabel('[veh/km/lane]', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14);
xlim([0 1210]); 
grid on; 
legend(segment_legends, 'Location','eastoutside', 'FontSize', 14);


% Queue length
figure('Name', 'Task 7: Optimal States (Queue)');
plot(sim_time_s, Wr_opt_7, 'b-', 'LineWidth', 2);
hold on
plot(sim_time_s, Wr_b, 'LineWidth', 2);
title('Queue Length for the different segments with discrete inputs vs. no contol case', 'FontSize', 14); 
xlabel('Time [s]', 'FontSize', 14); 
ylabel('Queue [veh]', 'FontSize', 14);
xlim([0 1210]);
legend('Discrete control inputs', 'No control case', 'FontSize', 14);
grid on;

%% Plotting instantaneous and cumulative TTS values
close all
instTTS_opt7 = outputHist_opt7;
cumulativeTTS_opt7 = cumsum(instTTS_opt7);

instTTS_no_control = output1_b;
cumulativeTTS_no_control = cumsum(instTTS_no_control);

figure()
subplot(2,1,1)
plot(sim_time_s, instTTS_opt7, 'LineWidth', 2);
hold on;
plot(sim_time_s, instTTS_no_control, 'LineWidth', 2);
xlim([0 1210]);
ylabel('Total Time Spent [veh h]', 'FontSize', 14);
legend('With discrete inputs', 'No control case', 'FontSize', 14);
title('Instantaneous TTS using discrete control inputs vs. no control case', 'FontSize', 14);


subplot(2,1,2);
plot(sim_time_s, cumulativeTTS_opt7, 'LineWidth', 2);
hold on;
plot(sim_time_s, cumulativeTTS_no_control, 'LineWidth', 2);
xlim([0 1210]);
ylabel('Total Time Spent [veh h]', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
legend('With discrete inputs', 'No control case', 'FontSize', 14);
title('Cumulative TTS using discrete control inputs vs. no control case', 'FontSize', 14);