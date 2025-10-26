%% Optimization problem Question 5
close all;

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

%% Question 5 Plotting part 1
vsl_term = (1 + params.alpha) .* VSL_a;

rho_k_seg2 = Rho_a(2, 2:end); 
rho_k_seg3 = Rho_a(3, 2:end); 

exp_term_seg2 = params.v_f .* exp(-(1/params.a) .* (rho_k_seg2 ./ params.rho_c).^params.a);
exp_term_seg3 = params.v_f .* exp(-(1/params.a) .* (rho_k_seg3 ./ params.rho_c).^params.a);

V_i_k_seg2 = min(vsl_term', exp_term_seg2);
V_i_k_seg3 = min(vsl_term', exp_term_seg3);

% Plotting (1+alpha)*VSL vs V_i(k)
close all
figure('Name', 'Task 5: VSL vs. Desired Speed');
subplot(2,1,1);
stairs(output_time_s, vsl_term, 'r--', 'LineWidth', 2, 'DisplayName', '(1+\alpha)V_{SL}(k)');
hold on;
stairs(output_time_s, V_i_k_seg2, 'b-', 'LineWidth', 2, 'DisplayName', 'V_i(k) (Desired Speed)');
title('Segment 2: Desired Speed vs. VSL Term', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('[km/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); 
grid on;
hold off;

subplot(2,1,2);
stairs(output_time_s, vsl_term, 'r--', 'LineWidth', 2, 'DisplayName', '(1+\alpha)V_{SL}(k)');
hold on;
stairs(output_time_s, V_i_k_seg3, 'b-', 'LineWidth', 2, 'DisplayName', 'V_i(k) (Desired Speed)');
title('Segment 3: Desired Speed vs. VSL Term', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('[km/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); 
grid on;
hold off;

%% Question 5 Plotting part 2

r_k_vec = r_a; 
wr_k_vec = Wr_a(1, 1:120)';
rho5_k_vec = Rho_a(5, 1:120)';

q_r5_term1_allowed = r_k_vec .* params.C_r;
q_r5_term2_demand = params.D_r + wr_k_vec ./ params.T;
q_r5_term3_capacity = params.C_r .* (params.rho_m - rho5_k_vec) ./ (params.rho_m - params.rho_c);

q_r5_k = min(min(q_r5_term1_allowed, q_r5_term2_demand), q_r5_term3_capacity);

% Plotting r(k)*C_r vs q_r,5(k)
close all
figure('Name', 'Task 5: Ramp Metering Analysis');
stairs(output_time_s, q_r5_term1_allowed, 'r--', 'LineWidth', 2, 'DisplayName', 'r(k) * C_r (Allowed Flow)');
hold on;
stairs(output_time_s, q_r5_k, 'b-', 'LineWidth', 2, 'DisplayName', 'q_{r,5}(k) (Actual Flow)');
ylim([0 2100]);
title('Segment 5: Allowed Ramp Flow vs. Actual Ramp Flow', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('Flow [veh/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); grid on;
hold off;

%% Determining weights
% Extract cost terms
costTerms3 = CostFunctionQ5(u_opt_a, x0, params, 0, 0, 0);
J_TTS_3 = costTerms3.J_TTS;
J_vsl_3 = costTerms3.J_vsl;
J_ramp_3 = costTerms3.J_ramp;
W_vsl = J_TTS_3 / J_vsl_3;
W_ramp = J_TTS_3 / J_ramp_3;
fprintf('\nWeight for VSL term, W_{vsl} = %d\n', W_vsl);
fprintf('Weight for VSL term, W_{ramp} = %d\n', W_ramp);

%% Multi-objective optimisation

% Objective function
costQ5 = @(u) CostFunctionQ5(u, x0, params, 0, W_vsl, W_ramp).cost;

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...          % Use Sequential Quadratic Programming
    'Display', 'iter', ...           % Show output for each iteration
    'MaxFunctionEvaluations', 50000, ... 
    'MaxIterations', 600, ...        % Put to 500 for Q5
    'OptimalityTolerance', 1e-4, ... % Reduced from 1e-6
    'StepTolerance', 1e-4, ...       % Reduced from 1e-6
    'ConstraintTolerance', 1e-6);    % Default

% Run optimization_3_a_b.m before to have u_opt_a in workspace for warm start
tic;
[u_opt_5, f_opt_5, exitflag_5, output_5] = fmincon(costQ5, u_opt_a, [], [], [], [], lb, ub, [], options);
time_5 = toc; 

fprintf('Task 5 optimization complete. Time: %.2f s\n', time_5);
fprintf('Optimal combined cost: %.4f\n', f_opt_5);

%% Post-processing
% Obtaining optimal history
stateHist_opt5 = simulation(u_opt_5, x0, params, 0).stateHist;
costQ5_opt = CostFunctionQ5(u_opt_5, x0, params, 0, W_vsl, W_ramp);
fprintf('\nCost TTS (opt 5): %e\n', costQ5_opt.J_TTS);
fprintf('Cost vsl (opt 5): %e\n', costQ5_opt.J_vsl);
fprintf('Cost ramp (opt 5): %e\n', costQ5_opt.J_ramp);
fprintf('Weighted Cost vsl (opt 5): %e\n', costQ5_opt.J_vsl * W_vsl);
fprintf('Weighted Cost ramp (opt 5): %e\n', costQ5_opt.J_ramp * W_ramp);

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
ylabel('[km/h]', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylim([50 130]);
legend('show', 'FontSize', 14); 
grid on;
hold off;


%% Question 5 Plotting optimal results
vsl_term = (1 + params.alpha) .* VSL_opt_5;

rho_k_seg2 = Rho_opt_5(2, 2:end); 
rho_k_seg3 = Rho_opt_5(3, 2:end); 

exp_term_seg2 = params.v_f .* exp(-(1/params.a) .* (rho_k_seg2 ./ params.rho_c).^params.a);
exp_term_seg3 = params.v_f .* exp(-(1/params.a) .* (rho_k_seg3 ./ params.rho_c).^params.a);

V_i_k_seg2 = min(vsl_term', exp_term_seg2);
V_i_k_seg3 = min(vsl_term', exp_term_seg3);

% Plotting (1+alpha)*VSL vs V_i(k)
close all
figure('Name', 'Task 5: VSL vs. Desired Speed');
subplot(2,1,1);
stairs(output_time_s, vsl_term, 'r-', 'LineWidth', 2, 'DisplayName', '(1+\alpha)V_{SL}(k)');
hold on;
stairs(output_time_s, V_i_k_seg2, 'b--', 'LineWidth', 2, 'DisplayName', 'V_i(k) (Desired Speed)');
title('Segment 2: Desired Speed vs. VSL Term with multi-object optimization', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('[km/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); 
grid on;
hold off;

subplot(2,1,2);
stairs(output_time_s, vsl_term, 'r-', 'LineWidth', 2, 'DisplayName', '(1+\alpha)V_{SL}(k)');
hold on;
stairs(output_time_s, V_i_k_seg3, 'b--', 'LineWidth', 2, 'DisplayName', 'V_i(k) (Desired Speed)');
title('Segment 3: Desired Speed vs. VSL Term with multi-objective optimization', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('[km/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); 
grid on;
hold off;

r_k_vec = r_opt_5; 
wr_k_vec = Wr_opt_5(1, 1:120)';
rho5_k_vec = Rho_opt_5(5, 1:120)';

q_r5_term1_allowed = r_k_vec .* params.C_r;
q_r5_term2_demand = params.D_r + wr_k_vec ./ params.T;
q_r5_term3_capacity = params.C_r .* (params.rho_m - rho5_k_vec) ./ (params.rho_m - params.rho_c);

q_r5_k = min(min(q_r5_term1_allowed, q_r5_term2_demand), q_r5_term3_capacity);

% Plotting r(k)*C_r vs q_r,5(k)
figure('Name', 'Task 5: Ramp Metering Analysis with optimized inputs');
stairs(output_time_s, q_r5_term1_allowed, 'r-', 'LineWidth', 2, 'DisplayName', 'r(k) * C_r (Allowed Flow)');
hold on;
stairs(output_time_s, q_r5_k, 'b--', 'LineWidth', 2, 'DisplayName', 'q_{r,5}(k) (Actual Flow)');
ylim([0 2100]);
title('Segment 5: Allowed Ramp Flow vs. Actual Ramp Flow with multi-objective optimization', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 14); 
ylabel('Flow [veh/h]', 'FontSize', 14);
legend('show', 'FontSize', 14); grid on;
hold off;