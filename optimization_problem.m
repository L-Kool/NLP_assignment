% Optimization problem
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
% Guess 1: Ramp closed (r=0), Min speed limit (VSL=60)
u_control0_1 = [zeros(Nc, 1); 60 * ones(Nc, 1)];

% Guess 2: Ramp fully open (r=1), Max speed limit (VSL=120) - "No Control"
u_control0_2 = [ones(Nc, 1); 120 * ones(Nc, 1)];

% Initial conditions (arbitrary?)
rho_0 = 25 * ones(6,1);
v_0 = 80 * ones(6,1);
w_r_0 = 0;
x0 = [v_0 ; rho_0 ; w_r_0];


% objective function
f = @(u) simulation(u, x0);

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...         % Use Sequential Quadratic Programming
    'Display', 'iter', ...           % Show output for each iteration
    'MaxFunctionEvaluations', 10000, ... % Increase limit (simulation is expensive)
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