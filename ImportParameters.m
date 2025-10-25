function params = ImportParameters()
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
end 