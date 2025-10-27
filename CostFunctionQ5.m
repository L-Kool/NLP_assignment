function costQ5 = CostFunctionQ5(u, x_0, params, simType, W_vsl, W_ramp)
% This function computes the cost terms needed for Task 5
% The cost is defined as:
%   costQ5 = J_TTS + W_vsl * J_vsl + W_ramp * J_ramp

% Running the simulation to get state and output history
simResults = simulation(u, x_0, params, simType).stateHist;
stateHist = simResults.stateHist;
outputHist = simResults.outputHist;

% Finding applied inputs
Nc = length(u) / 2;
r_sim = repelem(u(1:Nc)', 1, 6);     % 1x120 row vector
VSL_sim = repelem(u(Nc+1:end)', 1, 6); % 1x120 row vector

% Calculating cost terms

% Sum of TTS
J_TTS = sum(outputHist);

% VSL term
vsl_term = (1 + params.alpha) .* VSL_sim;

% Densities rho(k) for k=1..120 (cols 2:121 of state_hist)
Rho2_k = stateHist(8, 2:121); % 1x120 (Segment 2)
Rho3_k = stateHist(9, 2:121); % 1x120 (Segment 3)

% Density-dependent term from Eq. (3)
exp_term_seg2 = params.v_f .* exp(-(1/params.a) .* (Rho2_k ./ params.rho_c).^params.a);
exp_term_seg3 = params.v_f .* exp(-(1/params.a) .* (Rho3_k ./ params.rho_c).^params.a);

% Actual desired speed V_i(k)
V_i_k_seg2 = min(vsl_term, exp_term_seg2);
V_i_k_seg3 = min(vsl_term, exp_term_seg3);

% (1+a)VSL - V_i(k)
diff_vsl_seg2 = vsl_term - V_i_k_seg2;
diff_vsl_seg3 = vsl_term - V_i_k_seg3;

% Cost is the sum of squares of these differences
J_vsl = sum(diff_vsl_seg2.^2) + sum(diff_vsl_seg3.^2);

% We need states at k=0..119 to calculate q_r,5(k)
Wr_k_ramp  = stateHist(13, 1:120); % 1x120
Rho5_k_ramp = stateHist(11, 1:120); % 1x120

% Calculate the three cost terms
q_r5_term1_allowed = r_sim .* params.C_r;
q_r5_term2_demand = params.D_r + Wr_k_ramp ./ params.T;
q_r5_term3_capacity = params.C_r .* (params.rho_m - Rho5_k_ramp) ./ (params.rho_m - params.rho_c);

% The actual on-ramp flow q_r,5(k)
q_r5_k = min(min(q_r5_term1_allowed, q_r5_term2_demand), q_r5_term3_capacity);

% diff = r(k)C_r - q_r,5(k)
diff_ramp = q_r5_term1_allowed - q_r5_k;

% Cost is the sum of squares of these differences
J_ramp = sum(diff_ramp.^2);

% Total combined cost
costQ5.cost = J_TTS + W_vsl * J_vsl + W_ramp * J_ramp;
costQ5.J_TTS = J_TTS;
costQ5.J_vsl = J_vsl;
costQ5.J_ramp = J_ramp;
end