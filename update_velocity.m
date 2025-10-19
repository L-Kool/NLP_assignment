% Function to calculate v(k+1)
% Inputs:
%   x_k - entire state vector [rho ; v ; w_k]
%   u_k - entire control input vector [r ; V_SL]
%   params - parameters
%   dims - dimensions

function v_next = update_velocity(x_k, u_k, params)
    % Unpacking the state vector 
    rho_k = x_k(1:6);
    v_k = x_k(7:12);
    w_r = x_k(13);
    % Unpacking the input control vector
    r_k = u_k(1);
    V_SL = u_k(2);

    % ADD BOUNDARY CONDITIONS
    v_prev = [v_k(1) ; v_k(1:end-1)];
    rho_next = [rho_k(2:end) ; rho_k(end)];

    % Relaxation term
    V_i([1, 4, 5, 6]) = min( (1+params.alpha)*V_SL, params.v_f * exp(-(1/params.a) * (rho_k/params.rho_c) .^ params.a));
    V_i([2,3]) = params.v_f * exp(-(1/params.a) * (rho_k/params.rho_c) .^ params.a);
    
    relax_term = (params.T / params.tau) * (V_i - v_k);

    % Convection term
    convection_term = (params.T / params.L) * (v_prev - v_k);

    % Anticipation term
    anticipation_term = (params.mu * params.T / (params.tau * params.L) ) .* (rho_next - rho_k) ./ (rho_k + params.K) ;

    % Calculation next velocity
    v_next = v_k + relax_term + convection_term - anticipation_term;
end