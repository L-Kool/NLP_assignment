% Function to calculate rho(k+1)
% Inputs:
%   x_k - entire state vector [rho ; v ; w_k]
%   u_k - entire control input vector [r ; V_SL]
%   params - parameters
%   dims - dimensions
%   k - time-step, needed to define q_i

function rho_next = update_density_var_b(x_k, u_k, params, k, E2)

    % Unpacking the state vector 
    v_k = x_k(1:6);
    rho_k = x_k(7:12);
    w_r = x_k(13);

    % Unpacking the input control vector
    r_k = u_k(1);

    % Defining initial flow entering
    if k < 60
        q_0 = (3000 + 50 * E2) * 1.5;        
    else
        q_0 = (1000 + 50 * E2) * 1.5;
    end

    % Defining q_i, q_r_i
    q_i = params.lambda .* rho_k .* v_k ;
    q_i_prev = [q_0 ; q_i(1:end-1)];
    q_r_i = zeros(6,1);
    qr5_1 = r_k * params.C_r;
    qr5_2 = params.D_r + w_r / params.T;
    qr5_3 = params.C_r * ((params.rho_m - rho_k(5)) / (params.rho_m - params.rho_c));
    q_r_i(5) = min([qr5_1, qr5_2, qr5_3]);

    % Calculation next traffic density
    rho_next = rho_k + (params.T / (params.lambda * params.L)) * (q_i_prev - q_i + q_r_i);
    
end