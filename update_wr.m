% Function to calculate w_r(k+1)
% Inputs:
%   x_k - entire state vector [rho ; v ; w_k]
%   u_k - entire control input vector [r ; V_SL]
%   params - parameters
%   dims - dimensions

function w_r_next = update_wr(x_k, u_k, params)

    % Unpacking the state vector 
    rho_k = x_k(1:6);
    w_r = x_k(13);

    % Unpacking the input control vector
    r_k = u_k(1);

    % Defining q_r_5
    q_r_5 = min(r_k * params.C_r, params.D_r + w_r / T, ...
        params.C_r * (params.rho_m - rho_k(5) / (params.rho_m - params.rho_c)));

    % Calculating next w_r
    w_r_next = w_r + params.T*params.D_r - q_r_5;
    
end