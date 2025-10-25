A = [1:20 0.1:0.1:2];

A = [A(1, 1:20);
    A(1, 21:end)];

B = repelem(A, 1, 6);

u_control = [ones(1, 20);
    120 * ones(1, 20)];
control = repelem(u_control, 1, 6);

v = zeros(6, 121);
rho = zeros(6, 121);
w_r = zeros(1, 121);
v(:,1) = 80;
rho(:,1) = 25;
w_r(1) = 0;

v_prev = [v(1, 1);
        v(1:end-1, 1)];