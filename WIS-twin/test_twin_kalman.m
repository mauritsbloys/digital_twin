%% test_twin_kalman.m — Tests for the Kalman filter

% Simple 1D system: x(k+1) = x(k), y(k) = x(k)
A = 1; B = 0; C = 1;
Q_kal = 0.01; R_kal = 0.1;
x0 = 0; P0 = 1;

% After 20 steps with y=1, x_hat should converge close to 1
x_hat = x0; P = P0;
for k = 1:20
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, 1, 0);
end
assert(abs(x_hat - 1) < 0.05, 'Kalman did not converge to measurement');
assert(isscalar(innov), 'Innovation should be scalar for 1D system');
disp('test_twin_kalman: PASSED');
