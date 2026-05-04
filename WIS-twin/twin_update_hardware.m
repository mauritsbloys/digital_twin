function twin_update_hardware(y_meas, u_prev, epoch)
%TWIN_UPDATE_HARDWARE  Called from FireflyCommunicationPSTC callback.
%   Runs one Kalman+MPC step and logs/plots in hardware mode.

persistent x_hat P u_mpc_prev log_file plt ...
           t_vec y_hist y_pred_hist innov_hist u_hist K_diag_hist ...
           comb_Pool_disc

% Lazy initialisation on first call
if isempty(x_hat)
    twin_config;
    load(fullfile(fileparts(mfilename('fullpath')), ...
        '../WIS-sim/simulation/distributed_workspace.mat'), 'comb_plant_cont');
    comb_Pool_disc = c2d(comb_plant_cont, 1, 'zoh');
    x_hat      = zeros(size(comb_Pool_disc.A, 1), 1);
    P          = eye(size(comb_Pool_disc.A, 1));
    u_mpc_prev = zeros(size(comb_Pool_disc.B, 2), 1);
    timestamp  = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    log_file   = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
    log_file_latest = fullfile(LOG_DIR, 'twin_log.csv');
    if ~isfolder(LOG_DIR); mkdir(LOG_DIR); end
    if isfile(log_file_latest); delete(log_file_latest); end
    t_vec = []; y_hist = zeros(3,0); y_pred_hist = zeros(3,0);
    innov_hist = zeros(3,0); u_hist = zeros(3,0); K_diag_hist = zeros(3,0);
    if PLOT_LIVE; plt = twin_plot_init(y_ref, N); end
end

twin_config;  % reload config each call so tuning changes take effect
A = comb_Pool_disc.A; B = comb_Pool_disc.B; C = comb_Pool_disc.C;
Q_kal = Q_kal_scale * eye(size(A,1));
R_kal = R_kal_scale * eye(size(C,1));

y_dev = y_meas - y_ref;
[x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_mpc_prev);

u_mpc = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), Q_mpc, R_mpc, N, du_max, u_min, u_max, u_mpc_prev);
u_mpc_prev = u_mpc;

y_pred = C * x_hat + y_ref;
twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, 1);
twin_log_write(log_file_latest, epoch, y_meas, y_pred, innov, u_mpc, 1);

t_vec       = [t_vec, epoch];
y_hist      = [y_hist,      y_meas];
y_pred_hist = [y_pred_hist, y_pred];
innov_hist  = [innov_hist,  innov];
u_hist      = [u_hist,      u_mpc];
K_gain      = (P * C') / (C * P * C' + R_kal);
K_diag_hist = [K_diag_hist, diag(K_gain(1:3,:))];

mpc_traj = zeros(3, N);
x_tmp = x_hat;
for i = 1:N
    x_tmp = A * x_tmp + B * u_mpc;
    mpc_traj(:,i) = C * x_tmp + y_ref;
end

if PLOT_LIVE
    twin_plot_update(plt, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref);
end
end
