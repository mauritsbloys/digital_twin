%% digital_twin.m — Main loop for the WIS digital twin
%
% Runs simulator mode by default (USE_HARDWARE = false in twin_config.m).
% In simulator mode, advances comb_Pool_disc internally and injects a
% step disturbance at epoch 20.

addpath(fileparts(mfilename('fullpath')));
twin_config;

%% Load plant matrices from pre-computed workspace
load(fullfile('../WIS-sim/simulation/distributed_workspace.mat'), ...
    'comb_Pool_disc', 'h');
fprintf('Sample time h = %.4f s (expected ~1 s for 1 Hz)\n', h);
assert(h > 0.1 && h < 10.0, 'pause duration h=%.4f is outside expected range', h);
A = comb_Pool_disc.A;
B = comb_Pool_disc.B;
C = comb_Pool_disc.C;

%% Initialise Kalman state
x_hat = zeros(size(A,1), 1);
P     = eye(size(A,1));

%% Initialise MPC
u_prev = zeros(size(B,2), 1);

%% Initialise simulator plant state (only used when USE_HARDWARE = false)
x_plant = zeros(size(A,1), 1);
DISTURBANCE_EPOCH = 20;
disturbance = [-0.015; 0; 0];   % outflow from pool 1 [m³/min equivalent]

%% Run duration — lower for quick tests, 1800 = 30 min full run
MAX_STEPS = 60;

%% Initialise logging
timestamp  = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
log_file   = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
log_file_latest = fullfile(LOG_DIR, 'twin_log.csv');
if ~isfolder(LOG_DIR)
    mkdir(LOG_DIR);
end
if isfile(log_file_latest); delete(log_file_latest); end

%% Initialise plots
if PLOT_LIVE
    plt = twin_plot_init(y_ref, N);
end

%% History buffers
t_vec        = [];
y_hist       = zeros(3, 0);
y_pred_hist  = zeros(3, 0);
innov_hist   = zeros(3, 0);
u_hist       = zeros(3, 0);
K_diag_hist  = zeros(3, 0);

fprintf('WIS Digital Twin starting (SIMULATOR mode)...\n');

%% Main loop
for epoch = 1:MAX_STEPS

    %% 1. Simulator: advance internal plant
    d = zeros(size(A,1), 1);
    if epoch >= DISTURBANCE_EPOCH
        d(1) = disturbance(1);  % state-space disturbance on pool-1 level state
    end
    x_plant = A * x_plant + B * u_prev + d;
    y_meas  = C * x_plant + y_ref;   % absolute water levels
    triggered = 1;

    %% 2. Kalman filter update
    y_dev = y_meas - y_ref;   % work in deviation from setpoint
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_prev);

    %% 3. MPC
    u_mpc = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);
    u_prev = u_mpc;

    %% 4. Compute MPC predicted trajectory for plotting
    mpc_traj = zeros(3, N);
    x_tmp = x_hat;
    for i = 1:N
        x_tmp = A * x_tmp + B * u_mpc;
        mpc_traj(:,i) = C * x_tmp + y_ref;
    end

    %% 5. Log
    y_pred = C * x_hat + y_ref;
    twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, triggered);
    twin_log_write(log_file_latest, epoch, y_meas, y_pred, innov, u_mpc, triggered);

    %% 6. Update history and plot
    t_vec       = [t_vec, epoch];
    y_hist      = [y_hist,      y_meas];
    y_pred_hist = [y_pred_hist, y_pred];
    innov_hist  = [innov_hist,  innov];
    u_hist      = [u_hist,      u_mpc];
    K_gain      = (P * C') / (C * P * C' + R_kal);  % approx from post-update P
    K_diag_hist = [K_diag_hist, diag(K_gain(1:3,:))];

    if PLOT_LIVE
        twin_plot_update(plt, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref);
    end

    pause(h);   % simulate real-time 1Hz
end

fprintf('Digital twin finished. Log: %s\n', log_file);
