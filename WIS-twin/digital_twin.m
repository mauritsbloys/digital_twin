%% digital_twin.m — Main loop for the WIS digital twin
%
% USE_HARDWARE = false (twin_config.m): simulator mode, internal plant model.
% USE_HARDWARE = true:  hardware mode, reads sensor data from Firefly via serial.

addpath(fileparts(mfilename('fullpath')));
twin_config;

%% Load plant matrices from pre-computed workspace
% comb_plant_cont is the continuous-time Cantoni plant (Ap, Bp, Cp).
% We discretize it at 1 Hz here so the Kalman/MPC match the twin loop rate.
load(fullfile('../WIS-sim/simulation/distributed_workspace.mat'), 'comb_plant_cont');
plant_disc = c2d(comb_plant_cont, 1, 'zoh');
A = plant_disc.A;
B = plant_disc.B;
C = plant_disc.C;

% Bepaal welke toestanden de waterpeilen zijn (via C-matrix)
wl_idx = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
if USE_ESTIMATED_QR
    Q_kal = Q_kal_final;
    R_kal = R_kal_final;
else
    Q_kal = Q_kal_scale * eye(size(A,1));
    R_kal = R_kal_scale * eye(size(C,1));
end

%% Initialise Kalman state
x_hat = zeros(size(A,1), 1);
P     = eye(size(A,1));

%% Initialise MPC
u_prev = zeros(size(B,2), 1);

%% Initialise simulator plant state (only used when USE_HARDWARE = false)
x_plant           = zeros(size(A,1), 1);
DISTURBANCE_EPOCH = 20;
disturbance       = [-0.015; 0; 0];

%% Run duration — lower for quick tests, 1800 = 30 min full run
MAX_STEPS = 60;

%% Initialise logging
timestamp       = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
log_file        = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
log_file_latest = fullfile(LOG_DIR, 'twin_log.csv');
if ~isfolder(LOG_DIR)
    mkdir(LOG_DIR);
end
try; delete(log_file_latest); catch; end

%% Initialise plots
if PLOT_LIVE
    plt = twin_plot_init(y_ref, N);
end

%% History buffers (preallocated to MAX_STEPS)
t_vec       = zeros(1, MAX_STEPS);
y_hist      = zeros(3, MAX_STEPS);
y_pred_hist = zeros(3, MAX_STEPS);
innov_hist  = zeros(3, MAX_STEPS);
u_hist      = zeros(3, MAX_STEPS);
K_diag_hist = zeros(3, MAX_STEPS);

%% Open serial connection for hardware mode
if USE_HARDWARE
    device = serialport(COM_PORT, 115200, 'Timeout', 2);
    configureTerminator(device, 'LF');
    fprintf('WIS Digital Twin starting (HARDWARE mode, %s)...\n', COM_PORT);
else
    fprintf('WIS Digital Twin starting (SIMULATOR mode)...\n');
end

%% Main loop
epoch = 0;
step  = 0;
while step < MAX_STEPS

    %% 1. Data acquisition
    if USE_HARDWARE
        try
            serial_line = readline(device);
        catch
            continue;
        end
        parts = split(strtrim(serial_line), ',');
        if numel(parts) ~= 13
            continue;
        end
        epoch     = str2double(parts(1));
        y_meas    = [str2double(parts(7)); str2double(parts(8)); str2double(parts(9))] / 1e6;
        triggered = str2double(parts(13));
    else
        epoch      = epoch + 1;
        h_sim      = C * x_plant + y_ref;
        d_leak_sim = twin_compute_leakage(h_sim, Wis, wl_idx, size(A,1));
        d_ext      = zeros(size(A,1), 1);
        if epoch >= DISTURBANCE_EPOCH
            d_ext(wl_idx(1)) = disturbance(1);
        end
        x_plant   = A * x_plant + B * u_prev + d_leak_sim + d_ext;
        y_meas    = C * x_plant + y_ref;
        triggered = 1;
    end
    step = step + 1;

    %% 2. Kalman filter update
    y_dev   = y_meas - y_ref;
    h_est   = C * x_hat + y_ref;
    d_leak  = twin_compute_leakage(h_est, Wis, wl_idx, size(A,1));
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_prev, d_leak);

    %% 3. MPC
    u_mpc  = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);
    u_prev = u_mpc;

    %% 4. MPC predicted trajectory for plotting (inclusief lekkage)
    mpc_traj = zeros(3, N);
    x_tmp = x_hat;
    for i = 1:N
        h_tmp         = C * x_tmp + y_ref;
        d_mpc         = twin_compute_leakage(h_tmp, Wis, wl_idx, size(A,1));
        x_tmp         = A * x_tmp + B * u_mpc + d_mpc;
        mpc_traj(:,i) = C * x_tmp + y_ref;
    end

    %% 5. Log
    y_pred = C * x_hat + y_ref;
    twin_log_write(log_file,        epoch, y_meas, y_pred, innov, u_mpc, triggered);
    twin_log_write(log_file_latest, epoch, y_meas, y_pred, innov, u_mpc, triggered);

    %% 6. Update history and plot
    t_vec(:,step)       = epoch;
    y_hist(:,step)      = y_meas;
    y_pred_hist(:,step) = y_pred;
    innov_hist(:,step)  = innov;
    u_hist(:,step)      = u_mpc;
    K_gain              = (P * C') / (C * P * C' + R_kal);
    K_diag_hist(:,step) = diag(K_gain(1:3,:));

    if PLOT_LIVE
        twin_plot_update(plt, t_vec(:,1:step), y_hist(:,1:step), y_pred_hist(:,1:step), ...
                         innov_hist(:,1:step), u_hist(:,1:step), K_diag_hist(:,1:step), ...
                         mpc_traj, y_ref);
    end

    pause(H_LOOP);
end

%% Cleanup
if USE_HARDWARE
    delete(device);
end

fprintf('Digital twin finished. Log: %s\n', log_file);
