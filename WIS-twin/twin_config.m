%% twin_config.m — Digital twin configuration

% Guard: mfilename returns empty when run interactively from Editor in wrong directory
if isempty(mfilename('fullpath'))
    error('twin_config must be run as a script (e.g. via digital_twin), not interactively from the Editor in a different directory.');
end

% Data source
USE_HARDWARE = false;  % true = Firefly serial, false = internal plant simulator
COM_PORT     = 'COM3'; % seriële poort Firefly (alleen relevant bij USE_HARDWARE = true)

% Kalman filter noise covariances — laad data-gedreven schatting indien beschikbaar
qr_file = fullfile(fileparts(mfilename('fullpath')), 'data', 'Q_R_estimated.mat');
USE_ESTIMATED_QR = isfile(qr_file);
if USE_ESTIMATED_QR
    load(qr_file, 'Q_kal_final', 'R_kal_final');
else
    % standaardwaarden totdat schat_Q_R.m is uitgevoerd
    Q_kal_scale = 1e-4;
    R_kal_scale = 1e-3;
end

% MPC parameters
N      = 10;             % prediction horizon [time steps]
Q_mpc  = 10  * eye(3);   % weight on setpoint deviation
R_mpc  = 0.1 * eye(3);   % weight on control effort
du_max = 20;             % max gate change per time step [servo units]
u_min  = zeros(3,1);     % lower bound on gate opening
u_max  = 255 * ones(3,1); % upper bound on gate opening

% Setpoints [m]
y_ref = [0.25; 0.20; 0.15];

% Loop timing: 0 = zo snel mogelijk (testen), 1 = real-time 1 Hz
H_LOOP = 0;

% Logging and display
LOG_DIR  = fullfile(fileparts(mfilename('fullpath')), 'data');
PLOT_LIVE = true;
WEB_DASH  = true;

% Add WIS-sim modules to path (guarded against duplicates)
for sub_dir = {'../WIS-sim/simulation', '../WIS-sim/functions', '../WIS-sim/functions_jacob'}
    p = fullfile(fileparts(mfilename('fullpath')), sub_dir{1});
    if ~contains(path, p)
        addpath(p);
    end
end
