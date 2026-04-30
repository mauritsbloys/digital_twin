%% twin_config.m — Digital twin configuration

% Data source
USE_HARDWARE = false;  % true = Firefly serial, false = internal plant simulator

% Kalman filter noise covariances
Q_kal = 1e-4 * eye(6);   % process noise
R_kal = 1e-3 * eye(3);   % measurement noise

% MPC parameters
N      = 10;             % prediction horizon [time steps]
Q_mpc  = 10  * eye(3);   % weight on setpoint deviation
R_mpc  = 0.1 * eye(3);   % weight on control effort
du_max = 20;             % max gate change per time step [servo units]
u_min  = zeros(3,1);     % lower bound on gate opening
u_max  = 255 * ones(3,1); % upper bound on gate opening

% Setpoints [m]
y_ref = [0.25; 0.20; 0.15];

% Logging and display
LOG_DIR  = fullfile(fileparts(mfilename('fullpath')), 'data');
PLOT_LIVE = true;
WEB_DASH  = true;

% Paths to other modules
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/simulation'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/functions'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/functions_jacob'));
