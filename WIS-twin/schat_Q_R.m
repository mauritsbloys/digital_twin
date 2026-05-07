%% schat_Q_R.m — EM-schatting van Kalman ruis-covarianties Q en R
%
% Uitvoer: data/Q_R_estimated.mat met variabelen Q_kal_final en R_kal_final.
% Voer dit script uit vanuit de WIS-twin map (of via STARTEN.m).

base = fileparts(mfilename('fullpath'));
addpath(base);   % zorg dat kalman_smoother.m gevonden wordt

%% Stap 1: Data laden en resamplen naar 1 Hz
raw   = readmatrix(fullfile(base, 'data', 'data.csv'));
t_raw = raw(:, 1);
y_raw = raw(:, [3, 5, 7]) / 100;      % kolommen s2, s4, s6 — omgezet naar [m]

t_1hz = (0 : floor(t_raw(end)))';     % uniforme tijdas: 0, 1, 2, ... seconden
y     = interp1(t_raw, y_raw, t_1hz, 'linear')';   % resultaat: 3 x N matrix
N     = size(y, 2);

fprintf('Data geladen: %d tijdstappen na resamplen naar 1 Hz\n', N);

%% Stap 2: Plantmodel laden en discretiseren op 1 Hz
load(fullfile(base, '..', 'WIS-sim', 'simulation', 'distributed_workspace.mat'), ...
     'comb_plant_cont');
plant = c2d(comb_plant_cont, 1, 'zoh');
A  = plant.A;
C  = plant.C;
nx = size(A, 1);
ny = size(C, 1);

fprintf('Plant geladen: nx = %d, ny = %d\n', nx, ny);

%% Stap 3: R schatten uit het stationaire segment (t > 500 s)
win    = 51;                           % vensterlengte: 51 samples = 51 seconden
trend  = movmean(y, win, 2);           % schat de langzame trend per kanaal
resid  = y - trend;                    % residu = hoog-frequente ruis

idx_ss = find(t_1hz >= 500);
R_est  = diag(var(resid(:, idx_ss), 0, 2));

fprintf('R schatting uit stationair segment (m^2):\n');
disp(R_est)

%% Stap 4: EM-algoritme (Shumway & Stoffer 1982)
Q  = 1e-6 * eye(nx);       % start klein: laat de data Q bepalen
R  = R_est;                % beginwaarde uit Stap 3
x0 = pinv(C) * y(:,1);    % schat begintoestand uit eerste meting
P0 = eye(nx);              % grote beginonzekerheid

fprintf('\nEM-iteraties starten...\n');
for iter = 1:30
    % E-stap: schat volledige toestandsgeschiedenis met alle metingen
    [xs, Ps, Pcs] = kalman_smoother(A, C, Q, R, y, x0, P0);

    % M-stap: update Q
    S11 = zeros(nx);  S10 = zeros(nx);  S00 = zeros(nx);
    for k = 2:N
        S11 = S11 + Ps(:,:,k)    + xs(:,k)   * xs(:,k)';
        S10 = S10 + Pcs(:,:,k-1) + xs(:,k)   * xs(:,k-1)';
        S00 = S00 + Ps(:,:,k-1)  + xs(:,k-1) * xs(:,k-1)';
    end
    Q = (S11 - S10*A' - A*S10' + A*S00*A') / (N-1);
    Q = (Q + Q') / 2;            % symmetrie herstellen (afrondingsfouten)
    Q = Q + 1e-10 * eye(nx);     % kleine regularisering: voorkomt singulariteit

    % M-stap: update R (diagonaal houden — sensoren zijn onafhankelijk)
    R_new = zeros(ny);
    for k = 1:N
        e     = y(:,k) - C * xs(:,k);
        R_new = R_new + e*e' + C*Ps(:,:,k)*C';
    end
    R = diag(diag(R_new / N));

    fprintf('  Iteratie %2d voltooid\n', iter);
end

Q_kal_final = Q;
R_kal_final = R;

%% Stap 5: Resultaat opslaan
out_file = fullfile(base, 'data', 'Q_R_estimated.mat');
save(out_file, 'Q_kal_final', 'R_kal_final');
fprintf('\nResultaat opgeslagen in %s\n', out_file);
fprintf('R_kal_final diagonaal (m^2): [%.3e  %.3e  %.3e]\n', ...
        R_kal_final(1,1), R_kal_final(2,2), R_kal_final(3,3));

%% Stap 6: Validatie — witheidstoets op innovaties
innov = zeros(ny, N);
x_hat = x0;  P = P0;
for k = 1:N
    x_prior    = A * x_hat;
    P_prior    = A * P * A' + Q_kal_final;
    innov(:,k) = y(:,k) - C * x_prior;
    S          = C * P_prior * C' + R_kal_final;
    K          = P_prior * C' / S;
    x_hat      = x_prior + K * innov(:,k);
    IKC        = eye(nx) - K * C;
    P          = IKC * P_prior * IKC' + K * R_kal_final * K';
end

figure('Name', 'Innovatie witheidstoets');
for i = 1:3
    subplot(3,1,i)
    autocorr(innov(i,:), 'NumLags', 30)
    title(sprintf('Innovatie autocorrelatie — sensor y%d', i))
    grid on
end

fprintf('\nValidatie: controleer of alle ACF-pieken (lag >= 1) binnen de blauwe band vallen.\n');
fprintf('Zo ja: Q en R zijn correct. Ga door naar Stap 6 (twin_config.m updaten).\n');
