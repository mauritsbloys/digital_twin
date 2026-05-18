%% schat_Q_R.m — Schatting van Kalman ruis-covarianties Q en R
%
% R: data-gedreven uit hoog-frequente sensorresiduen (movmean trendverwijdering)
% Q: fysisch gemotiveerd uit gemeten lekkagesnelheid per bassin
%
% Invoer: METING_FILE hieronder (kolommen: t_s, s1..s7 in cm)
% Uitvoer: data/Q_R_estimated.mat

base = fileparts(mfilename('fullpath'));
addpath(base);

%% Configuratie — pas aan bij nieuwe meting
METING_FILE = fullfile(base, 'data', 'meting_20260513_143530.csv');

%% Stap 1: Data laden
% Waterstandsensoren: kolommen 3, 5, 7 (s2, s4, s6) → omzetten naar [m]
raw   = readmatrix(METING_FILE);
t_raw = raw(:, 1);
y_raw = raw(:, [3, 5, 7]) / 100;

t_1hz = (0 : floor(t_raw(end)))';
y     = interp1(t_raw, y_raw, t_1hz, 'linear')';   % 3 x N
N     = size(y, 2);

fprintf('Data geladen: %d tijdstappen (%.0f seconden)\n', N, t_1hz(end));

%% Stap 2: Plantmodel laden en discretiseren op 1 Hz
load(fullfile(base, '..', 'WIS-sim', 'simulation', 'distributed_workspace.mat'), ...
     'comb_plant_cont');
plant = c2d(comb_plant_cont, 1, 'zoh');
A  = plant.A;
C  = plant.C;
nx = size(A, 1);
ny = size(C, 1);

fprintf('Plant geladen: nx = %d, ny = %d\n', nx, ny);

%% Stap 3: R schatten uit hoog-frequente sensorresiduen
% Verwijder langzame trend (lekkage, drift) met bewegend gemiddelde
% Wat overblijft is puur meetruis → variantie daarvan is R
win   = min(51, floor(N/4)*2 + 1);   % vensterlengte past zich aan N aan
trend = movmean(y, win, 2);
resid = y - trend;

R_est = diag(var(resid, 0, 2));
fprintf('\nR schatting uit sensorresiduen (m^2):\n');
fprintf('  y1: %.3e\n  y2: %.3e\n  y3: %.3e\n', R_est(1,1), R_est(2,2), R_est(3,3));

%% Stap 4: Q fysisch schatten uit gemeten lekkagesnelheid
% Lekkagesnelheid = gemiddelde daling per kanaal over de volledige meting
lek_ms = abs(y(:,1) - y(:,end)) / t_1hz(end);   % [m/s]
fprintf('\nGemeten lekkagesnelheid per bassin [mm/s]:\n');
fprintf('  Bassin 1: %.4f\n  Bassin 2: %.4f\n  Bassin 3: %.4f\n', lek_ms*1000);

% Q[i,i] = (lekkage per tijdstap)^2 — procesruis door onzekerheid in lekkagesnelheid
% Dit is een conservatieve bovengrens: het Kalman-filter krijgt genoeg speling
% om de werkelijke waterstand bij te houden ondanks niet-gemodelleerde lekkage
q_vals       = (lek_ms * 1).^2;   % dt = 1 s
Q_est        = 1e-8 * eye(nx);
water_states = [1, 5, 9];
for j = 1:3
    Q_est(water_states(j), water_states(j)) = max(q_vals(j), 1e-8);
end
fprintf('\nQ waterstandsdiagonaal (m^2):\n');
fprintf('  Q(1,1): %.3e\n  Q(5,5): %.3e\n  Q(9,9): %.3e\n', ...
        Q_est(1,1), Q_est(5,5), Q_est(9,9));

%% Stap 5: Opslaan
Q_kal_final = Q_est;
R_kal_final = R_est;
out_file = fullfile(base, 'data', 'Q_R_estimated.mat');
save(out_file, 'Q_kal_final', 'R_kal_final');
fprintf('\nOpgeslagen in %s\n', out_file);

%% Stap 6: Validatie — witheidstoets op innovaties
% Voer Kalman-filter voorwaarts uit met geschatte Q en R
% Als innovaties wit zijn → Q en R consistent met de data
innov = zeros(ny, N);
x_hat = pinv(C) * y(:, 1);
P     = eye(nx);
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
    autocorr(innov(i,:), 'NumLags', min(30, floor(N/4)));
    title(sprintf('Innovatie autocorrelatie — sensor y%d', i))
    grid on
end
fprintf('\nValidatie: ACF-pieken bij lag >= 1 moeten binnen de blauwe band vallen.\n');
fprintf('Let op: bij aanwezige lekkage zijn grote pieken bij lage lags verwacht.\n');
