%% demo_aemf_lekkagefout.m — Demonstratie multiplicatieve lekkagefoutschatting
%
% Injecteert bekende fouten (c1=+20%, c3=+15%) in de gesimuleerde plant
% terwijl de Kalman-filter het nominale lekkagemodel gebruikt.
% De AEMF-schatter schat elke 20 stappen de fouten terug uit de innovaties.
%
% Draai vanuit WIS-twin/

addpath(fileparts(mfilename('fullpath')));
twin_config;

%% Plant matrices laden en discretiseren op 1 Hz
load(fullfile('../WIS-sim/simulation/distributed_workspace.mat'), 'comb_plant_cont');
plant_disc = c2d(comb_plant_cont, 1, 'zoh');
A = plant_disc.A;  B = plant_disc.B;  C = plant_disc.C;
wl_idx   = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
n_states = size(A,1);

%% Kalman covarianties
if USE_ESTIMATED_QR
    Q_kal_use = Q_kal_final;
    R_kal_use = R_kal_final;
else
    Q_kal_use = Q_kal_scale * eye(n_states);
    R_kal_use = R_kal_scale * eye(3);
end

%% Ware multiplicatieve lekkagefouten
c_true = [0.20; 0.00; 0.15];   % kanaal 1: +20%, kanaal 2: 0%, kanaal 3: +15%

%% Foutief Wis-struct voor de gesimuleerde plant
Wis_faulty = Wis;
for j = 1:3
    Wis_faulty.leak_alpha(j) = Wis.leak_alpha(j) * (1 + c_true(j));
    Wis_faulty.leak_beta(j)  = Wis.leak_beta(j)  * (1 + c_true(j));
end

%% Nominale lekkage bij setpoints (voor toestandsafwijkingsformulering)
d_leak_nom = twin_compute_leakage(y_ref, Wis, wl_idx, n_states);

%% Simulatie-instellingen
N_SIM        = 120;
FAULT_WINDOW = 20;
rng(42);   % reproduceerbare ruis

%% Initialisatie plant en Kalman
x_plant            = zeros(n_states, 1);
x_plant(wl_idx(1)) =  0.04;   % start iets boven setpoint voor voldoende dynamiek
x_plant(wl_idx(2)) = -0.03;
x_plant(wl_idx(3)) =  0.05;
x_hat   = zeros(n_states, 1);
P       = eye(n_states);
u_prev  = 0.15 * ones(3, 1);  % vaste sluisopening (geen MPC nodig voor demo)

%% Sliding window buffers
innov_buf = nan(3, FAULT_WINDOW);
hest_buf  = nan(3, FAULT_WINDOW);

%% Opslag resultaten
t_est_vec = [];
c_hat_mat = zeros(3, 0);
sig_vec   = [];
innov_mat = zeros(3, N_SIM);
y_mat     = zeros(3, N_SIM);

%% Hoofdlus
for step = 1:N_SIM

    % 1. Plant stap (faulty lekkage + meetruis)
    h_sim      = C * x_plant + y_ref;
    d_leak_sim = twin_compute_leakage(h_sim, Wis_faulty, wl_idx, n_states) - d_leak_nom;
    x_plant    = A * x_plant + B * u_prev + d_leak_sim;
    for ii = 1:3
        x_plant(wl_idx(ii)) = max(x_plant(wl_idx(ii)), -y_ref(ii));
    end
    noise  = sqrt(diag(R_kal_use)) .* randn(3,1);
    y_meas = C * x_plant + y_ref + noise;

    % 2. Kalman-filter (nominale lekkage — weet niet van de fout)
    y_dev  = y_meas - y_ref;
    h_est  = C * x_hat + y_ref;
    d_leak = twin_compute_leakage(h_est, Wis, wl_idx, n_states) - d_leak_nom;
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal_use, R_kal_use, ...
                                           x_hat, P, y_dev, u_prev, d_leak);

    % 3. Sliding window update
    h_est_now = C * x_hat + y_ref;
    innov_buf = [innov_buf(:,2:end), innov];
    hest_buf  = [hest_buf(:,2:end),  h_est_now];

    % 4. Lekkagefout-schatting elke FAULT_WINDOW stappen
    if mod(step, FAULT_WINDOW) == 0 && step >= FAULT_WINDOW
        [c_hat, sig_min] = twin_estimate_leakage_faults( ...
            innov_buf, hest_buf, Wis, wl_idx, C, n_states);
        t_est_vec = [t_est_vec, step];          %#ok<AGROW>
        c_hat_mat = [c_hat_mat, c_hat];         %#ok<AGROW>
        sig_vec   = [sig_vec,   sig_min];       %#ok<AGROW>
        fprintf('Stap %3d — c_hat = [%+.3f  %+.3f  %+.3f],  sigma_min^2 = %.2e\n', ...
                step, c_hat(1), c_hat(2), c_hat(3), sig_min);
    end

    innov_mat(:,step) = innov;
    y_mat(:,step)     = y_meas;
end

%% Plot
figure('Name','AEMF Lekkagefoutschatting — Demo','Position',[80 60 1050 720]);
clr = lines(3);
lbl = {['c_1  (ware waarde = ' num2str(c_true(1),'%+.2f') ')'], ...
       ['c_2  (ware waarde = ' num2str(c_true(2),'%+.2f') ')'], ...
       ['c_3  (ware waarde = ' num2str(c_true(3),'%+.2f') ')']};
t_all = 1:N_SIM;

% --- Subplot 1: geschatte vs ware fout ---
subplot(3,1,1); hold on; box on;
for j = 1:3
    yline(c_true(j), '--', 'Color', clr(j,:), 'LineWidth', 1.2, ...
          'HandleVisibility', 'off');
    if ~isempty(c_hat_mat)
        plot(t_est_vec, c_hat_mat(j,:), 'o-', 'Color', clr(j,:), ...
             'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', clr(j,:), ...
             'DisplayName', lbl{j});
    end
end
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlim([0 N_SIM]); grid on;
xlabel('Tijdstap [s]');
ylabel('Lekkagefout  c_j  [-]');
title('Geschatte multiplicatieve lekkagefout (punten) vs. ware waarde (stippellijn)');
legend('Location', 'best', 'FontSize', 9);

% --- Subplot 2: gemeten waterpeilen ---
subplot(3,1,2); hold on; box on;
for i = 1:3
    plot(t_all, y_mat(i,:), 'Color', clr(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Pool %d (setpoint %.2f m)', i, y_ref(i)));
    yline(y_ref(i), '--', 'Color', clr(i,:), 'LineWidth', 0.8, ...
          'HandleVisibility', 'off');
end
xlim([0 N_SIM]); grid on;
xlabel('Tijdstap [s]');
ylabel('Waterpeil [m]');
title('Gemeten waterpeilen — dynamiek zorgt voor observeerbaarheid');
legend('Location', 'best', 'FontSize', 9);

% --- Subplot 3: observeerbaarheidsmaat sigma_min^2 ---
subplot(3,1,3); hold on; box on;
if ~isempty(sig_vec)
    semilogy(t_est_vec, sig_vec, 'ks-', 'LineWidth', 1.6, ...
             'MarkerSize', 7, 'MarkerFaceColor', 'k', ...
             'DisplayName', '\sigma_{min}(E)^2');
    yline(1e-6, 'r--', 'LineWidth', 1.2, ...
          'DisplayName', 'Drempel  1\times10^{-6}  (te singulier)');
end
xlim([0 N_SIM]); grid on;
xlabel('Tijdstap [s]');
ylabel('\sigma_{min}(E)^2  [-]');
title('Observeerbaarheidsmaat — boven drempel: betrouwbare schatting');
legend('Location', 'best', 'FontSize', 9);

sgtitle('WIS Digital Twin — AEMF Multiplicatieve Lekkagefoutschatting', ...
        'FontSize', 12, 'FontWeight', 'bold');
