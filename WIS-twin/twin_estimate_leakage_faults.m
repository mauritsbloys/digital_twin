function [c_hat, sigma_min_sq] = twin_estimate_leakage_faults(innov_hist, h_est_hist, Wis, wl_idx, C, n_states)
% TWIN_ESTIMATE_LEAKAGE_FAULTS  Schat multiplicatieve lekkagefouten via AEMF-methode.
%
%   Inputs:
%     innov_hist   — [3×K] Kalman-innovatiegeschiedenis (K tijdstappen)
%     h_est_hist   — [3×K] absolute waterstandschattingen [m]
%     Wis          — lekkage-struct (h0, leak_alpha, leak_beta, area1/2/3)
%     wl_idx       — indices waterstandtoestanden in toestandsvector, 3×1
%     C            — meetmatrix [3×n_states]
%     n_states     — totaal aantal toestanden
%
%   Outputs:
%     c_hat        — [3×1] multiplicatieve foutschatting per lekkagekanaal
%                    c_j: werkelijke lekkage_j = (1 + c_j) * nominale_lekkage_j
%     sigma_min_sq — observeerbaarheidsmaat: sigma_min(E)^2, laag = slecht observeerbaar
%
%   Methode (analoog aan AEMF run_and_process_simulation.m, lijnen 11-15):
%     E((k-1)*3+1:k*3, j) = C * delta_d_j(h_est(:,k))
%     c_hat = pinv(E) * r_vec

K = size(innov_hist, 2);
E = zeros(3*K, 3);

for k = 1:K
    h_k = h_est_hist(:, k);
    for j = 1:3
        delta_d = compute_delta_d_j(j, h_k, Wis, wl_idx, n_states);
        E((k-1)*3 + 1 : k*3, j) = C * delta_d;
    end
end

r_vec = innov_hist(:);   % [3K×1]

sv = svd(E);
sigma_min_sq = min(sv)^2;

if sigma_min_sq < 1e-6
    warning('twin_estimate_leakage_faults: E bijna singulier (sigma_min^2=%.2e) — waterpeilen te dicht bij setpoint, fout niet onderscheidbaar.', sigma_min_sq);
    c_hat = nan(3,1);
else
    c_hat = pinv(E) * r_vec;
end
end

% -------------------------------------------------------------------------
function delta_d = compute_delta_d_j(j, h_abs, Wis, wl_idx, n_states)
% Lekkagecorrectievector voor uitsluitend kanaal j (effect van multiplicatieve fout c_j).
%   j=1: q1 (h0 -> pool 1),  j=2: q2 (pool 1 -> pool 2),  j=3: q3 (pool 2 -> pool 3)
delta_d = zeros(n_states, 1);
switch j
    case 1
        q1 = wis_leakage(Wis.h0,    h_abs(1), Wis.leak_alpha(1), Wis.leak_beta(1));
        delta_d(wl_idx(1)) =  q1 / Wis.area1;
    case 2
        q2 = wis_leakage(h_abs(1), h_abs(2), Wis.leak_alpha(2), Wis.leak_beta(2));
        delta_d(wl_idx(1)) = -q2 / Wis.area1;
        delta_d(wl_idx(2)) =  q2 / Wis.area2;
    case 3
        q3 = wis_leakage(h_abs(2), h_abs(3), Wis.leak_alpha(3), Wis.leak_beta(3));
        delta_d(wl_idx(2)) = -q3 / Wis.area2;
        delta_d(wl_idx(3)) =  q3 / Wis.area3;
end
end
