% wis_properties.m — WIS laboratoriumopstelling eigenschappen
% Bevat empirische en fysische parameters voor de 3-bassin opstelling.

% Lekkageparameters per sluis (empirisch bepaald)
% Formule: q_lek [cm3/s] = alpha*sqrt(dh [cm]) + beta*dh^(3/2)
Wis.leak_alpha = [39.617, 9.402, 40.310]; % sluis 1, 2, 3
Wis.leak_beta  = [0.328,  0.162,  0.559];
Wis.h0         = 0.30;  % aanname pool 0 peil [m]
