%% STARTEN.m — Startcommando's voor de WIS Digital Twin
%
% Dit bestand bevat alle commando's om de digital twin en het dashboard
% op te starten. Kopieer de relevante regels naar de MATLAB Command Window.

%% ── 0. Q/R RUIS-COVARIANTIE SCHATTEN (eenmalig, vóór eerste gebruik) ────
%
%  Voer dit eenmalig uit om Q en R te schatten uit echte sensordata.
%  Vereist: data/data.csv (kolommen: t_s, s1_cm..s7_cm, gates gesloten).
%  Resultaat wordt opgeslagen in data/Q_R_estimated.mat en automatisch
%  geladen door twin_config.m bij elke volgende run.

cd(fileparts(which('digital_twin')))
schat_Q_R

%% ── 1. DIGITAL TWIN (simulator-modus) ───────────────────────────────────
%
%  Draait de twin volledig in software: geen hardware nodig.
%  Instellingen (horizon, setpoints, looptijd) staan in twin_config.m.
%  Logs worden opgeslagen in WIS-twin/data/.

cd(fileparts(which('digital_twin')))   % zorg dat je in de WIS-twin map zit
digital_twin

%% ── 2. DASHBOARD (web-interface) ────────────────────────────────────────
%
%  Open een terminal (bijv. Windows Terminal of de MATLAB Terminal) en
%  voer de onderstaande twee regels uit:
%
%    cd(fileparts(which('digital_twin')))
%    python -m http.server 8080
%
%  Open daarna in de browser:
%    http://localhost:8080/twin_dashboard.html
%
%  Het dashboard ververst automatisch elke 2 seconden en leest
%  data/twin_log.csv.

%% ── 3. HARDWARE-MODUS ───────────────────────────────────────────────────
%
%  Activeer de twin vanuit FireflyCommunicationPSTC door de property
%  twin_active op true te zetten voordat je main_pstc start:
%
%    fc = FireflyCommunicationPSTC(...);
%    fc.twin_active = true;
%    main_pstc
%
%  twin_update_hardware.m wordt dan automatisch elke callback-stap
%  aangeroepen.

%% ── 4. TESTS UITVOEREN ──────────────────────────────────────────────────
%
%  Verifieer de losse componenten:

cd(fileparts(which('digital_twin')))
test_twin_kalman
test_twin_mpc
test_twin_log
