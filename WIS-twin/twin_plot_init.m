function handles = twin_plot_init(y_ref, N)
%TWIN_PLOT_INIT  Create live-updating figure window. Call once before the loop.
%   handles = twin_plot_init(y_ref, N)
%   y_ref : 3x1 reference water levels
%   N     : MPC horizon length (number of predicted steps)

handles.fig = figure('Name', 'WIS Digital Twin', 'NumberTitle', 'off', ...
    'Position', [50 50 1400 900]);

handles.ax_levels  = subplot(3,2,1); title('Water levels'); ylabel('m'); grid on; hold on;
handles.ax_u       = subplot(3,2,2); title('MPC control input'); ylabel('servo'); grid on; hold on;
handles.ax_innov   = subplot(3,2,3); title('Innovation (pre-correction residual)'); ylabel('m'); grid on; hold on;
handles.ax_kgain   = subplot(3,2,4); title('Kalman gain (diagonal)'); grid on; hold on;
handles.ax_horizon = subplot(3,2,[5 6]); title('MPC predicted trajectory (current step)'); ylabel('m'); grid on; hold on;

colors = {'b','r','g'};
for i = 1:3
    handles.h_meas(i)    = plot(handles.ax_levels,  NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('Pool %d meas', i));
    handles.h_pred(i)    = plot(handles.ax_levels,  NaN, NaN, [colors{i} '--'], 'DisplayName', sprintf('Pool %d pred', i));
    handles.h_ref(i)     = yline(handles.ax_levels, y_ref(i), [colors{i} ':']);
    handles.h_u(i)       = plot(handles.ax_u,        NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('u%d', i));
    handles.h_innov(i)   = plot(handles.ax_innov,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('innov%d', i));
    handles.h_kg(i)      = plot(handles.ax_kgain,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('K%d', i));
    handles.h_hor(i)     = plot(handles.ax_horizon, NaN(1,N), NaN(1,N), [colors{i} '-o'], 'DisplayName', sprintf('Pool %d predicted', i));
    handles.h_ref_hor(i) = yline(handles.ax_horizon, y_ref(i), [colors{i} ':']);
end
legend(handles.ax_levels, 'Location', 'best');
legend(handles.ax_horizon, 'Location', 'best');
yline(handles.ax_innov, 0, 'k:');
end
