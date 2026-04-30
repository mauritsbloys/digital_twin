function handles = twin_plot_init(y_ref)
%TWIN_PLOT_INIT  Create 5 live-updating figure windows. Call once before the loop.
%   Returns a struct of figure and axes handles for use in twin_plot_update.

handles.fig = figure('Name', 'WIS Digital Twin', 'NumberTitle', 'off', ...
    'Position', [50 50 1400 900]);

handles.ax_levels  = subplot(3,2,1); title('Water levels'); ylabel('m'); grid on; hold on;
handles.ax_u       = subplot(3,2,2); title('MPC control input'); ylabel('servo'); grid on; hold on;
handles.ax_innov   = subplot(3,2,3); title('Innovation (pre-correction residual)'); ylabel('m'); grid on; hold on;
handles.ax_kgain   = subplot(3,2,4); title('Kalman gain (diagonal)'); grid on; hold on;
handles.ax_horizon = subplot(3,2,[5 6]); title('MPC predicted trajectory (current step)'); ylabel('m'); grid on; hold on;

colors = {'b','r','g'};
for i = 1:3
    handles.h_meas(i)  = plot(handles.ax_levels,  NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('Pool %d meas', i));
    handles.h_pred(i)  = plot(handles.ax_levels,  NaN, NaN, [colors{i} '--'], 'DisplayName', sprintf('Pool %d pred', i));
    handles.h_ref(i)   = yline(handles.ax_levels, y_ref(i), [colors{i} ':']);
    handles.h_u(i)     = plot(handles.ax_u,        NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('u%d', i));
    handles.h_innov(i) = plot(handles.ax_innov,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('innov%d', i));
    handles.h_kg(i)    = plot(handles.ax_kgain,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('K%d', i));
end
legend(handles.ax_levels, 'Location', 'best');
yline(handles.ax_innov, 0, 'k:');
end

function twin_plot_update(handles, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref)
%TWIN_PLOT_UPDATE  Refresh all live plot windows with current history.
%   Call every time step inside the main loop, followed by drawnow.

for i = 1:3
    set(handles.h_meas(i),  'XData', t_vec, 'YData', y_hist(i,:));
    set(handles.h_pred(i),  'XData', t_vec, 'YData', y_pred_hist(i,:));
    set(handles.h_u(i),     'XData', t_vec, 'YData', u_hist(i,:));
    set(handles.h_innov(i), 'XData', t_vec, 'YData', innov_hist(i,:));
    set(handles.h_kg(i),    'XData', t_vec, 'YData', K_diag_hist(i,:));
end

% MPC horizon preview
cla(handles.ax_horizon);
t_hor = t_vec(end) + (0:size(mpc_traj,2)-1);
colors = {'b','r','g'};
for i = 1:3
    plot(handles.ax_horizon, t_hor, mpc_traj(i,:), [colors{i} '-o'], ...
        'DisplayName', sprintf('Pool %d predicted', i));
    yline(handles.ax_horizon, y_ref(i), [colors{i} ':']);
end
legend(handles.ax_horizon, 'Location', 'best');

drawnow;
end
