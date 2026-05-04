function twin_plot_update(handles, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref)
%TWIN_PLOT_UPDATE  Refresh all live plot windows with current history.
%   Call every time step inside the main loop.

for i = 1:3
    set(handles.h_meas(i),  'XData', t_vec, 'YData', y_hist(i,:));
    set(handles.h_pred(i),  'XData', t_vec, 'YData', y_pred_hist(i,:));
    set(handles.h_u(i),     'XData', t_vec, 'YData', u_hist(i,:));
    set(handles.h_innov(i), 'XData', t_vec, 'YData', innov_hist(i,:));
    set(handles.h_kg(i),    'XData', t_vec, 'YData', K_diag_hist(i,:));
end

t_hor = t_vec(end) + (0:size(mpc_traj,2)-1);
for i = 1:3
    set(handles.h_hor(i), 'XData', t_hor, 'YData', mpc_traj(i,:));
end

drawnow;
end
