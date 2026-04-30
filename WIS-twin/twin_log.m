function twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, triggered)
%TWIN_LOG_WRITE  Append one row to the twin log CSV.
%
%   Columns: epoch, y1_meas, y2_meas, y3_meas, y1_pred, y2_pred, y3_pred,
%            innov1, innov2, innov3, u_mpc1, u_mpc2, u_mpc3, triggered

header = 'epoch,y1_meas,y2_meas,y3_meas,y1_pred,y2_pred,y3_pred,innov1,innov2,innov3,u_mpc1,u_mpc2,u_mpc3,triggered';

write_header = ~isfile(log_file);

fid = fopen(log_file, 'a');
if fid == -1
    error('twin_log_write: cannot open %s', log_file);
end

if write_header
    fprintf(fid, '%s\n', header);
end

row = [epoch, y_meas(:)', y_pred(:)', innov(:)', u_mpc(:)', triggered];
fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%d\n', row);
fclose(fid);
end
