out = fullfile('C:\Ash\Projects\DARES_UACS\MATLAB\results', '_ping.log');
fid = fopen(out, 'w');
fprintf(fid, 'ping ok %s\n', datestr(now, 'HH:MM:SS'));
fclose(fid);
disp('ping script ran');
