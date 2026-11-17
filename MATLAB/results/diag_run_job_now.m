% diag_run_job_now Run a named job inline, without the blocking run_job wrapper,
% and print only the verdict markers. For use through a channel with a call
% timeout: the job itself can take minutes, the caller only needs the markers.
if ~exist('job_name', 'var') || isempty(job_name)
   job_name = 'control_design';
end
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
fname = ['uacs_job_' job_name];
logfile = fullfile(root, 'results', ['job_' job_name '.log']);
fid = fopen(logfile, 'w');
fprintf(fid, '=== JOB START   %s\n', job_name);
fprintf(fid, 'owner ash-control2  at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fclose(fid);
diary_file = fullfile(root, 'results', ['job_' job_name '.diary']);
if exist(diary_file, 'file'), delete(diary_file); end
diary(diary_file); diary on;
t0 = tic;
ok = true; err = [];
try
   evalin('base', fname);
catch ME
   ok = false; err = ME;
end
elapsed = toc(t0);
diary off;
fid = fopen(logfile, 'a');
if exist(diary_file, 'file')
   try
      fwrite(fid, fileread(diary_file));
   catch
      fprintf(fid, '(diary unreadable)\n');
   end
end
if ok
   fprintf(fid, '=== JOB DONE    %s  (%.1f s)\n', job_name, elapsed);
else
   fprintf(fid, '!! JOB FAILED   %s  (%.1f s)\n', job_name, elapsed);
   fprintf(fid, '   %s | %s\n', err.identifier, err.message);
   for k = 1:numel(err.stack)
      fprintf(fid, '   at %s line %d\n', err.stack(k).name, err.stack(k).line);
   end
end
fclose(fid);
disp(fileread(logfile));
