% V12_LAUNCH Start a V12 job from a timer.
%
% The MCP channel serves one request at a time and a client call is dropped when
% it runs long, so the job is not started inline: the timer fires once the
% launcher has returned, run_job takes the shared MATLAB lock, and a dropped
% client call cannot take a running job down with it.
%
% The job name comes from results/v12_job_name.txt, defaulting to sim_v12, so a
% diagnostic job can be started without editing this file.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;

name_file = fullfile(root, 'results', 'v12_job_name.txt');
job_name = 'sim_v12';
if exist(name_file, 'file')
   txt = strtrim(fileread(name_file));
   if ~isempty(txt)
      job_name = txt;
   end
end

stat = fullfile(root, 'results', 'v12_launch_status.txt');
fid = fopen(stat, 'w');
if fid > 0
   fprintf(fid, 'launcher reached %s, arming %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), job_name);
   fclose(fid);
end
fprintf('v12 launcher in %s, job %s\n', root, job_name);

fcn = @(~, ~) run_job(job_name, 'owner', 'ash-v12', 'stale_s', 1800, 'timeout_s', 3600);
j = timer('Name', 'v12_job_launcher', 'ExecutionMode', 'singleShot', ...
   'StartDelay', 2, 'TimerFcn', fcn);
start(j);
disp('V12 LAUNCHED');
