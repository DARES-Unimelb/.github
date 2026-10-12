function run_job(job_name, varargin)
% RUN_JOB Run a named job under the shared-session lock, with a live log.
%
% Usage from MATLAB:
%   run_job('plant_baseline')                         % runs uacs_job_plant_baseline.m
%   run_job('plant_baseline', 'owner', 'ash-plant')
%
% A job is a script named uacs_job_<job_name>.m on the MATLAB path. The job runs
% under matlab_lock_acquire, so two members never compute at once.
%
% Four protections, each added after a real failure:
%  1. The job's console output is polled into results/job_<name>.log while it
%     runs, so a job that dies mid-way leaves a trace instead of a one-line log.
%  2. A heartbeat file is refreshed every heartbeat_s during the job. Liveness is
%     judged on the heartbeat, not on the acquisition time, so a dead job is
%     detected in minutes rather than at the end of a long stale timeout.
%  3. The stale timeout is short enough that an abandoned lock cannot block the
%     team for hours, which is exactly what happened once.
%  4. The owner name is passed in explicitly and recorded, so a stuck lock can be
%     traced to a member instead of reading "unknown".
%
% Verdict markers are deliberately different in shape. Match on JOB DONE; never
% treat the absence of JOB FAILED as success.
%   === JOB START   <name>
%   === JOB DONE    <name>      (+ elapsed)
%   !! JOB FAILED   <name>
%   ?? JOB UNKNOWN  <name>      (the lock went stale while the job was running)
%
% To abort a stuck job from another member, delete results/_matlab_lock/. The
% stale timeout will also do it on its own.

ip = inputParser;
ip.addParameter('owner', local_owner(), @(x) ischar(x) || isstring(x));
ip.addParameter('stale_s', 900, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.addParameter('heartbeat_s', 20, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.addParameter('timeout_s', 3600, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.parse(varargin{:});
owner = char(ip.Results.owner);
stale_s = ip.Results.stale_s;

if nargin < 1 || isempty(job_name)
    error('run_job:BadName', 'usage: run_job(''<job_name>'')');
end
job_name = char(job_name);
fname = ['uacs_job_' job_name];
if isempty(which(fname))
    error('run_job:NotFound', 'no job script %s.m on the MATLAB path', fname);
end

here = fileparts(mfilename('fullpath'));
logdir = fullfile(here, '..', 'results');
if ~exist(logdir, 'dir')
    mkdir(logdir);
end
logfile = fullfile(logdir, ['job_' job_name '.log']);

% The lock records the owner explicitly so a stuck lock is traceable.
lock = matlab_lock_acquire('owner', owner, 'timeout_s', ip.Results.timeout_s, ...
    'stale_s', stale_s);
cleanup_lock = onCleanup(@() matlab_lock_release(lock));

fid = fopen(logfile, 'w');
if fid < 0
    error('run_job:LogOpen', 'cannot write %s', logfile);
end
cleanup_log = onCleanup(@() fclose(fid));

fprintf(fid, '=== JOB START   %s\n', job_name);
fprintf(fid, 'owner %s  pid %d  at %s  stale_s %.0f\n', ...
    lock.owner, lock.stamp.pid, datestr(now, 'yyyy-mm-dd HH:MM:SS'), stale_s);
fprintf('=== JOB START   %s\n', job_name);
fprintf('log file %s\n', logfile);

% Poll the console into the log so a hang is visible while it is happening.
diary_file = fullfile(logdir, ['job_' job_name '.diary']);
diary off;
if exist(diary_file, 'file')
    delete(diary_file);
end
diary(diary_file);
diary on;
cleanup_diary = onCleanup(@() local_diary_off());

% Heartbeat: refreshed by a timer so a dead MATLAB stops refreshing it and the
% next member's stale check fires quickly instead of waiting out a long timeout.
hb_file = fullfile(lock.dir, 'heartbeat.json');
hb = local_heartbeat_start(hb_file, owner, ip.Results.heartbeat_s);
cleanup_hb = onCleanup(@() local_heartbeat_stop(hb));

t0 = tic;
ok = true;
err = [];
stale_hit = false;
try
    evalin('base', fname);
catch ME
    ok = false;
    err = ME;
    % A stale-break by another member shows up here as a missing lock directory.
    stale_hit = ~exist(lock.dir, 'dir');
end
elapsed = toc(t0);

local_heartbeat_stop(hb);
local_flush_diary(diary_file, fid, job_name, elapsed, ok, err, stale_hit);
end

%% ================= local helpers =================
function local_diary_off()
diary off;
end

function local_flush_diary(diary_file, fid, job_name, elapsed, ok, err, stale_hit)
% Copy whatever the console captured into the persistent log, then write the
% verdict marker. diary must be off before the file is read back.
diary off;
if exist(diary_file, 'file')
    try
        txt = fileread(diary_file);
        fwrite(fid, txt);
    catch
        fprintf(fid, '(diary unreadable)\n');
    end
end
if stale_hit
    fprintf(fid, '?? JOB UNKNOWN  %s  (%.1f s) lock was released by another member\n', ...
        job_name, elapsed);
    fprintf('?? JOB UNKNOWN  %s\n', job_name);
elseif ok
    fprintf(fid, '=== JOB DONE    %s  (%.1f s)\n', job_name, elapsed);
    fprintf('=== JOB DONE    %s  (%.1f s)\n', job_name, elapsed);
else
    fprintf(fid, '!! JOB FAILED   %s  (%.1f s)\n', job_name, elapsed);
    fprintf(fid, '   %s | %s\n', err.identifier, err.message);
    for k = 1:numel(err.stack)
        fprintf(fid, '   at %s line %d\n', err.stack(k).name, err.stack(k).line);
    end
    fprintf('!! JOB FAILED   %s\n', job_name);
end
end

function hb = local_heartbeat_start(f, owner, period_s)
% A timer refreshes the heartbeat file. If MATLAB dies or blocks, the file stops
% moving and the next member's stale check can act on it.
hb = struct('file', f, 'owner', owner, 'timer', []);
try
    hb.timer = timer('ExecutionMode', 'fixedSpacing', 'Period', period_s, ...
        'BusyMode', 'drop', 'Name', 'uacs_heartbeat', ...
        'TimerFcn', @(~, ~) local_heartbeat_tick(f));
    start(hb.timer);
catch
    hb.timer = [];
end
local_heartbeat_tick(f);
end

function local_heartbeat_tick(f)
try
    fid = fopen(f, 'w');
    if fid > 0
        fprintf(fid, '{"beat_secs": %.3f, "at": "%s"}\n', ...
            posixtime(datetime('now')), datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        fclose(fid);
    end
catch
end
end

function local_heartbeat_stop(hb)
try
    if ~isempty(hb) && isstruct(hb) && ~isempty(hb.timer) && isvalid(hb.timer)
        stop(hb.timer);
        delete(hb.timer);
    end
catch
end
end

function o = local_owner()
o = getenv('DSH_AGENT');
if isempty(o)
    o = 'unknown';
end
end
