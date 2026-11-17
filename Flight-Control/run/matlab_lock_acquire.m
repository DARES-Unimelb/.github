function lock_id = matlab_lock_acquire(varargin)
% MATLAB_LOCK_ACQUIRE Serialise the shared MATLAB session between team members.
%
% One MATLAB process serves one MCP request at a time, so two members running heavy jobs at
% once starve each other. The lock is a directory plus a JSON owner file, created with
% mkdir, which is atomic on a local filesystem: whoever creates the directory owns it.
%
% The lock is held for a whole job, not for a single call. Wrap a job, not a line.
%
% Usage:
%   id = matlab_lock_acquire();                  % default owner from getenv('DSH_AGENT')
%   id = matlab_lock_acquire('owner', 'ash-plant', 'timeout_s', 1800);
%   ... run the job ...
%   matlab_lock_release(id);
%
% Options: owner, timeout_s (1800), poll_s (10), stale_s (900), dir.
% Throws MATLAB_LOCK:Timeout if the lock cannot be taken within timeout_s.

ip = inputParser;
ip.addParameter('owner', local_owner(), @(x) ischar(x) || isstring(x));
ip.addParameter('timeout_s', 1800, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.addParameter('poll_s', 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.addParameter('stale_s', 900, @(x) isnumeric(x) && isscalar(x) && x > 0);
ip.addParameter('dir', local_lock_dir(), @(x) ischar(x) || isstring(x));
ip.parse(varargin{:});

owner = char(ip.Results.owner);
timeout_s = ip.Results.timeout_s;
poll_s = ip.Results.poll_s;
stale_s = ip.Results.stale_s;
lockdir = char(ip.Results.dir);

t_start = tic;
waited_reported = false;
while true
    if mkdir(lockdir)
        % Directory created by this process: the lock is ours.
        stamp = struct('owner', owner, 'pid', local_pid(), ...
            'acquired', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
            'acquired_secs', posixtime(datetime('now')));
        local_write_json(fullfile(lockdir, 'owner.json'), stamp);
        lock_id = struct('dir', lockdir, 'owner', owner, 'stamp', stamp);
        fprintf('LOCK acquired by %s after %.1f s\n', owner, toc(t_start));
        return;
    end

    % Someone holds it. Liveness is judged on the heartbeat when the holder publishes
    % one, and falls back to the acquisition time otherwise.
    info = local_read_json(fullfile(lockdir, 'owner.json'));
    hb = local_read_json(fullfile(lockdir, 'heartbeat.json'));
    age = inf;
    holder = 'unknown';
    if ~isempty(info)
        holder = info.owner;
        if isfield(info, 'acquired_secs')
            age = posixtime(datetime('now')) - info.acquired_secs;
        end
    end
    if ~isempty(hb) && isfield(hb, 'beat_secs')
        age = posixtime(datetime('now')) - hb.beat_secs;
    end
    if age > stale_s
        fprintf('LOCK held by %s, no activity for %.0f s (stale_s %.0f s): breaking it\n', ...
            holder, age, stale_s);
        local_force_release(lockdir);
        continue;
    end

    if toc(t_start) > timeout_s
        error('MATLAB_LOCK:Timeout', ...
            'could not acquire MATLAB lock within %.0f s; holder is %s (age %.0f s)', ...
            timeout_s, holder, age);
    end
    if ~waited_reported
        fprintf('LOCK busy, held by %s (age %.0f s), waiting up to %.0f s\n', ...
            holder, age, timeout_s);
        waited_reported = true;
    end
    pause(poll_s);
end
end

%% Local helpers
function d = local_lock_dir()
% Lock lives inside the project so it is visible to all members and survives a restart.
here = fileparts(mfilename('fullpath'));
d = fullfile(here, '..', 'results', '_matlab_lock');
end

function o = local_owner()
o = getenv('DSH_AGENT');
if isempty(o)
    o = 'unknown';
end
end

function p = local_pid()
% matlab.processid is the supported form; feature('getpid') is the undocumented fallback.
try
    p = matlab.processid('current');
catch
    try
        p = feature('getpid');
    catch
        p = -1;
    end
end
end

function local_write_json(f, s)
fid = fopen(f, 'w');
if fid < 0
    return;
end
fprintf(fid, '{\n  "owner": "%s",\n  "pid": %d,\n  "acquired": "%s",\n  "acquired_secs": %.3f\n}\n', ...
    s.owner, s.pid, s.acquired, s.acquired_secs);
fclose(fid);
end

function s = local_read_json(f)
s = [];
if ~exist(f, 'file')
    return;
end
try
    s = jsondecode(fileread(f));
catch
    s = [];
end
end

function local_force_release(lockdir)
% Remove the whole lock tree; rmdir with 's' clears the owner file too.
try
    rmdir(lockdir, 's');
catch
end
end
