function matlab_lock_release(lock_id)
% MATLAB_LOCK_RELEASE Release the shared-session lock taken by matlab_lock_acquire.
%
% Safe to call twice, and with a stale or empty handle: the lock is only removed when the
% owner file still names this owner, so a member cannot release somebody else's lock.

if nargin < 1 || isempty(lock_id) || ~isstruct(lock_id) || ~isfield(lock_id, 'dir')
    return;
end
lockdir = lock_id.dir;
if ~exist(lockdir, 'dir')
    return;
end
info = [];
of = fullfile(lockdir, 'owner.json');
if exist(of, 'file')
    try
        info = jsondecode(fileread(of));
    catch
        info = [];
    end
end
if ~isempty(info) && isfield(info, 'owner') && ~strcmp(info.owner, lock_id.owner)
    warning('matlab_lock:ForeignLock', ...
        'lock is owned by %s, not %s; leaving it alone', info.owner, lock_id.owner);
    return;
end
try
    rmdir(lockdir, 's');
    fprintf('LOCK released by %s\n', lock_id.owner);
catch ME
    warning('matlab_lock:ReleaseFailed', '%s', ME.message);
end
end
