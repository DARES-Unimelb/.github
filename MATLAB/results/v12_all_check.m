% V12_ALL_CHECK Run the whole Simulink acceptance suite and capture the verdict
% table, so the V12 wiring in verify/v_sim_all.m is checked rather than assumed.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;
txt = evalc(['try, v_sim_all(); catch err, ' ...
   'fprintf(1, ''v_sim_all raised %s | %s\n'', err.identifier, err.message); end']);
fid = fopen(fullfile(root, 'results', 'v12_all_check.txt'), 'w');
fprintf(fid, '%s\n', txt);
fclose(fid);
disp('ALL CHECK DONE');
