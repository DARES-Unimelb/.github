% V12_V11_CHECK Static check of every delivered file, then rebuild the reference
% model so it registers its census entry, then read all four models back from
% disk through V11.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
fid = fopen(fullfile(root, 'results', 'v12_v11_check.txt'), 'w');

files = {'sim\sim_plant_ref_aero.m', 'sim\sim_push_ref_vars.m', 'sim\sim_push_vars.m', ...
   'sim\sim_guarded_sim.m', 'sim\sim_build_plant_ref.m', 'sim\sim_run_plant_cmd.m', ...
   'sim\sim_ref_calibrate.m', 'sim\sim_v12_measure.m', 'sim\sim_v12_trim.m', ...
   'verify\v_sim_v12_crosscheck.m', 'verify\v_sim_v11_models.m', 'verify\v_sim_all.m', ...
   'run\uacs_job_sim_v12.m', 'run\build_uacs_models.m'};
fprintf(fid, '=== checkcode ===\n');
for k = 1:numel(files)
   p = fullfile(root, files{k});
   msgs = checkcode(p, '-struct');
   noisy = {};
   for j = 1:numel(msgs)
      if ~isempty(strfind(msgs(j).message, 'datestr')) || ...
            ~isempty(strfind(msgs(j).message, '''now''')) || ...
            ~isempty(strfind(msgs(j).message, 'clear'' with the ''functions'''))
         continue;
      end
      noisy{end + 1} = sprintf('    line %4d  %s', msgs(j).line, msgs(j).message); %#ok<AGROW>
   end
   if isempty(noisy)
      fprintf(fid, '  %-34s clean (project style warnings filtered)\n', files{k});
   else
      fprintf(fid, '  %-34s\n', files{k});
      for j = 1:numel(noisy)
         fprintf(fid, '%s\n', noisy{j});
      end
   end
end

fprintf(fid, '\n=== census before the rebuild ===\n');
cf = fullfile(root, 'results', 'sim_build_census.mat');
if exist(cf, 'file')
   S = load(cf);
   fprintf(fid, '  fields: %s\n', strjoin(fieldnames(S.census)', ', '));
else
   fprintf(fid, '  no census file\n');
end

fprintf(fid, '\n=== rebuild the reference model ===\n');
[P, src] = sim_init_params(); %#ok<ASGLU>
assignin('base', 'sim_ts_ctrl', P.sim.Ts_ctrl);
assignin('base', 'sim_t_end', P.sim.t_end_s);
tr = sim_v12_trim(P);
assignin('base', 'trim', tr);
assignin('base', 'sim_x0', tr.x_star(:));
info = sim_build_plant_ref(P);
fprintf(fid, '  built %s, top %d total %d lines %d\n', info.model, ...
   info.census.top_count, info.census.total_count, info.census.line_count);

fprintf(fid, '\n=== census after the rebuild ===\n');
S = load(cf);
fprintf(fid, '  fields: %s\n', strjoin(fieldnames(S.census)', ', '));

fprintf(fid, '\n=== V11 ===\n');
try
   out = v_sim_v11_models();
   fprintf(fid, '  V11 PASS, %d models\n', numel(out));
   for k = 1:numel(out)
      fprintf(fid, '  %-18s top %3d/%3d total %3d/%3d names %d -> %d\n', out(k).model, ...
         out(k).top_count, out(k).expected_top, out(k).total_count, out(k).expected_total, ...
         out(k).names_match, out(k).pass);
   end
catch err
   fprintf(fid, '  V11 FAILED %s | %s\n', err.identifier, err.message);
end
fclose(fid);
disp('V11 CHECK DONE');
