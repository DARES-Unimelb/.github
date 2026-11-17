% V12_CHECKCODE Static analysis of the V12 deliverable files.
root = fileparts(fileparts(mfilename('fullpath')));
files = {'sim\sim_plant_ref_aero.m', 'sim\sim_push_ref_vars.m', 'sim\sim_push_vars.m', ...
   'sim\sim_guarded_sim.m', 'sim\sim_build_plant_ref.m', 'sim\sim_run_plant_cmd.m', ...
   'sim\sim_ref_calibrate.m', 'sim\sim_v12_measure.m', 'sim\sim_v12_trim.m', ...
   'verify\v_sim_v12_crosscheck.m', 'verify\v_sim_all.m', 'verify\v_sim_v11_models.m', 'run\uacs_job_sim_v12.m', 'run\build_uacs_models.m'};
fid = fopen(fullfile(root, 'results', 'v12_checkcode.txt'), 'w');
for k = 1:numel(files)
   p = fullfile(root, files{k});
   fprintf(fid, '=== %s\n', files{k});
   if exist(p, 'file') ~= 2
      fprintf(fid, '   MISSING\n');
      continue;
   end
   try
      msgs = checkcode(p, '-struct');
   catch err
      fprintf(fid, '   checkcode failed: %s\n', err.message);
      continue;
   end
   if isempty(msgs)
      fprintf(fid, '   clean\n');
   end
   for j = 1:numel(msgs)
      if isfield(msgs, 'id')
         tag = msgs(j).id;
      else
         tag = '';
      end
      fprintf(fid, '   line %4d  %-12s %s\n', msgs(j).line, tag, msgs(j).message);
   end
end
fclose(fid);
disp('CHECKCODE DONE');
