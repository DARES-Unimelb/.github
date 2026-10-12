function ok = sim_stage_run(name, fn)
% SIM_STAGE_RUN Run one named stage of a job, log its verdict, never throw.
%
% A shared MATLAB session can drop a request while the job keeps running, so a
% long job must leave a readable trace of which stage it reached and which one
% failed. Each stage is timed and its verdict printed immediately, and the
% stages after a failure still run because they may be independent.
%
% Returns true when the stage completed, so a job can summarise at the end.

fprintf('\n===== STAGE %s =====\n', name);
t0 = tic;
try
   fn();
   ok = true;
   fprintf('STAGE %s -> OK (%.1f s)\n', name, toc(t0));
catch err
   ok = false;
   fprintf('STAGE %s -> FAILED (%.1f s): %s | %s\n', name, toc(t0), ...
      err.identifier, err.message);
   st = err.stack;
   for j = 1:min(4, numel(st))
      fprintf('   at %s line %d\n', st(j).name, st(j).line);
   end
   if ~isempty(err.cause)
      for j = 1:numel(err.cause)
         fprintf('   cause: %s\n', err.cause{j}.message);
      end
   end
end
end
