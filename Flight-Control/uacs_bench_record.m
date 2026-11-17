function uacs_bench_record(mdl)
% UACS_BENCH_RECORD Archive the run that just stopped and write the named report.
%
% Set as the bench model StopFcn. Writes results/bench_last_run.mat and
% results/bench_report.txt, plus a timestamped copy under results/runs/.

if nargin < 1 || isempty(mdl)
   mdl = bdroot;
end
keep = 5;                     % runs kept under results/runs/
root = fileparts(mfilename('fullpath'));
out_dir = fullfile(root, 'results');
run_dir = fullfile(out_dir, 'runs');
trace = fullfile(out_dir, 'bench_run_trace.txt');
stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
tag = datestr(now, 'yyyymmdd_HHMMSS');
fid = fopen(trace, 'a', 'n', 'UTF-8');
try
   % Recording must never interrupt the flight, so a failure only leaves a trace.
   ls = [];
   src = '';
   if evalin('base', 'exist(''out'', ''var'')') == 1
      o = evalin('base', 'out');
      if isa(o, 'Simulink.SimulationOutput')
         try
            ls = o.logsout;
            src = 'out.logsout';
         catch
         end
      end
   end
   if isempty(ls) && evalin('base', 'exist(''logsout'', ''var'')') == 1
      ls = evalin('base', 'logsout');
      src = 'logsout';
   end
   if isempty(ls)
      fprintf(fid, '%s  %s  停止时工作区里既没有 out 也没有 logsout，本轮没记\n', stamp, mdl);
   else
      if ~exist(run_dir, 'dir')
         mkdir(run_dir);
      end
      last = fullfile(out_dir, 'bench_last_run.mat');
      arch = fullfile(run_dir, ['bench_' tag '.mat']);
      logsout = ls;
      save(last, 'logsout');
      save(arch, 'logsout');
      fprintf(fid, '%s  %s  已记 %s，归档 %s，来源 %s\n', stamp, mdl, last, arch, src);
      % Write the named report as well, one copy fixed and one archived.
      try
         rep = bench_report();
         copyfile(rep.file, fullfile(run_dir, ['bench_' tag '.txt']));
      catch e
         fprintf(fid, '   打表失败：%s | %s\n', e.identifier, e.message);
      end
      % Keep only the newest keep runs; the timestamp names sort chronologically.
      d = dir(fullfile(run_dir, 'bench_*.mat'));
      if numel(d) > keep
         [~, ix] = sort({d.name});
         for k = 1:(numel(d) - keep)
            old = fullfile(run_dir, d(ix(k)).name);
            delete(old);
            old_txt = strrep(old, '.mat', '.txt');
            if exist(old_txt, 'file')
               delete(old_txt);
            end
         end
         fprintf(fid, '   归档清理：删掉最旧的 %d 轮，保留最近 %d 轮\n', ...
            numel(d) - keep, keep);
      end
   end
catch e
   try
      fprintf(fid, '   记录失败：%s | %s\n', e.identifier, e.message);
   catch
   end
end
if fid > 0
   fclose(fid);
end
end
