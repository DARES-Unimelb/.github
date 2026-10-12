function out = v_sim_all(varargin)
% V_SIM_ALL Run every Simulink side acceptance case and report the verdicts.
%
%   out = v_sim_all()
%
% Each case is executed in turn and its failure is caught, so one failure still
% leaves the other verdicts on the record. The function then raises if anything
% failed: the table is evidence, the assertion is the criterion.
%
% V12 needs the Aerospace Blockset, and it is added to the list only when the
% toolbox is present: an absent toolbox is a SKIP with its reason, never a silent
% pass, which is the rule in architecture.md section 9. When the toolbox is there
% but the cross check has not been run yet, the case raises v_sim_v12:NoData and
% is reported as SKIP for the same reason: no measurement, no verdict.

ip = inputParser;
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
verbose = logical(ip.Results.verbose);

addpath(fullfile(sim_root(), 'verify'), '-begin');
v = [];
try
   v = ver('aeroblks');
catch
end
cases = {@v_sim_v11_models, @v_sim_v10_stepsize, @v_sim_v6_pid, @v_sim_v9_wind, ...
   @v_sim_v13_consistency};
if ~isempty(v)
   cases{end + 1} = @v_sim_v12_crosscheck;
end
skip_ids = {'v_sim_v10:NoData', 'v_sim_v6:NoData', 'v_sim_v9:NoData', ...
   'v_sim_v11:NoCensus', 'v_sim_v12:NoData', 'v_sim_v13:NoData'};
out = struct('case', {}, 'status', {}, 'detail', {});
for k = 1:numel(cases)
   fn = func2str(cases{k});
   try
      cases{k}();
      out(end + 1) = struct('case', fn, 'status', 'PASS', 'detail', ''); %#ok<AGROW>
   catch err
      detail = sprintf('%s | %s', err.identifier, err.message);
      if any(strcmp(err.identifier, skip_ids))
         status = 'SKIP';
      else
         status = 'FAIL';
      end
      out(end + 1) = struct('case', fn, 'status', status, 'detail', detail); %#ok<AGROW>
   end
   if verbose
      fprintf('%-20s %s  %s\n', out(end).case, out(end).status, out(end).detail);
   end
end

if isempty(v)
   out(end + 1) = struct('case', 'v_sim_v12_crosscheck', 'status', 'SKIP', ...
      'detail', 'Aerospace Blockset is not installed');
end

n_fail = sum(strcmp({out.status}, 'FAIL'));
if verbose
   fprintf('V-sim summary: %d pass, %d fail, %d skip\n', ...
      sum(strcmp({out.status}, 'PASS')), n_fail, sum(strcmp({out.status}, 'SKIP')));
end
assert(n_fail == 0, 'v_sim_all:Failed', '%d Simulink acceptance cases failed', n_fail);
end
