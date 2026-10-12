function out = v_sim_v10_stepsize(varargin)
% V_SIM_V10_STEPSIZE V10: ode45 and ode4 agree on the same command sequence.
%
%   out = v_sim_v10_stepsize()
%
% Reads the trajectories run_scenarios saved. The two solvers use different time
% grids, so the variable-step trajectory is interpolated onto the fixed-step
% grid with pchip before the comparison; the linear interpolation numbers are
% reported next to it as a sensitivity, not as the criterion.
%
% Criterion, architecture.md V10: the largest per-state difference is below 1
% percent. The denominator is the state's own peak-to-peak excursion in the
% fixed-step run, which is the strict reading; for a state that barely moves the
% reference scale of sim_state_scale is used instead, and which states those are
% is reported.

ip = inputParser;
ip.addParameter('tol', 0.01, @(x) isnumeric(x) && x > 0);
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
verbose = logical(ip.Results.verbose);

root = sim_root();
file = fullfile(root, 'results', 'sim_v10_stepsize.mat');
assert(exist(file, 'file') == 2, 'v_sim_v10:NoData', ...
   '%s is missing; run run_scenarios with V10 enabled first', file);
S = load(file);
v = S.v10;

out = struct();
out.file = file;
out.interp = v.interp;
out.max_rel_ptp = v.max_rel_ptp;
out.max_rel_ptp_linear = v.max_rel_ptp_linear;
out.max_rel_scale = v.max_rel_scale;
out.tol = ip.Results.tol;
out.pass = v.max_rel_ptp < ip.Results.tol;
out.worst_state = v.state_names{v.worst_state_rel_ptp};
out.table = table(v.state_names(:), v.diff_max_abs(:), v.state_ptp(:), ...
   v.rel_ptp(:), v.rel_ptp_linear(:), v.rel_scale(:), ...
   'VariableNames', {'state', 'max_diff', 'ptp', 'rel_ptp', 'rel_ptp_linear', 'rel_scale'});

if verbose
   fprintf('V10 tol %.3g, denominator peak-to-peak of the ode4 run\n', ip.Results.tol);
   disp(out.table);
   fprintf('V10 max rel %.6e at state %s (linear interpolation would give %.6e)\n', ...
      out.max_rel_ptp, out.worst_state, out.max_rel_ptp_linear);
   fprintf('V10 max rel to the reference scale %.6e\n', out.max_rel_scale);
   fprintf('V10 -> %s\n', local_verdict(out.pass));
end
assert(out.pass, 'v_sim_v10:Failed', ...
   'ode45 and ode4 differ by %.4f%% of the trajectory peak-to-peak, limit %.4f%%', ...
   out.max_rel_ptp * 100, ip.Results.tol * 100);
end

function s = local_verdict(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
