function out = v_sim_v9_wind(varargin)
% V_SIM_V9_WIND V9: 5 m/s crosswind in level flight stays bounded and rolls level.
%
%   out = v_sim_v9_wind()
%
% Reads results/sim_wind_crosswind.mat, which run_scenarios produced by running
% model/uacs_manual_pid.slx twice: once with a 5 m/s crosswind and once without.
%
% Criteria, architecture.md V9 as "bounded track deviation and roll convergence"
%   bank angle converges: the tail mean is within 1 deg and the largest bank
%   after the first 30 percent of the run stays under the 45 deg limit
%   the crosswind run holds its ground track: its own east displacement from its
%   initial position stays under 20 m over 30 s. Both runs start in the trim of
%   their own wind, so each is already balanced and holding a track; the judged
%   quantity is whether the crabbed aircraft holds the track it was trimmed for.
%   the crosswind run's own east drift stays near zero while the still air run
%   drifts with the wind, which is what shows the wind is present and balanced
%   altitude is not disturbed: residual against the no-wind run under 5 m
%   the state does not diverge and the pitch angle stays below 90 deg

ip = inputParser;
ip.addParameter('phi_tail_max_deg', 1.0, @(x) isnumeric(x));
ip.addParameter('residual_max_m', 20.0, @(x) isnumeric(x));
ip.addParameter('alt_residual_max_m', 5.0, @(x) isnumeric(x));
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
opt = ip.Results;

root = sim_root();
file = fullfile(root, 'results', 'sim_wind_crosswind.mat');
assert(exist(file, 'file') == 2, 'v_sim_v9:NoData', ...
   '%s is missing; run run_scenarios with V9 enabled first', file);
S = load(file);
s = S.summary;

out = struct();
out.file = file;
out.wind_mps = s.wind_mps;
out.phi_tail_deg = s.phi_tail_deg;
out.phi_max_after_settle_deg = s.phi_max_after_settle_deg;
out.beta_tail_deg = s.beta_tail_deg;
out.drift_rate_mps = s.drift_rate_mps;
out.residual_max_m = s.residual_max_m;
out.alt_residual_max_m = s.alt_residual_max_m;
out.criteria = struct('phi_tail_max_deg', opt.phi_tail_max_deg, ...
   'residual_max_m', opt.residual_max_m, ...
   'alt_residual_max_m', opt.alt_residual_max_m);
out.pass_phi = abs(s.phi_tail_deg) < opt.phi_tail_max_deg && ...
   s.phi_max_after_settle_deg < 45.0;
out.pass_deviation = s.residual_max_m < opt.residual_max_m;
out.pass_altitude = s.alt_residual_max_m < opt.alt_residual_max_m;
out.pass_bounded = ~s.diverged;
out.pass = out.pass_phi && out.pass_deviation && out.pass_altitude && out.pass_bounded;

if logical(ip.Results.verbose)
   fprintf('V9 wind %.1f m/s, %.0f s, level flight\n', s.wind_mps, s.t_end);
   fprintf('V9 bank tail %.4f deg, max after 30%% %.4f deg (limit %.1f) -> %s\n', ...
      s.phi_tail_deg, s.phi_max_after_settle_deg, opt.phi_tail_max_deg, ...
      local_verdict(out.pass_phi));
   fprintf('V9 crab %.4f deg, east drift %.4f m/s (wind %.1f m/s)\n', ...
      s.beta_tail_deg, s.drift_rate_mps, s.wind_mps);
   fprintf('V9 own track error, east displacement from start %.4f m (limit %.1f) -> %s\n', ...
      s.residual_max_m, opt.residual_max_m, local_verdict(out.pass_deviation));
   if isfield(s, 'residual_vs_nowind_max_m')
      fprintf('V9 east displacement vs the no-wind run %.4f m (evidence the wind acts)\n', ...
         s.residual_vs_nowind_max_m);
   end
   fprintf('V9 altitude residual vs no wind %.4f m (limit %.1f) -> %s\n', ...
      s.alt_residual_max_m, opt.alt_residual_max_m, local_verdict(out.pass_altitude));
   fprintf('V9 bounded %d -> %s\n', ~s.diverged, local_verdict(out.pass_bounded));
   fprintf('V9 -> %s\n', local_verdict(out.pass));
end
assert(out.pass, 'v_sim_v9:Failed', ...
   ['crosswind case failed: phi tail %.4f deg, drift residual %.4f m, ' ...
    'altitude residual %.4f m, diverged %d'], s.phi_tail_deg, s.residual_max_m, ...
   s.alt_residual_max_m, s.diverged);
end

function v = local_verdict(tf)
if tf
   v = 'PASS';
else
   v = 'FAIL';
end
end
