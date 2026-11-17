function out = v_sim_v6_pid(varargin)
% V_SIM_V6_PID V6 time-domain evidence from the closed-loop PID model.
%
%   out = v_sim_v6_pid()
%
% Reads results/sim_manual_pid_traj.mat, which run_manual_pid produced by
% simulating model/uacs_manual_pid.slx. Criteria, architecture.md V6
%   rise time of every excited loop within a factor of the bandwidth target
%   overshoot below 20 percent
%   no sustained oscillation, taken as a tail ripple below 2 percent of the step
%
% Only the scenario that excites a loop is used for that loop's verdict: a loop
% that was never commanded has no rise time to report, and that is not a pass.
% The bandwidth targets are the P.ctrl.bw fields the gains were designed
% against, so the comparison is against the frozen numbers and not against the
% achieved response.

ip = inputParser;
ip.addParameter('rise_factor', 3.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('overshoot_max_pct', 20.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('ripple_max_pct', 2.0, @(x) isnumeric(x) && x > 0);
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
opt = ip.Results;
verbose = logical(opt.verbose);

root = sim_root();
file = fullfile(root, 'results', 'sim_manual_pid_traj.mat');
assert(exist(file, 'file') == 2, 'v_sim_v6:NoData', ...
   '%s is missing; run run_manual_pid first', file);
S = load(file);
traj = S.traj;
[P_init, params_src] = sim_init_params(); %#ok<ASGLU>
P = evalin('base', 'P');

loops = struct( ...
   'name', {'phi', 'theta', 'r', 'V', 'h'}, ...
   'scenario', {'roll_step', 'pitch_step', 'yaw_rate_step', 'speed_step', 'altitude_step'}, ...
   'bw', {P.ctrl.bw.att_phi_rad_s, P.ctrl.bw.att_theta_rad_s, P.ctrl.bw.rate_r_rad_s, ...
          P.ctrl.bw.speed_rad_s, P.ctrl.bw.height_rad_s}, ...
   'unit', {'deg', 'deg', 'rad/s', 'm/s', 'm'});

out = struct('loop', {}, 'scenario', {}, 'rise_time_s', {}, 'rise_target_s', {}, ...
   'overshoot_pct', {}, 'settling_time_s', {}, 'error_ss', {}, 'ripple_pct', {}, ...
   'delta', {}, 'pass', {});
for k = 1:numel(loops)
   L = loops(k);
   idx = find(strcmp({traj.name}, L.scenario), 1);
   assert(~isempty(idx), 'v_sim_v6:NoScenario', ...
      'scenario %s is not in %s', L.scenario, file);
   m = traj(idx).metrics.(L.name);
   e = struct();
   e.loop = L.name;
   e.scenario = L.scenario;
   e.rise_time_s = m.rise_time_s;
   e.rise_target_s = 2.2 / L.bw;
   e.overshoot_pct = m.overshoot_pct;
   e.settling_time_s = m.settling_time_s;
   e.error_ss = m.error_ss;
   e.ripple_pct = m.tail_ripple_pct;
   e.delta = m.delta;
   e.pass = (abs(m.delta) > 0) && (m.rise_time_s < opt.rise_factor * e.rise_target_s) ...
      && (m.overshoot_pct < opt.overshoot_max_pct) && (m.tail_ripple_pct < opt.ripple_max_pct);
   out(end + 1) = e; %#ok<AGROW>

   if verbose
      fprintf(['V6 %-6s %-15s rise %8.4f s (target %8.4f, factor %d)  os %6.2f%%  ' ...
         'settle %7.3f s  err %10.3e  ripple %6.3f%%  -> %s\n'], ...
         L.name, L.scenario, m.rise_time_s, e.rise_target_s, opt.rise_factor, ...
         m.overshoot_pct, m.settling_time_s, m.error_ss, m.tail_ripple_pct, ...
         local_verdict(e.pass));
   end
end

bad = out(~[out.pass]);
assert(isempty(bad), 'v_sim_v6:Failed', ...
   'loops failing V6: %s', strjoin({bad.loop}, ', '));
end

function s = local_verdict(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
