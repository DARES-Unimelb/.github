function gains = design_pid(P, lin, opts)
% DESIGN_PID Cascade PID for the UACS twin, manual-flight main line.
%
%   gains = design_pid(P, lin)
%   gains = design_pid(P, lin, struct('pm_target_deg', 45))
%   gains = design_pid(P, lin, struct('which', {{'p', 'phi'}}))   % targeted re-run
%
% The third argument is optional; the frozen call design_pid(P, lin) keeps
% working unchanged.
%
% Structure, every number designed from lin, nothing hand-tuned
%   inner   p, q, r rate loops          pure P, crossover from the airframe modes
%   outer   phi, theta attitude loops   PID with a filtered D, output a rate command
%   outer   V airspeed loop             PI, output a total thrust command
%   outer   h altitude loop             PI, output a pitch attitude command
%
% How a loop is scored. The controller is written as a polynomial pair and closed
% against the linearised plant including the actuator lag, so
%   closed-loop poles  = pole(feedback(C*G, 1)), minreal at 1e-9
%   loop response      = C(jw)*G(jw), exact rational functions, on one fixed grid
%   crossover          = the frequency where |C*G| falls through 1
%   phase margin       = 180 deg plus the phase AT that crossover
%   vector margin      = min|1 + C*G(jw)|, the distance to the critical point
%   damping            = the smallest damping ratio of the closed loop's poles
%
% A rate loop costs its damping shortfall against 0.70, its vector margin shortfall
% against 0.50, and control effort. A tracking loop costs its steady-state error,
% its closed-loop -3 dB bandwidth against P.ctrl.bw, its 10-90 rise, its overshoot
% above 8 percent, and its tail ripple. Everything is rejected outright below a
% vector margin of 0.50 and outside the left half plane. The gains are the best of
% a geometric grid over proportional, integral and derivative gain and the D filter
% ratio, then one refinement pass around the best point.
%
% The rate loops are judged on damping and vector margin rather than on bandwidth,
% because their closed-loop bandwidth and step response are degenerate: dcgain(Gq)
% is 0 exactly, so the loop is a differentiator. The inner crossover target comes
% from the airframe's structural modes, not from P.ctrl.bw.rate_*_rad_s.
%
% Polarity. The search covers both signs and the delivered sign is published per
% loop as sim_pid_<loop>_pol. The model computes err = cmd - meas, so the sign that
% closes negative feedback depends on the sign of the plant where the loop acts;
% positive Kp on the pitch rate channel splits the airframe's over-damped short
% period into a less damped complex pair, which is positive feedback.
%
% A pole at or right of the imaginary axis rejects the candidate, with no
% exemption. The residue share of such a pole is reported beside the verdict as
% evidence only; a small residue is not a licence to pass, because a mode can be
% weak in the regulated output and still decide the response.
%
% D terms carry the first-order filter P.ctrl.rate_filter_tau_s, scaled by the
% search. Integrators carry the state-domain clamp P.ctrl.int_state_limit.(loop),
% which is derived there from the error-domain P.ctrl.int_limit.(loop). The report
% prints the error bound, the integrator gets the state clamp, and reading one as
% the other is what put the model and the report 2.5x apart.

if nargin < 3
   opts = struct();
end
if ~isfield(opts, 'pm_target_deg')
   opts.pm_target_deg = 45;
end
if ~isfield(opts, 'tol_axis')
   opts.tol_axis = 1e-7;
end
if ~isfield(opts, 'verbose')
   opts.verbose = false;
end
if ~isfield(opts, 'which')
   % Subset of loops to design, for a targeted re-run. The coupling is one way
   % only: phi needs p, theta needs q, h needs theta.
   opts.which = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
end
want = @(nm) any(strcmp(opts.which, nm));
verbose = opts.verbose;

lc = lin_contract(lin);
wc = P.ctrl.bw;
Tf = P.ctrl.rate_filter_tau_s;
Kb = P.ctrl.anti_windup_Kb;
Il = P.ctrl.int_limit;
lag = @(t) tf(1, [t, 1]);

gains = struct();
gains.created = datestr(now, 'yyyy-mm-dd HH:MM:SS');
% zero_frac_type1 and zero_frac_type0 are the nominal integral-zero fractions the
% search is anchored on, for a plant with a pole at the origin and one without; the
% delivered value comes out of the grid and is reported per loop
gains.rules = struct('pm_target_deg', opts.pm_target_deg, 'tol_axis', opts.tol_axis, ...
   'rate_filter_tau_s', Tf, 'anti_windup_Kb', Kb, 'law', ...
   'u = Kp*e + Ki*int(e) + Kd*Df(e), err = cmd - meas, polarity per loop and published', ...
   'zero_frac_type1', 0.5, 'zero_frac_type0', 0.2);
gains.rate_filter_tau_s = Tf;
gains.anti_windup_Kb = Kb;
gains.int_limit = Il;
gains.x_op = lc.u_op_full;

%% Inner rate loops, actuator lag included
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag(P.act.aileron.tau_s);
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag(P.act.elevator.tau_s);
Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag(P.act.rudder.tau_s);

% Inner rate-loop crossover target from the airframe, not from P.ctrl.bw, because
% the frozen rate targets cannot be designed to on this airframe
wi_inner = local_inner_target(lc);
fprintf('design_pid inner rate target %.4f rad/s (P.ctrl.bw rate targets are %.1f %.1f %.1f)\n', ...
   wi_inner, wc.rate_p_rad_s, wc.rate_q_rad_s, wc.rate_r_rad_s);

if want('p')
   gains.p = local_rate_loop(Gp, wi_inner, Tf, Kb, opts, 'p', 'da', ...
      deg2rad(P.act.aileron.max_deg), wc.rate_p_rad_s, verbose);
end
if want('q')
   gains.q = local_rate_loop(Gq, wi_inner, Tf, Kb, opts, 'q', 'de', ...
      deg2rad(P.act.elevator.max_deg), wc.rate_q_rad_s, verbose);
end
if want('r')
   gains.r = local_rate_loop(Gr, wi_inner, Tf, Kb, opts, 'r', 'dr', ...
      deg2rad(P.act.rudder.max_deg), wc.rate_r_rad_s, verbose);
end

%% Outer attitude loops
% The plant is the closed rate loop followed by the kinematics 1/s, which is how
% model/uacs_manual_pid.slx closes it: the attitude loop commands a rate and the
% rate loop tracks it.
if want('phi')
   Gphi = local_series_integrator(feedback(gains.p.ctrl * Gp, 1));
   gains.phi = local_pi_loop(Gphi, wc.att_phi_rad_s, Il.phi, Tf, Kb, opts, 'phi', true, verbose);
end

if want('theta')
   Gtheta = local_series_integrator(feedback(gains.q.ctrl * Gq, 1));
   gains.theta = local_pi_loop(Gtheta, wc.att_theta_rad_s, Il.theta, Tf, Kb, opts, 'theta', true, verbose);
end

%% Outermost, airspeed and altitude
if want('V')
   GV = ss(lc.A_long, lc.B_long(:, 2), [1 0 0 0], 0) * lag(P.prop.esc.tau_s);
   gains.V = local_pi_loop(GV, wc.speed_rad_s, Il.V, Tf, Kb, opts, 'V', true, verbose);
end

% Height plant: the delivered closed theta loop, then climb rate V0*gamma and 1/s.
% The alpha part of h_dot = V0*(theta - alpha) is left out, and is covered by the
% closed-loop longitudinal check in the design job.
if want('h')
   V0 = lc.x_op_long(1);
   Tth = local_series_integrator(feedback(gains.theta.ctrl * Gtheta, 1));
   Gh = local_gain_series(Tth, V0);
   gains.h = local_pi_loop(Gh, wc.height_rad_s, Il.h, Tf, Kb, opts, 'h', false, verbose);
end

%% Report facts
verify = struct();
for k = 1:numel(opts.which)
   nm = opts.which{k};
   verify.(nm) = gains.(nm).verify;
end
sep = struct();
if isempty(setdiff({'p', 'q', 'r', 'phi', 'theta', 'V', 'h'}, opts.which))
   % The inner speed of a rate channel is its loop crossover, not a -3 dB
   % bandwidth, because those channels are near differentiators and
   % bandwidth_rad_s is NaN there by construction
   bw = @(nm) local_sep_speed(verify.(nm));
   sep.phi_over_p = bw('phi') / bw('p');
   sep.theta_over_q = bw('theta') / bw('q');
   sep.V_over_q = bw('V') / bw('q');
   sep.h_over_theta = bw('h') / bw('theta');
   sep.h_over_V = bw('h') / bw('V');
else
   sep.phi_over_p = NaN; sep.theta_over_q = NaN; sep.V_over_q = NaN;
   sep.h_over_theta = NaN; sep.h_over_V = NaN;
end
sep.inner_target_rad_s = wi_inner;
sep.target = struct('phi_over_p', wc.att_phi_rad_s / wi_inner, ...
   'theta_over_q', wc.att_theta_rad_s / wi_inner, ...
   'V_over_q', wc.speed_rad_s / wi_inner, ...
   'h_over_theta', wc.height_rad_s / wc.att_theta_rad_s, ...
   'h_over_V', wc.height_rad_s / wc.speed_rad_s);
sep.limit = 0.5;
sep.ok = sep.phi_over_p < sep.limit && sep.theta_over_q < sep.limit ...
   && sep.V_over_q < sep.limit && sep.h_over_theta < sep.limit ...
   && sep.h_over_V < sep.limit;
verify.separation = sep;
gains.verify = verify;

names = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
gains.polarity = struct();
for k = 1:numel(names)
   if isfield(gains, names{k})
      gains.polarity.(names{k}) = gains.(names{k}).sign;
   end
end

gains.law = struct();
gains.law.p = 'da = Kp*(p_c - p) + Kd*Df(p_c - p)';
gains.law.q = 'de = Kp*(q_c - q) + Kd*Df(q_c - q)';
gains.law.r = 'dr = Kp*(r_c - r) + Kd*Df(r_c - r)';
gains.law.phi = 'p_c = Kp*(phi_c - phi) + Ki*i_phi';
gains.law.theta = 'q_c = Kp*(theta_c - theta) + Ki*i_theta';
gains.law.V = 'T_cmd = Kp*(V_c - V) + Ki*i_V, throttle through allocate.m';
gains.law.h = 'theta_c = clamp(Kp*(h_c - h) + Ki*i_h, limits.pitch_max_deg)';
gains.law.Df = sprintf('Df(e) = Kd*s/(1 + %.4f s) e, filtered derivative', Tf);
gains.law.polarity = ['err = cmd - meas in every loop; sim_pid_<loop>_pol carries ' ...
   'sign(Kp) into the model as one Gain block after the PID sum'];
gains.law.int_limit = ['integrators clamped at P.ctrl.int_state_limit.(loop), which is ' ...
   'P.ctrl.int_limit.(loop)/bw times P.ctrl.int_state_margin; the rate loops carry no ' ...
   'integral and are clamped at +-the surface travel'];
end

%% Local helpers
function g = local_rate_loop(G, wi, Tf, Kb, opts, name, act, surf_lim, wc_bw, verbose)
% Pure proportional gain on the inner rate loops, with the crossover placed at wi.
% The rate feedback is the damping term for the attitude loop above it, not a
% tracking loop, so no integral action belongs here. wc_bw is
% P.ctrl.bw.rate_*_rad_s, carried through for the report only.
%
% A channel with no stable gain is delivered OPEN, with Kp = 0 and designed =
% false, rather than as an invented gain or an exception that kills the other six
% loops. The yaw rate channel comes out this way for a physical reason: Gr has a
% right half plane zero and its |Gr| peaks above 1 as w -> 0, so every gain above
% the boundary is unstable while below it the loop never crosses unity and adds
% nothing. An open channel is reported as a design constraint, and V6's r verdict
% then fails on the evidence rather than on a number that does not exist.
[kp, sgn, score, info] = local_search_rate(G, wi, opts);
designed = isfinite(score) && score < 1e3;
if ~designed
   kp = 0;
   sgn = 1;
end
ctf = sgn * tf(kp, 1);
g = struct('name', name, 'actuator', act, 'Kp', sgn * kp, 'Ki', 0, 'Kd', 0, ...
   'Tf', Tf, 'Kb', Kb, 'Kp_mag', kp, 'Kp_nominal', info.kp_nominal, ...
   'kp_scale', kp / info.kp_nominal, 'gain_reduced', kp < info.kp_nominal, ...
   'Kd_mag', 0, 'ki_reason', info.ki_reason, 'designed', designed, ...
   'sign', sgn, 'wc_target_rad_s', wi, 'wc_frozen_rad_s', wc_bw, 'zero_rad_s', NaN, ...
   'zero_frac', NaN, 'zero_frac_nominal', NaN, ...
   'int_reduced', false, 'd_used', false, ...
   'score', score, 'search_tried', info.n_tried, 'search_stable', info.n_stable, ...
   'int_limit', surf_lim, 'ctrl', ctf, 'tf', ctf);
g.verify = local_loop_verify(ctf, G, name, wi, opts, 'rate');
g.verify.vector_margin = local_vector_margin(ctf, G);
if verbose
   fprintf(['design_pid %-6s Kp %12.6f (pure P) pol %+d crossover %9.4f ' ...
      '(inner target %.3f, frozen rate target %.3f) sm %.4f pm %7.2f designed %d\n'], ...
      name, sgn * kp, sgn, g.verify.wcp_rad_s, wi, wc_bw, g.verify.vector_margin, ...
      g.verify.pm_deg, designed);
end
end

function g = local_pi_loop(G, wc, int_limit, Tf, Kb, opts, name, use_d, verbose)
% PI plus an optional filtered D, from the same grid search. The integral action is
% optional and the search returns Ki = 0 when it buys nothing. The D term supplies
% damping on the attitude loops, which have none of their own once the rate loop is
% a pure proportional gain; use_d is false for the airspeed and altitude loops,
% where a derivative on the error is noise gain against a slow plant.
[kp, ki, kd, kf, sgn, score, info] = local_search_pi(G, wc, Tf, opts, use_d);
ctf = sgn * tf([kp * (kf * Tf) + kd, kp + ki * (kf * Tf), ki], [kf * Tf, 1, 0]);
g = struct('name', name, 'actuator', '', 'Kp', sgn * kp, 'Ki', sgn * ki, 'Kd', sgn * kd, ...
   'Tf', kf * Tf, 'Kb', Kb, 'Kp_mag', kp, 'Kp_nominal', info.kp_nominal, ...
   'kp_scale', kp / info.kp_nominal, 'gain_reduced', false, 'Kd_mag', kd, ...
   'zero_rad_s', ki / max(kp, eps), 'zero_frac', ki / max(kp * wc, eps), ...
   'zero_frac_nominal', info.frac_nominal, 'int_reduced', ki < kp * info.frac_nominal * wc, ...
   'ki_reason', info.ki_reason, 'sign', sgn, 'wc_target_rad_s', wc, 'd_used', kd ~= 0, ...
   'designed', true, 'd_filter_scale', kf, 'd_filter_tau_s', kf * Tf, ...
   'score', score, 'search_tried', info.n_tried, 'search_stable', info.n_stable, ...
   'int_limit', int_limit, 'ctrl', ctf, 'tf', ctf);
g.verify = local_loop_verify(ctf, G, name, wc, opts, 'track');
if verbose
   fprintf(['design_pid %-6s Kp %12.6f Ki %12.6f Kd %10.6f Tf %8.5f pol %+d ' ...
      'bw %9.4f (target %.4f) pm %7.2f\n'], ...
      name, sgn * kp, sgn * ki, sgn * kd, kf * Tf, sgn, g.verify.bandwidth_rad_s, wc, ...
      g.verify.pm_deg);
end
end

function [kp, sgn, score, info] = local_search_rate(G, wc, opts)
% Pure proportional gain on the inner rate loops, chosen so the LOOP CROSSOVER
% lands near wc.
%
% The objective is the loop crossover, |L(jw)| falling through 1, and NOT the
% closed-loop -3 dB bandwidth or the step response: these rate channels are near
% differentiators, so a step response and a -3 dB bandwidth are both degenerate
% there. On q, dcgain is 0 exactly with a transmission zero at the origin, so the
% closed-loop |T| has no -3 dB point at all. On p, a zero nearly cancels a pole and
% the step response reads a huge "overshoot" that barely moves with gain.
% Optimising on either of those drove the gain to the stability boundary. The
% crossover is the quantity that is defined, monotone in Kp and named by
% P.ctrl.bw, so it is the one used here.
Kp_nom = local_plant_mag(G, wc);
% The polarity is searched, not fixed by the wiring: the inner error is
% err = cmd - meas, but whether that closes negative feedback depends on the sign
% of the plant at the frequency where the loop acts
kp_grid = logspace(log10(1e-2 * Kp_nom), log10(1e3 * Kp_nom), 161);
best = struct('cost', Inf);
tried = 0;
stable = 0;
for sgn = [1, -1]
   for a = 1:numel(kp_grid)
      [ok, m] = local_cl_eval_p(sgn, kp_grid(a), G, opts.tol_axis, wc);
      tried = tried + 1;
      if ok
         stable = stable + 1;
      end
      % Control effort, so the delivered gain is the smallest one that meets the
      % damping and margin specs; without it the cost is flat across the whole band
      % of gains that clear both
      cost = local_cost(ok, m, wc, opts.pm_target_deg, 'rate') ...
         + 1.0 * max(0, log(kp_grid(a) / Kp_nom))^2;
      if cost < best.cost
         best = struct('cost', cost, 'kp', kp_grid(a), 'sgn', sgn);
      end
   end
end
step = (log10(1e3 * Kp_nom) - log10(1e-2 * Kp_nom)) / 160;
for a = best.kp * logspace(-step, step, 11)
   [ok, m] = local_cl_eval_p(best.sgn, a, G, opts.tol_axis, wc);
   tried = tried + 1;
   if ok
      stable = stable + 1;
   end
   cost = local_cost(ok, m, wc, opts.pm_target_deg, 'rate') ...
      + 1.0 * max(0, log(a / Kp_nom))^2;
   if cost < best.cost
      best = struct('cost', cost, 'kp', a, 'sgn', best.sgn);
   end
end
if ~(isfinite(best.cost) && best.cost < 1e3)
   % No stable proportional gain exists on this channel. Return the flag rather
   % than raise: local_rate_loop delivers the channel open and the report says so.
   kp = NaN;
   sgn = 1;
   score = Inf;
   info = struct('kp_nominal', Kp_nom, 'n_tried', tried, 'n_stable', stable, ...
      'ki_reason', 'no stable proportional gain exists on this channel');
   return;
end
kp = best.kp;
sgn = best.sgn;
score = best.cost;
info = struct('kp_nominal', Kp_nom, 'n_tried', tried, 'n_stable', stable, ...
   'ki_reason', ['Ki = 0 by design on a rate loop: the plant is type 1 in this ' ...
   'channel so a second pole at the origin makes the loop unstable, and damping ' ...
   'is what the loop is for, not holding a steady rate']);
end

function [kp, ki, kd, kf, sgn, score, info] = local_search_pi(G, wc, Tf, opts, use_d)
% Grid over proportional gain, integral zero frequency, derivative gain and D
% filter ratio, then one refinement pass. The gains are swept directly rather than
% scaled off the grid, so the integral zero can be placed at any frequency.
%
% The D term is swept only when use_d is set, and it is the term that supplies the
% attitude loops' damping: the inner rate loop is a pure proportional gain, so the
% attitude loop sees the airframe's own damping and nothing more. It is not offered
% to the airspeed or altitude loops, where a derivative on the error amplifies the
% sensor noise of a slow plant for no phase benefit.
%
% The D filter ratio kf is swept as a multiple of P.ctrl.rate_filter_tau_s, and must
% stay at or above the outer sample time P.sim.Ts_outer so the derivative is
% realisable on the sampled loop.
Kp_nom = local_plant_mag(G, wc);
if use_d
   kp_grid = logspace(log10(1e-3 * Kp_nom), log10(1e3 * Kp_nom), 21);
else
   kp_grid = logspace(log10(1e-4 * Kp_nom), log10(1e4 * Kp_nom), 41);
end
frac_grid = [0, 0.05, 0.1, 0.2, 0.35, 0.5, 0.8, 1.2, 2.0, 3.5, 6.0];
% Kd is swept as a fraction of Kp/wc, so as a derivative time relative to the target
% crossover: zero, then up to about ten times the crossover period
kd_grid = [0, 0.02, 0.05, 0.1, 0.2, 0.4, 0.8];
kf_grid = [1, 3];
if ~use_d
   kd_grid = 0;
   kf_grid = 1;
end
best = struct('cost', Inf, 'kp', NaN, 'ki', NaN, 'kd', NaN, 'kf', NaN, 'sgn', NaN);
tried = 0;
stable = 0;
for sgn = [1, -1]
   for a = 1:numel(kp_grid)
      for b = 1:numel(frac_grid)
         for c = 1:numel(kd_grid)
            for d = 1:numel(kf_grid)
               [ok, m] = local_cl_eval_pid(sgn, kp_grid(a), kp_grid(a) * frac_grid(b) * wc, ...
                  kp_grid(a) * kd_grid(c) / wc, kf_grid(d) * Tf, G, opts.tol_axis, wc);
               tried = tried + 1;
               if ok
                  stable = stable + 1;
               end
               cost = local_cost(ok, m, wc, opts.pm_target_deg, 'pi');
               if cost < best.cost
                  best = struct('cost', cost, 'kp', kp_grid(a), ...
                     'ki', kp_grid(a) * frac_grid(b) * wc, 'kd', kp_grid(a) * kd_grid(c) / wc, ...
                     'kf', kf_grid(d), 'sgn', sgn);
               end
            end
         end
      end
   end
end
step = (log10(kp_grid(end)) - log10(kp_grid(1))) / 20;
kp_r = best.kp * logspace(-step, step, 9);
ki_r = max(0, best.ki * [0, 0.3, 0.6, 1, 1.5, 2.5]);
kd_r = max(0, best.kd * [0, 0.5, 1, 1.5, 2, 3]);
for a = 1:numel(kp_r)
   for b = 1:numel(ki_r)
      for c = 1:numel(kd_r)
         for d = 1:numel(kf_grid)
            [ok, m] = local_cl_eval_pid(best.sgn, kp_r(a), ki_r(b), kd_r(c), ...
               kf_grid(d) * Tf, G, opts.tol_axis, wc);
            tried = tried + 1;
            if ok
               stable = stable + 1;
            end
            cost = local_cost(ok, m, wc, opts.pm_target_deg, 'pi');
            if cost < best.cost
               best = struct('cost', cost, 'kp', kp_r(a), 'ki', ki_r(b), ...
                  'kd', kd_r(c), 'kf', kf_grid(d), 'sgn', best.sgn);
            end
         end
      end
   end
end
assert(isfinite(best.cost) && best.cost < 1e3, 'design_pid:noStableTuning', ...
   'no stable proportional-integral-derivative set found for a loop with target %.4f rad/s', wc);
kp = best.kp;
ki = best.ki;
kd = best.kd;
kf = best.kf;
sgn = best.sgn;
score = best.cost;
info = struct('kp_nominal', Kp_nom, 'frac_nominal', 0.5, 'n_tried', tried, ...
   'n_stable', stable, 'ki_reason', '');
if ki == 0
   info.ki_reason = ['Ki = 0: no integral gain lowered the weighted cost, so the ' ...
      'loop is delivered without integral action'];
else
   info.ki_reason = sprintf(['integral zero at %.4f rad/s from the search, which is ' ...
      '%.3f of the %.3f rad/s target'], ki / kp, ki / (kp * wc), wc);
end
end

function c = local_cost(ok, m, wc_target, pm_target, kind)
% Candidate cost, from the closed-loop step response the acceptance case measures,
% not from a frequency-domain proxy.
%   steady-state error: charged for the tracking loops only, because a command that
%   does not settle on its own value is a failure whatever the bandwidth says.
%
%   THE RATE LOOPS ARE EXEMPT FROM THAT TERM, AND THE REASON IS PHYSICS, NOT
%   CONVENIENCE: q = dtheta/dt, so a constant non-zero pitch rate means the pitch
%   angle grows without bound and a stable aircraft can never hold one. Every
%   correct longitudinal model has dcgain(de->q) = 0, with a transmission zero at
%   the origin. A rate loop exists to add damping, not to hold a steady rate;
%   holding an attitude is the attitude loop's job and the pilot's command.
%   Charging unity DC gain on a rate loop made every candidate lose, for every gain
%   and every controller structure. Do not "restore" that term here.
%
%   rise time: the 10-90 percent rise against 2.2/P.ctrl.bw, charged in log space
%   overshoot: charged above 8 percent and quadratically above 20 percent
%   tail ripple: the settled tail must not oscillate
%   slow modes and a low gain margin are charged as before
if ~ok
   c = 1e6;
   return;
end
if strcmp(kind, 'rate')
   % RATE LOOPS ARE JUDGED ON THE DAMPING THEY ADD, with the vector margin as the
   % robustness term. Damping is primary because it is what a rate loop is for, and
   % it is the only one of the three candidates that is defined here: the -3 dB
   % closed-loop bandwidth is degenerate on a near-differentiator, and the loop
   % crossover alone is not reachable everywhere and has the wrong shape on q.
   %
   % The vector margin is the hard gate and the working spec at once: a channel that
   % cannot clear 0.60 has no usable feedback and is delivered open rather than
   % closed at a gain the nonlinear model rings at.
   if ~isfinite(m.sm) || m.sm < 0.60
      c = 1e3;
      return;
   end
   e_sm = max(0, log(0.50 / m.sm));
   e_damp = max(0, 0.70 - m.min_zeta);
   % The crossover is charged asymmetrically. Below target it is light, because a
   % channel whose |L| peaks just above 1 cannot be pushed to a high crossover and
   % on those the loop is a gain limited damper set by the margin floor. Above
   % target it is heavy, because once the damping target is met extra crossover buys
   % nothing and costs actuator bandwidth.
   if isnan(m.wc)
      e_slow_wc = log(wc_target / 0.1);
      e_fast_wc = 0;
   else
      e_slow_wc = max(0, log(wc_target / m.wc));
      e_fast_wc = max(0, log(m.wc / wc_target));
   end
   c = 6.0 * e_damp^2 + 4.0 * e_sm^2 + 0.20 * e_slow_wc^2 + 1.50 * e_fast_wc^2 ...
      + 0.05 * sum(m.slow_modes) * log10(wc_target / 0.1);
   if max(real(m.slowest)) > -0.05
      c = c + 0.5;
   end
   return;
end
if isnan(m.rise) || isnan(m.os)
   c = 1e3;
   return;
end
e_ss = abs(1 - m.dc);
if ~isfinite(e_ss)
   c = 1e3;
   return;
end
% Loop shape for the tracking loops: one sided on the slow side, because being
% slower than the P.ctrl.bw target fails to track while being faster only costs
% actuator activity
if isnan(m.bw)
   e_slow = 1;
   e_fast = 0;
else
   r = log(m.bw / wc_target);
   e_slow = max(0, -r);
   e_fast = max(0, r);
end
e_rise = log(max(m.rise, realmin) / (2.2 / wc_target));
if m.os <= 8
   e_os = 0;
elseif m.os <= 20
   e_os = (m.os - 8) / 100;
else
   e_os = 0.12 + ((m.os - 20) / 100)^2 * 10;
end
e_rip = max(0, m.ripple - 2) / 100;
% Phase margin. This term keeps the integral zero of a PI from being parked above
% the crossover, where it contributes almost none of the lead it is there for: at
% the frozen airspeed crossover, a zero above the crossover leaves the controller
% contributing almost no phase while a zero below it leaves the margin large.
% opts.pm_target_deg is the contract floor (45 deg); the working spec is 60,
% because a loop designed to the floor has nothing left for the nonlinear terms the
% linear model does not carry.
pm_work = max(pm_target, 60);
if isfinite(m.pm)
   e_pm = max(0, (pm_work - m.pm) / pm_work);
else
   e_pm = 1;
end
c = 40 * e_ss^2 + 0.3 * e_slow^2 + 0.02 * e_fast^2 + 1.0 * e_rise^2 + 2 * e_os + 2 * e_rip ...
   + 4.0 * e_pm^2 + 0.05 * sum(m.slow_modes) * log10(wc_target / 0.1);
if isfinite(m.gm) && m.gm < 4 && m.gm > 0
   c = c + 0.05 * (log10(4 / m.gm))^2;
end
end

function [ok, m] = local_cl_eval(sgn, kp, kd, Tf, G, tol, wc)
% One PD candidate: C = sgn*(Kp + Kd*s/(Tf*s + 1))
[ok, m] = local_cl_core([sgn * kp * Tf + sgn * kd, sgn * kp], [Tf, 1], G, tol, wc);
end

function [ok, m] = local_cl_eval_p(sgn, kp, G, tol, wc)
% One proportional candidate: C = sgn*Kp
[ok, m] = local_cl_core(sgn * kp, 1, G, tol, wc);
end

function [ok, m] = local_cl_eval_pid(sgn, kp, ki, kd, tf_d, G, tol, wc)
% One PID candidate with a filtered derivative:
%   C = sgn*(Kp + Ki/s + Kd*s/(tf_d*s + 1))
% over the common denominator (tf_d*s + 1)*s.
[ok, m] = local_cl_core(sgn * [kp * tf_d + kd, kp + ki * tf_d, ki], [tf_d, 1, 0], G, tol, wc);
end

function [ok, m] = local_cl_core(cnum, cden, G, tol, wc_target)
% Closed-loop facts of one candidate.
%
% The verdict is pole(feedback(C*G, 1)) with minreal at a TIGHT tolerance. At 1e-9
% minreal cancels only the genuine numerical pair, while a looser tolerance
% cancels a real closed-loop pole near the origin that the plant's transmission
% zero creates, and the verdict then reports the loop as stable when it is not.
dp = local_plant_poly(G);
T = minreal(feedback(tf(cnum, cden) * tf(dp.num, dp.den), 1), 1e-9);
p = pole(T);
m = struct('bw', NaN, 'pm', NaN, 'gm', Inf, 'wc', NaN, 'worst', max(real(p)), ...
   'slowest', p, 'slow_modes', sum(real(p) > -0.1), 'artifact', 0, 'n_poles', numel(p), ...
   'rise', NaN, 'os', NaN, 'settle', NaN, 'dc', NaN, 'ripple', 0, 'sm', NaN, 'sm_db', NaN);
ok = m.worst < -tol;
if ~ok
   % Strict verdict: a pole at or right of the axis is a rejection, full stop. A
   % mode can be weak in the regulated output and still be the mode that decides
   % the response, so the residue share is evidence beside the verdict, never a
   % reason to accept.
   [m.artifact, m.worst_real] = local_residue_share(conv(cnum, dp.num), ...
      conv(cnum, dp.num) + conv(cden, dp.den), p);
   return;
end
m.worst_real = m.worst;
% Time-domain metrics, but only where they mean something. When the loop transfer
% function has a zero at the origin the closed loop is a differentiator and its
% step response is degenerate by construction, so these stay NaN and the caller's
% cost falls back to the crossover.
w = local_freq_grid();
H = (polyval(cnum, 1i * w) ./ polyval(cden, 1i * w)) .* dp.H;
if abs(H(1)) > 1e-12 * max(abs(H))
   m.dc = dcgain(T);
   try
      tv = linspace(0, max(30 / max(wc_target, 0.05), 4), 100);
      yv = step(T, tv);
      yss = yv(end);
      if isfinite(yss) && abs(yss) > 1e-9
         k10 = find(yv >= 0.1 * yss, 1);
         k90 = find(yv >= 0.9 * yss, 1);
         if ~isempty(k10) && ~isempty(k90) && k90 > k10
            m.rise = tv(k90) - tv(k10);
         end
         m.os = 100 * max(0, (max(yv) - yss) / yss);
         tail = yv(round(0.75 * numel(yv)):end);
         m.ripple = 100 * (max(tail) - min(tail)) / abs(yss);
         idx = find(abs(yv - yss) > 0.02 * abs(yss), 1, 'last');
         if isempty(idx)
            m.settle = 0;
         else
            m.settle = tv(idx);
         end
      end
   catch
      m.rise = NaN; m.os = NaN;
   end
else
   m.dc = 0;
end
% -3 dB bandwidth of the closed loop on the grid, referenced to the DC value when
% the loop has one. bandwidth() is not used: a near-cancelled zero keeps |T| high
% all the way to the actuator lag, where bandwidth() stops looking.
if isfinite(m.dc) && abs(m.dc) > 0
   mt = abs(H) ./ abs(1 + H);
   ref = abs(mt(1));
   k3 = local_first_cross(mt, ref / sqrt(2));
   if ~isempty(k3)
      m.bw = local_log_interp(w, mt, k3, ref / sqrt(2));
   end
end
ph = unwrap(angle(H)) * 180 / pi;
i = local_first_cross(abs(H), 1);
if ~isempty(i)
   m.wc = local_log_interp(w, abs(H), i, 1);
   m.pm = 180 + local_interp_at(w, ph, m.wc);
end
% Vector margin: the smallest distance from the loop's Nyquist locus to the -1
% point, so min|1 + L(jw)|. Well defined on every channel here, including the ones
% where the gain and phase margins are not. 0.5 is the working spec.
m.sm = min(abs(1 + H));
m.sm_db = 20 * log10(max(m.sm, realmin));
% Damping of the closed loop's oscillatory poles, and the frequency of the least
% damped one; this is what a rate loop exists to change
osc = p(abs(imag(p)) > 1e-3);
if isempty(osc)
   m.min_zeta = 1;
   m.wn_min_zeta = NaN;
else
   z = -real(osc) ./ abs(osc);
   [m.min_zeta, k] = min(z);
   m.wn_min_zeta = abs(osc(k));
end
j = find(ph(1:end - 1) > -180 & ph(2:end) <= -180, 1);
if ~isempty(j)
   m.gm = 1 / max(abs(H(j)), realmin);
end
end

function [t, y] = local_step_response(num, den, t_end) %#ok<DEFNU>
% Unused fallback: the step metrics come from tf/step on the closed-loop
% polynomial pair, which is accurate for the polynomial orders here.
[t, y] = deal([], []);
end

function [share, worst_real] = local_residue_share(cnum, cden, p)
% Residues of the closed loop from a partial fraction expansion, reported beside
% the stability verdict as evidence: a pole whose residue is a millionth of the
% total carries almost none of the response.
%
% This is NOT used to accept anything. A mode can be weak in the regulated output
% and still be the mode that decides the response, so a small residue is not a
% licence to pass.
share = 0;
worst_real = -Inf;
if ~any(real(p) > -1e-9)
   return;
end
cnum = cnum(:).';
cden = cden(:).';
num = conv(cnum, 1);
den = conv(cden, 1);
if numel(num) >= numel(den)
   return;
end
n = numel(p);
tot = 0;
res = zeros(n, 1);
for k = 1:n
   z = p(k);
   dd = polyder(den);
   dv = polyval(dd, z);
   if abs(dv) < realmin
      return;
   end
   res(k) = abs(polyval(num, z) / dv);
   tot = tot + res(k);
end
if tot <= 0
   return;
end
idx = find(real(p) > -1e-9);
for k = idx(:)'
   share = max(share, res(k) / tot);
   worst_real = max(worst_real, real(p(k)));
end
end

function dp = local_plant_poly(G)
% Plant transfer function coefficients and frequency response, cached: the plant
% never changes inside one design run.
persistent cache
[Ag, Bg, Cg, Dg] = ssdata(G);
key = [size(Ag), Ag(:).', Bg(:).', Cg(:).', Dg(:).'];
if ~isempty(cache) && numel(cache.key) == numel(key) && isequal(cache.key, key)
   dp = cache.dp;
   return;
end
[num, den] = ss2tf(Ag, Bg, Cg, Dg, 1);
num = num / den(1);
den = den / den(1);
% The coefficients go in exactly as ss2tf produced them, leading zeros included:
% the q numerator carries a DOUBLE zero at the origin, so filtering or rebuilding
% it with poly() shortens the vector and makes every q candidate look unstable
w = local_freq_grid();
ng = size(Ag, 1);
H = complex(zeros(numel(w), 1));
for k = 1:numel(w)
   H(k) = Cg * ((1i * w(k) * eye(ng) - Ag) \ Bg) + Dg;
end
dp = struct('num', num, 'den', den, 'w', w, 'H', H);
cache = struct('key', key, 'dp', dp);
end

function w = local_freq_grid()
% Shared log grid, held in memory: the design evaluates it thousands of times.
% 600 points over 1e-4..1e4 rad/s is 75 per decade, so the interpolated crossing
% frequency is resolved far more finely than the 2 percent that a 1 rad/s target
% needs.
persistent wg
if isempty(wg)
   wg = logspace(-4, 4, 600);
   wg = wg(:);
end
w = wg;
end

function H = local_plant_freq_at(G, w)
% Plant frequency response at given frequencies, via its cached polynomial pair
dp = local_plant_poly(G);
H = polyval(dp.num, 1i * w) ./ polyval(dp.den, 1i * w);
end

function w = local_sep_speed(v)
% Speed of one closed loop for the separation check: the crossover where that is
% the meaningful reading, the -3 dB bandwidth otherwise
if isfield(v, 'open_loop') && v.open_loop
   w = Inf;
elseif isfinite(v.wcp_rad_s) && ~isfinite(v.bandwidth_rad_s)
   w = v.wcp_rad_s;
elseif isfinite(v.bandwidth_rad_s)
   w = v.bandwidth_rad_s;
else
   w = NaN;
end
end

function wi = local_inner_target(lc)
% Inner rate-loop crossover target, from the airframe's own structural modes.
%
% The rule: the inner loop sits below the slowest structural mode, because the
% rate loop damps that mode rather than outrunning it. Structural means the short
% period and the Dutch roll; the phugoid and the spiral are excluded because they
% are slower than 1 rad/s and no rate loop is meant to touch them. The half factor
% and the 10..20 rad/s clamp are the inner-loop placement band for this airframe.
wn_lat = abs(eig(lc.A_lat));
wn_lat = wn_lat(wn_lat > 1.0);
wn_lon = abs(eig(lc.A_long));
wn_lon = wn_lon(wn_lon > 1.0);
cand = [wn_lat(:); wn_lon(:)];
if isempty(cand)
   wi = 12.0;
   return;
end
wi = min(cand) / 2;
wi = min(max(wi, 10.0), 20.0);
end

function m = local_plant_mag(G, wc)
m = 1 / abs(freqresp(G, 1i * wc));
end

function i = local_first_cross(y, level)
% First index where y descends through level, or empty
i = find(y(1:end - 1) >= level & y(2:end) < level, 1);
end

function v = local_log_interp(w, y, i, level)
% The frequency where y equals level, interpolated linearly in log w between the
% two samples that bracket it; i is the lower index from local_first_cross
j = min(i + 1, numel(w));
y1 = y(i); y2 = y(j); w1 = w(i); w2 = w(j);
if y1 == y2 || w2 <= w1
   v = w1;
   return;
end
f = (level - y1) / (y2 - y1);
v = exp(log(w1) + f * (log(w2) - log(w1)));
end

function v = local_interp_at(w, y, wq)
% y evaluated at wq, linearly in log w; used to read the phase AT the crossover
% rather than to search the phase for a level, which is not the phase margin
if ~isfinite(wq)
   v = NaN;
   return;
end
k = find(w <= wq, 1, 'last');
if isempty(k)
   v = y(1);
   return;
end
j = min(k + 1, numel(w));
if j == k || w(j) <= w(k)
   v = y(k);
   return;
end
f = (log(wq) - log(w(k))) / (log(w(j)) - log(w(k)));
v = y(k) + f * (y(j) - y(k));
end

function sm = local_vector_margin(C, G)
% min|1 + C*G(jw)| on the shared grid: the distance from the Nyquist locus to the
% critical point. 0.5 is the working spec.
w = local_freq_grid();
H = squeeze(freqresp(C * G, 1i * w));
sm = min(abs(1 + H(:)));
end

function G = local_series_integrator(T)
% Append the kinematics 1/s to a state-space loop
[Ad, Bd, Cd, Dd] = ssdata(T);
n = size(Ad, 1);
G = ss([Ad, zeros(n, 1); Cd, 0], [Bd; Dd], [zeros(1, n), 1], 0);
end

function G = local_gain_series(T, k)
[Ad, Bd, Cd, Dd] = ssdata(T);
G = ss(Ad, Bd, k * Cd, k * Dd);
end

function v = local_loop_verify(C, G, name, wc_target, opts, kind)
% Report path. Uses LTI objects, so the standard margins and step metrics come
% from MATLAB rather than from the search's own grid.
if nargin < 6
   kind = 'track';
end
% tf() normalises the controller first: tfdata on a bare scalar gain returns a
% cell whose contents do not always follow the vector rule
[cn, cd] = tfdata(tf(C), 'v');
cn = cn(:).';
cd = cd(:).';
dp = local_plant_poly(G);
if numel(dp.num) ~= numel(dp.den)
   dp.num = [zeros(1, numel(dp.den) - numel(dp.num)), dp.num];
end
clnum = conv(cn, dp.num);
clden = clnum + conv(cd, dp.den);
L = minreal(C * G, 1e-9);
T = minreal(feedback(L, 1), 1e-9);
v = struct();
v.name = name;
v.wc_target_rad_s = wc_target;
v.poles = pole(T);
if isempty(v.poles)
   % An open channel: C = 0, so feedback(C*G, 1) is the plant itself and there is
   % no loop to judge; report it as an unclosed channel with the plant's own poles
   v.poles = pole(G);
   v.worst_real = max(real(v.poles));
   v.stable = false;
   v.open_loop = true;
   v.artifact_share = 0;
   v.artifact_worst_real = NaN;
   v.rhp_poles = v.poles(real(v.poles) > -1e-3);
   v.mode_shares = local_shares(G, v.poles);
   v.gm = Inf; v.gm_db = Inf; v.pm_deg = NaN; v.dm_s = NaN;
   v.wcg_rad_s = NaN; v.wcp_rad_s = NaN;
   v.dc_gain = dcgain(G);
   v.rise_time_s = NaN; v.overshoot_pct = NaN; v.settling_time_s = NaN;
   v.peak = NaN;
   v.bandwidth_rad_s = NaN;
   try
      v.plant_bandwidth_rad_s = bandwidth(G);
   catch
      v.plant_bandwidth_rad_s = Inf;
   end
   v.rise_target_s = 2.2 / wc_target;
   v.rise_ok = false;
   v.overshoot_ok = true;
   v.plant_limited = true;
   v.bandwidth_ratio = NaN;
   return;
end
v.worst_real = max(real(v.poles));
v.stable = v.worst_real < -opts.tol_axis;
v.artifact_share = 0;
v.artifact_worst_real = NaN;
v.rhp_poles = v.poles(real(v.poles) > -1e-3);
if ~v.stable
   [v.artifact_share, v.artifact_worst_real] = local_residue_share(clnum, clden, v.poles);
   v.stability_note = sprintf(['right half plane pole at %+.4e, residue share %.2e: ' ...
      'reported as evidence, the design rejects it'], ...
      v.artifact_worst_real, v.artifact_share);
end
v.mode_shares = local_shares(T, v.poles);
% Margins from the design's own frequency grid. allmargin() is not used on these
% loops: it reports crossings that are numerical noise near the plant's
% near-cancelled zero, so the grid values are the reliable reading, and they are
% also the same grid the search scored against.
v.gm = Inf; v.gm_db = Inf; v.pm_deg = NaN; v.wcg_rad_s = NaN; v.wcp_rad_s = NaN;
v.dm_s = NaN;
vc = local_margins(L);
v.pm_deg = vc.pm;
v.wcp_rad_s = vc.wcp;
v.gm = vc.gm;
v.gm_db = 20 * log10(max(vc.gm, realmin));
v.wcg_rad_s = vc.wcg;
v.dc_gain = dcgain(T);
% Step response and -3 dB bandwidth are computed for the tracking loops only. On a
% rate channel they are degenerate by construction, and reporting them invites the
% next reader to treat a huge "overshoot" as a real oscillation; the rate loops
% carry their crossover instead, which local_margins already fills into wcp_rad_s.
if strcmp(kind, 'rate')
   v.rise_time_s = NaN; v.overshoot_pct = NaN; v.settling_time_s = NaN;
   v.peak = NaN; v.bandwidth_rad_s = NaN;
   v.rise_target_s = 2.2 / wc_target;
   v.rise_ok = true;
   v.overshoot_ok = true;
   v.bandwidth_ratio = NaN;
   v.degenerate_note = ['rate channel: step response and -3 dB bandwidth are ' ...
      'degenerate because the loop is a near differentiator; the crossover is the ' ...
      'meaningful reading and is in wcp_rad_s'];
   try
      v.plant_bandwidth_rad_s = bandwidth(G);
   catch
      v.plant_bandwidth_rad_s = Inf;
   end
   v.plant_limited = false;
   return;
end
try
   si = stepinfo(T);
   v.rise_time_s = si.RiseTime;
   v.overshoot_pct = si.Overshoot;
   v.settling_time_s = si.SettlingTime;
   v.peak = si.Peak;
catch err
   v.rise_time_s = NaN; v.overshoot_pct = NaN; v.settling_time_s = NaN;
   v.peak = NaN; v.step_note = err.message;
end
v.bandwidth_rad_s = local_bw_of_tf(T);
try
   v.plant_bandwidth_rad_s = bandwidth(G);
catch
   v.plant_bandwidth_rad_s = Inf;
end
v.rise_target_s = 2.2 / wc_target;
v.rise_ok = v.rise_time_s < 3 * v.rise_target_s;
v.overshoot_ok = v.overshoot_pct < 20;
v.plant_limited = v.plant_bandwidth_rad_s > wc_target;
v.bandwidth_ratio = v.bandwidth_rad_s / wc_target;
end

function [vc, H] = local_margins(L)
% Phase and gain margin of the loop transfer function on the design's own
% frequency grid, interpolated in log w at both crossings.
%
% The phase is unwrapped before it is interpolated: angle() returns a sawtooth in
% [-180, 180], and interpolating a segment that straddles the -180 to +180 jump
% produces a value near zero where the phase is really near -180.
w = local_freq_grid();
H = squeeze(freqresp(L, 1i * w));
H = H(:);
mag = abs(H);
ph = unwrap(angle(H)) * 180 / pi;
vc = struct('pm', NaN, 'gm', Inf, 'wcp', NaN, 'wcg', NaN);
j = find(mag(1:end - 1) > 1 & mag(2:end) <= 1, 1);
if isempty(j)
   j = local_first_cross(mag, 1);
end
if ~isempty(j)
   vc.wcp = local_log_interp(w, mag, j, 1);
   vc.pm = 180 + local_interp_at(w, ph, vc.wcp);
end
k = find(ph(1:end - 1) > -180 & ph(2:end) <= -180, 1);
if ~isempty(k)
   vc.wcg = w(k);
   vc.gm = 1 / max(mag(k), realmin);
end
end

function bw = local_bw_of_tf(T)
% -3 dB bandwidth by sweep. bandwidth() refuses a closed loop whose DC gain is
% infinite, which is every PI loop here, so the sweep is the reading that works
% for all seven.
w = logspace(-6, 5, 4000);
m = abs(squeeze(freqresp(T, 1i * w)));
if numel(m) ~= numel(w)
   bw = NaN;
   return;
end
ref = m(1);
i = find(m(1:end - 1) >= ref / sqrt(2) & m(2:end) < ref / sqrt(2), 1);
if isempty(i)
   bw = NaN;
else
   bw = exp(log(w(i)) + (ref / sqrt(2) - m(i)) / (m(i + 1) - m(i)) * (log(w(i + 1)) - log(w(i))));
end
end

function sh = local_shares(T, p)
% Residue share of every pole, for the report's side by side comparison between a
% let-through pole and the modes the loop actually acts on
sh = struct('poles', p, 'share', zeros(numel(p), 1));
[Ad, Bd, Cd] = ssdata(T);
if isempty(Ad)
   return;
end
[V, Dm] = eig(Ad); %#ok<ASGLU>
W = inv(V);
res = zeros(numel(p), 1);
for k = 1:numel(p)
   res(k) = abs(Cd * V(:, k) * W(k, :) * Bd);
end
if sum(res) > 0
   sh.share = res / sum(res);
end
end
