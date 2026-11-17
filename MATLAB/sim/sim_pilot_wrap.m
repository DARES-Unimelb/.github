function cmd = sim_pilot_wrap(P, stick, t)
% SIM_PILOT_WRAP Pilot command shaping, from control/pilot_to_command.m when present.
%
% The fallback keeps the model chain runnable before the control deliverable
% lands. It reproduces the authority limits of pilot_to_command but not its ramp
% limiting, and sim_deps_report marks the run as placeholder based.

if nargin < 1 || isempty(P)
   P = evalin('base', 'P');
end
if sim_have('pilot_to_command')
   cmd = pilot_to_command(P, stick, t);
   return;
end
cmd = struct();
cmd.phi_c = min(max(stick.roll, -1), 1) * deg2rad(P.limits.bank_max_deg);
cmd.theta_c = min(max(stick.pitch, -1), 1) * deg2rad(P.limits.pitch_max_deg);
phi_max = deg2rad(P.limits.bank_max_deg);
cmd.r_c = min(max(stick.yaw, -1), 1) * ...
   min(P.env.g * tan(phi_max) / P.op.cruise.V_mps, P.ctrl.bw.rate_r_rad_s);
thr = min(max(stick.throttle, 0), 1);
cmd.V_c = P.limits.V_min_mps + thr * (P.limits.V_max_mps - P.limits.V_min_mps);
cmd.h_c = P.op.cruise.h_m;
end
