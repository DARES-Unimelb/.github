function cmd = sim_pilot_wrap(P, stick, t)
% SIM_PILOT_WRAP Pilot command shaping, from control/pilot_to_command.m when present
%
%   P       parameter struct, default P from the base workspace
%   stick   struct with fields roll, pitch, yaw, throttle, each -1 to 1
%   t       time in s
%   cmd     struct with fields phi_c, theta_c, r_c, V_c, h_c
%
% The local branch applies the same authority limits as pilot_to_command but no
% ramp limiting, and sim_deps_report marks the run as placeholder based

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
