function [cmd, info] = pilot_to_command(P, stick, t)
% PILOT_TO_COMMAND Shape RC stick input into reference commands.
%
%   cmd = pilot_to_command(P, stick, t)
%   [cmd, info] = pilot_to_command(P, stick, t)
%
%   stick.roll      -1..1   right positive
%   stick.pitch     -1..1   back (nose up) positive
%   stick.yaw       -1..1   right positive
%   stick.throttle   0..1
%   t               s, simulation time, non-decreasing within one run
%
%   cmd.phi_c  rad      bank angle command
%   cmd.theta_c rad     pitch attitude command, attitude mode
%   cmd.r_c    rad/s    yaw rate command
%   cmd.V_c    m/s      airspeed command
%   cmd.h_c    m        altitude command, altitude mode
%
% Authority limits come from P.limits, ramp limits from the bandwidth targets in
% P.ctrl.bw: a command may not slew faster than the loop that has to follow it.
% Both are applied here and nowhere else, so the control laws stay free of stick
% shaping.
%
% The pitch stick drives both theta_c and h_c, and a mode switch selects which one
% the controller follows: attitude mode uses theta_c directly, altitude mode feeds
% h_c to the height loop, which owns the pitch channel.
%
% The rate limiter and the altitude integrator are persistent state, reset
% whenever t does not advance, so every run starts from the same condition.

roll = local_sat(stick.roll);
pitch = local_sat(stick.pitch);
yaw = local_sat(stick.yaw);
thr = min(max(stick.throttle, 0), 1);

phi_max = deg2rad(P.limits.bank_max_deg);
theta_max = deg2rad(P.limits.pitch_max_deg);
V_min = P.limits.V_min_mps;
V_max = P.limits.V_max_mps;
h_max = P.limits.h_max_m;
w_climb = P.limits.climb_rate_max_mps;
% Yaw rate authority: coordinated turn rate at the bank limit and cruise speed
r_max = min(P.env.g * tan(phi_max) / P.op.cruise.V_mps, P.ctrl.bw.rate_r_rad_s);

ramp_phi = P.ctrl.bw.att_phi_rad_s * phi_max;
ramp_theta = P.ctrl.bw.att_theta_rad_s * theta_max;
ramp_r = P.ctrl.bw.rate_r_rad_s * r_max;
ramp_V = P.ctrl.bw.speed_rad_s * (V_max - V_min);

raw = struct('phi_c', roll * phi_max, 'theta_c', pitch * theta_max, ...
   'r_c', yaw * r_max, 'V_c', V_min + thr * (V_max - V_min), ...
   'h_c', P.op.cruise.h_m + pitch * w_climb);

persistent prev
if isempty(prev) || ~isscalar(t) || t <= prev.t
   prev = struct('t', t, 'phi_c', 0, 'theta_c', 0, 'r_c', 0, ...
      'V_c', P.op.cruise.V_mps, 'h_c', P.op.cruise.h_m);
end
dt = max(t - prev.t, 0);

cmd = struct();
cmd.phi_c = local_slew(prev.phi_c, raw.phi_c, ramp_phi * dt);
cmd.theta_c = local_slew(prev.theta_c, raw.theta_c, ramp_theta * dt);
cmd.r_c = local_slew(prev.r_c, raw.r_c, ramp_r * dt);
cmd.V_c = local_slew(prev.V_c, raw.V_c, ramp_V * dt);
% Altitude command integrates the same pitch stick as a climb rate
h_target = min(max(prev.h_c + pitch * w_climb * dt, 0), h_max);
cmd.h_c = local_slew(prev.h_c, h_target, w_climb * dt);

prev.t = t;
prev.phi_c = cmd.phi_c;
prev.theta_c = cmd.theta_c;
prev.r_c = cmd.r_c;
prev.V_c = cmd.V_c;
prev.h_c = cmd.h_c;

info = struct();
info.raw = raw;
info.ramp = struct('phi', ramp_phi, 'theta', ramp_theta, 'r', ramp_r, 'V', ramp_V, 'h', w_climb);
info.limits = struct('phi_max', phi_max, 'theta_max', theta_max, 'r_max', r_max, ...
   'V_min', V_min, 'V_max', V_max, 'h_max', h_max, 'climb_rate_max', w_climb);
info.dt = dt;
info.stick = struct('roll', roll, 'pitch', pitch, 'yaw', yaw, 'throttle', thr);
end

%% Local helpers
function y = local_sat(x)
y = min(max(x, -1), 1);
end

function y = local_slew(y0, target, max_step)
y = y0 + min(max(target - y0, -max_step), max_step);
end
