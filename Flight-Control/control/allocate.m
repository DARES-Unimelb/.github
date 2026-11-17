function [da, de, dr, dth_l, dth_r, info] = allocate(P, M_cmd, T_cmd)
% ALLOCATE Weighted pseudo-inverse control allocation for the UACS twin.
%
%   [da, de, dr, dth_l, dth_r] = allocate(P, M_cmd, T_cmd)
%   [da, de, dr, dth_l, dth_r, info] = allocate(P, M_cmd, T_cmd)
%
%   M_cmd  3x1 desired body-axis aerodynamic moment [L; M; N], N*m
%   T_cmd  scalar desired total thrust, N
%   outputs are the five actuator commands in the frozen order
%   (da, de, dr rad, dth_l, dth_r in 0..1); the sixth plant input dT is
%   realised as the common mode of the two throttles and is reported in
%   info.dT_cmd / info.dT_ach
%
% Priority, applied in this order
%   1. elevator    pitch has no second effector, so de is solved first and
%                  clamped on its own; its pitch residual is not re-allocated
%   2. aileron and rudder   primary roll and yaw effectors
%   3. differential thrust  supplement, weighted by its moment authority
%                  relative to the rudder so it is used only as a last resort
%
%   The allocator is static and linear about the design point: the effectiveness
%   matrix uses the P.aero control derivatives at the cruise dynamic pressure,
%   and the thrust inversion uses P.op.cruise.V_mps
%   info carries the request, the achievement, the saturation flags and the
%   limits actually used, so a caller can log request against achievement

M_cmd = M_cmd(:);
assert(numel(M_cmd) == 3, 'allocate: M_cmd must be 3x1');
assert(isscalar(T_cmd), 'allocate: T_cmd must be scalar');

%% Limits, weights and effectiveness, all from P
da_lim = deg2rad(P.act.aileron.max_deg);
de_lim = deg2rad(P.act.elevator.max_deg);
dr_lim = deg2rad(P.act.rudder.max_deg);
tc_min = P.prop.esc.min_throttle;
tc_max = P.prop.esc.max_throttle;
diff_lim = P.prop.differential_limit;

V_a = P.op.cruise.V_mps;          % allocation linearisation speed
qbar_a = 0.5 * P.env.rho_sl * V_a^2;
S = P.geom.S;
Sb = qbar_a * S * P.geom.b;
Sc = qbar_a * S * P.geom.cbar;
y_arm = P.prop.y_motor - P.mass.y_cg;   % station the thrust differential acts on
z_arm = P.prop.z_thrust_offset;         % thrust line offset used by plant/thrust_moment

% G maps v = [da; de; dr; dTdiff] to the body moment, dTdiff = T_r - T_l as in
% plant/thrust_moment.m. Row 1 roll, row 2 pitch, row 3 yaw.
G = [Sb * P.aero.Cl_da, 0,                   Sb * P.aero.Cl_dr, 0; ...
     0,                 Sc * P.aero.Cm_de,  0,                 0; ...
     Sb * P.aero.Cn_da, 0,                   Sb * P.aero.Cn_dr, y_arm];

%% Common throttle from the total thrust command
% The propulsion map is params/prop_thrust_from_throttle.m, the single
% authoritative one; this file carries no second thrust law. Throttle is a
% fraction of peak shaft power, so inverting it is a monotone solve. The result
% is cached on (speed, thrust), so a repeated request runs no solve at all.
T_static = P.prop.T_static_max;
T_want = 0.5 * T_cmd;
[cache, hit] = local_throttle_cache(P, V_a, T_want);
tc = cache.tc;
d_cap = diff_lim;
d_headroom = cache.d_headroom;
d_eff = cache.d_eff;
dT_lim = cache.dT_lim;
tc_raw = cache.tc_unclamped;
info_cache_hit = hit;

%% 1. Pitch: elevator alone
de_raw = M_cmd(2) / G(2, 2);
de = min(max(de_raw, -de_lim), de_lim);

%% 2 and 3. Roll/yaw, aileron and rudder first
M_ry = M_cmd([1 3]) - G([1 3], 2) * de;
v_lim = [da_lim; dr_lim; dT_lim];
% priority weight: differential thrust costs as much as the rudder is stronger
n_rud = Sb * abs(P.aero.Cn_dr) * dr_lim;
n_dif = abs(y_arm * dT_lim);
p_supp = min(max(n_rud / max(n_dif, eps), 1.0), 1e6);
p_pri = [1.0; 1.0; p_supp];

idx = [1 3 4];                 % da, dr, dTdiff inside G
G_ry = G([1 3], idx);
free = v_lim > 0;
v = zeros(3, 1);
for iter = 1:3
   if ~any(free)
      break;
   end
   b = M_ry - G_ry(:, ~free) * v(~free);
   An = G_ry(:, free) * diag(v_lim(free));      % normalised effectiveness
   Wi = diag(1 ./ p_pri(free).^2);
   e = Wi * An' * pinv(An * Wi * An') * b;      % weighted minimum norm
   v_new = zeros(3, 1);
   v_new(free) = v_lim(free) .* e;
   hit = abs(v_new) > v_lim * (1 + 1e-12);
   hit = hit & free;
   v = v_new;
   if ~any(hit)
      break;
   end
   v(hit) = sign(v(hit)) .* v_lim(hit);
   free = free & ~hit;
end
da = min(max(v(1), -da_lim), da_lim);
dr = min(max(v(2), -dr_lim), dr_lim);
dT = min(max(v(3), -dT_lim), dT_lim);

%% Throttles, then the hard clamp
dth_half = local_dth_for_dT(P, V_a, tc, dT, d_eff);
dth_l = tc - dth_half;
dth_r = tc + dth_half;
dth_l = min(max(dth_l, tc_min), tc_max);
dth_r = min(max(dth_r, tc_min), tc_max);

%% Achievement
% dT_ach is what the two throttles really produce through the authoritative map,
% so a caller can see the difference between request and achievement; the
% residual is the nonlinearity of the map, not an error of the linear solve
T_l = local_thrust(P, V_a, dth_l);
T_r = local_thrust(P, V_a, dth_r);
dT_ach = T_r - T_l;
T_ach = T_l + T_r;
M_ach = G(:, 1:3) * [da; de; dr] + [0; 0; y_arm * dT_ach];

info = struct();
info.M_cmd = M_cmd;
info.T_cmd = T_cmd;
info.M_ach = M_ach;
info.T_ach = T_ach;
info.M_err = M_ach - M_cmd;
info.M_err_norm = norm(M_ach - M_cmd);
info.M_err_rel = norm(M_ach - M_cmd) / max(norm(M_cmd), eps);
info.T_err = T_ach - T_cmd;
info.err_pitch_Nm = M_ach(2) - M_cmd(2);
info.v_cmd = [v(1); de_raw; v(2); v(3)];
info.v_ach = [da; de; dr; dT_ach];
info.limits = struct('da', da_lim, 'de', de_lim, 'dr', dr_lim, ...
   'dT', dT_lim, 'tc_min', tc_min, 'tc_max', tc_max, 'd_cap', d_cap, ...
   'd_headroom', d_headroom, 'd_eff', d_eff);
info.sat = struct('da', abs(v(1)) >= da_lim * (1 + 1e-12), ...
   'de', abs(de_raw) >= de_lim * (1 + 1e-12), ...
   'dr', abs(v(2)) >= dr_lim * (1 + 1e-12), ...
   'dT', abs(v(3)) >= dT_lim * (1 + 1e-12), ...
   'throttle', (tc_raw ~= tc) || (abs(dth_half) > d_eff * (1 + 1e-12)));
info.sat_any = info.sat.da || info.sat.de || info.sat.dr || info.sat.dT || info.sat.throttle;
info.throttle_cache_hit = info_cache_hit;
info.tc = tc;
info.tc_raw = tc_raw;
info.dT_cmd = dT;
info.dT_ach = dT_ach;
info.dT_lim = dT_lim;
info.priority = p_pri;
info.authority = struct('yaw_rudder_Nm', n_rud, 'yaw_diff_Nm', n_dif, ...
   'ratio_rudder_over_diff', p_supp);
info.G = G;
info.qbar = qbar_a;
info.V_alloc = V_a;
info.T_static_max = T_static;
info.M_thrust_pitch_Nm = z_arm * T_cmd;   % not part of M_cmd, reported for the mixer
info.rate_limit_deg_s = [P.act.aileron.rate_deg_s; P.act.elevator.rate_deg_s; ...
   P.act.rudder.rate_deg_s];
info.throttle_rate_per_s = P.prop.esc.rate_per_s;
end

%% Local helpers
function T = local_thrust(P, V, dth)
% The authoritative per-motor thrust map, called not re-derived
T = prop_thrust_from_throttle(P, dth, V);
end

function [c, hit] = local_throttle_cache(P, V, T_want)
% Throttle, differential authority and headroom for one (speed, thrust) request;
% repeated identical requests are served from the previous answer
persistent last
if isempty(last)
   last = struct('V', NaN, 'T', NaN, 'tc', NaN, 'tc_unclamped', NaN, ...
      'd_eff', NaN, 'dT_lim', NaN, 'd_headroom', NaN);
end
tol = 1e-12 * max(1, abs(T_want));
if isfinite(last.V) && abs(V - last.V) <= 1e-12 && abs(T_want - last.T) <= tol
   c = last;
   hit = true;
   return;
end

tc_min = P.prop.esc.min_throttle;
tc_max = P.prop.esc.max_throttle;
T_at_min = local_thrust(P, V, tc_min);
T_at_max = local_thrust(P, V, tc_max);
th = local_throttle_for_thrust(P, T_want, V, T_at_min, T_at_max);
tc = min(max(th, tc_min), tc_max);
% Differential authority: P.prop.differential_limit is an absolute throttle split
% and the usable split is whatever throttle headroom the operating point has,
% because the total thrust must not change and neither motor may leave the ESC
% window
d_cap = P.prop.differential_limit;
d_headroom = min(tc - tc_min, tc_max - tc);
d_eff = min(d_cap, d_headroom);
dT_lim = local_thrust(P, V, tc + d_eff) - local_thrust(P, V, tc - d_eff);

last = struct('V', V, 'T', T_want, 'tc', tc, 'tc_unclamped', th, ...
   'd_eff', d_eff, 'dT_lim', dT_lim, 'd_headroom', d_headroom);
c = last;
hit = false;
end

function th = local_throttle_for_thrust(P, T_want, V, T_at_min, T_at_max)
% Throttle that makes one motor produce T_want at speed V; the map is monotone
% increasing in throttle, so bisection is enough and the bracket is the ESC window
lo = P.prop.esc.min_throttle;
hi = P.prop.esc.max_throttle;
if T_want <= T_at_min
   th = lo;
   return;
end
if T_want >= T_at_max
   th = hi;
   return;
end
for k = 1:40
   mid = 0.5 * (lo + hi);
   if local_thrust(P, V, mid) < T_want
      lo = mid;
   else
      hi = mid;
   end
end
th = 0.5 * (lo + hi);
end

function dh = local_dth_for_dT(P, V, tc, dT, d_eff)
% Half the throttle split that produces the wanted thrust difference T_r - T_l;
% T(tc + d) - T(tc - d) is monotone in d on [0, d_eff], so guarded false position
% converges without a derivative, which the map does not have at idle
if abs(dT) < 1e-12 || d_eff <= 0
   dh = 0;
   return;
end
dh_max = d_eff;
T_lo = local_thrust(P, V, tc - dh_max);
T_hi = local_thrust(P, V, tc + dh_max);
if abs(dT) >= T_hi - T_lo
   dh = dh_max;
   return;
end
lo = 0;
hi = dh_max;
f_lo = -dT;
f_hi = (T_hi - T_lo) - dT;
for k = 1:40
   dh = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
   if ~(dh > lo && dh < hi)
      dh = 0.5 * (lo + hi);           % guard against a stalled secant
   end
   f = (local_thrust(P, V, tc + dh) - local_thrust(P, V, tc - dh)) - dT;
   if abs(f) < 1e-11
      return;
   end
   if f < 0
      lo = dh; f_lo = f;
   else
      hi = dh; f_hi = f;
   end
end
dh = 0.5 * (lo + hi);
end
