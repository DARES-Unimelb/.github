function wt = wind_trim(trim, wind_ned, track_deg)
% WIND_TRIM Steady wind trim: the still air trim re-oriented onto a ground track.
%
%   wt = wind_trim(trim, wind_ned)
%   wt = wind_trim(trim, wind_ned, track_deg)
%
%   trim       a plant/trim_cruise.m result, solved in still air
%   wind_ned   3x1 steady wind, NED, m/s
%   track_deg  commanded ground track, deg clockwise from north, default 0
%
%   wt.x_star           12x1 initial state for the plant: x(1:3) is the body axis
%                       GROUND velocity, x(9) is the heading
%   wt.u_star           the still air control vector, unchanged
%   wt.psi_rad/psi_deg  heading
%   wt.crab_deg         psi minus the commanded track
%   wt.ground_speed_mps horizontal ground speed
%   wt.vertical_rate_mps  pd rate, -V*sin(gamma) + W_down
%
% The result is an exact equilibrium, not an approximation. The still air trim has
% zero body rates, phi = 0, theta = alpha + gamma, and its NED air relative
% velocity is
%   v_a^n = V*[cos(gamma)*cos(psi); cos(gamma)*sin(psi); -sin(gamma)]
% which depends on the heading only, because the alpha of the trim cancels against
% theta = alpha + gamma. Aerodynamics depend on V, alpha, beta and the rates, none
% of which a heading change touches; gravity depends on phi and theta; thrust
% depends on the throttle and V. So rotating the heading leaves F = 0 and M = 0
% exactly and no trim re-solve is needed.
%
% The heading puts the ground velocity v_g^n = v_a^n + W along the commanded
% track t:
%   s   = (t.W_h) + sqrt((t.W_h)^2 - |W_h|^2 + (V*cos(gamma))^2)
%   psi = atan2(s*t_E - W_E, s*t_N - W_N)
% For a north track this reduces to the crab angle psi = -asin(W_E/(V*cos(gamma))).
% A solution needs V*cos(gamma) > |W_h|, that is an airspeed above the horizontal
% wind; below it the aircraft cannot hold the ground track and this function errors
% instead of returning a point that does not exist.
%
% Zero wind returns the input state untouched, bit for bit, rather than rebuilding
% it through the direction cosine matrix; the two routes agree to a few ulps.
%
% The body from NED matrix is built here rather than taken from sim/sim_dcm.m, so
% plant/ stays usable with only plant/ and params/ on the path. It is the same
% matrix, the transpose of the one plant/eom_6dof.m builds.

if nargin < 3 || isempty(track_deg)
   track_deg = 0;
end
wind_ned = wind_ned(:);
assert(numel(wind_ned) == 3, 'wind_trim:Size', 'wind must be a 3 element NED vector');
assert(numel(trim.x_star) == 12, 'wind_trim:Size', 'trim.x_star must have 12 elements');
assert(all(isfinite(wind_ned)), 'wind_trim:Range', 'wind must be finite');

wt = struct();
wt.trim = trim;
wt.wind_ned = wind_ned;
wt.track_deg = track_deg;
wt.air_speed_mps = trim.V_mps;
V = trim.V_mps;
gamma = deg2rad(trim.gamma_deg);
Vh = V * cos(gamma);

if all(wind_ned == 0)
   wt.x_star = trim.x_star;
   wt.u_star = trim.u_star;
   wt.psi_rad = trim.x_star(9);
   wt.psi_deg = rad2deg(wt.psi_rad);
   wt.crab_deg = local_wrap180(wt.psi_deg - track_deg);
   wt.ground_speed_mps = Vh;
   wt.vertical_rate_mps = -V * sin(gamma);
   wt.zero_wind_shortcut = true;
   return;
end

phi = trim.x_star(7);
theta = trim.theta_rad;
t = [cosd(track_deg); sind(track_deg)];
W_h = wind_ned(1:2);
disc = (t' * W_h)^2 - (W_h' * W_h) + Vh^2;
assert(disc > 0, 'wind_trim:NoSolution', ...
   sprintf(['air speed %.3f m/s across the track is not enough for a %.3f m/s ' ...
   'horizontal wind; no ground track solution exists'], Vh, norm(W_h)));
s = t' * W_h + sqrt(disc);
psi = atan2(s * t(2) - W_h(2), s * t(1) - W_h(1));

% Ground velocity in NED, then into body axes. The air relative velocity stays the
% trim's own, so this equals v_a^b + C_nb*W.
v_g_ned = [s * t(1); s * t(2); -V * sin(gamma) + wind_ned(3)];
ct = cos(theta); st = sin(theta);
cp = cos(phi);   sp = sin(phi);
cs = cos(psi);   ss = sin(psi);
R_nb = [ct * cs,                ct * ss,               -st; ...
        sp * st * cs - cp * ss, sp * st * ss + cp * cs, sp * ct; ...
        cp * st * cs + sp * ss, cp * st * ss - sp * cs, cp * ct];

wt.x_star = trim.x_star;
wt.x_star(1:3) = R_nb * v_g_ned;
wt.x_star(9) = psi;
wt.u_star = trim.u_star;
wt.psi_rad = psi;
wt.psi_deg = rad2deg(psi);
wt.crab_deg = local_wrap180(wt.psi_deg - track_deg);
wt.ground_speed_mps = s;
wt.vertical_rate_mps = v_g_ned(3);
wt.ground_velocity_ned = v_g_ned;
wt.zero_wind_shortcut = false;
end

function a = local_wrap180(a)
% Fold an angle into (-180, 180] so a crab angle is reported as the shortest turn.
a = mod(a + 180, 360) - 180;
end
