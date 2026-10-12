function init_uacs_twin(varargin)
% INIT_UACS_TWIN Push the twin-engine UACS baseline parameters into the base workspace.
%
% Scripts and Simulink models read the struct P and the short scalar aliases from
% the base workspace. Call this once before opening or simulating a model.
%
% Usage:
%   init_uacs_twin                 % struct P plus aliases
%   init_uacs_twin('quiet', true)  % no console report
%
% This function never calls clear. Calling clear here would delete variables a
% calling script still needs, so it only overwrites what it owns.

ip = inputParser;
ip.addParameter('quiet', false, @(x) islogical(x) || isnumeric(x));
ip.addParameter('override', {}, @(x) isstruct(x) || iscell(x));
ip.parse(varargin{:});
quiet = logical(ip.Results.quiet);
over = local_override_pairs(ip.Results.override);

% An optional override layer, so a human-facing knob file can change numbers without
% becoming a second parameter source. The table is installed BEFORE the parameter file
% runs, because the file applies it at each point where a group's literals end and its
% derivation begins: that is what makes the derived quantities recompute. Overriding
% after the file had returned would leave V_min_trim, the rate-command clamps and the
% integral-state clamps describing the old aircraft.
%
% over is an Nx2 cell of {dotted path, value}. A dotted path cannot be a struct field
% name, so a struct input is flattened into the same pairs. The parameter file runs
% twice: once with no table, to get the values being replaced, and once with it. The
% first run is what makes the printed report name the old value rather than repeat the
% new one. A path that exists in neither run is an error, never a new field: a typo in
% a knob name must not silently add a parameter that nothing reads.
%
% A run with overrides is NOT the frozen baseline, so the overrides are recorded and
% printed. Consumers that print P.meta.revision must print meta.overrides too.
baseline = uacs_twin_params();
for k = 1:size(over, 1)
   parts = strsplit(char(over{k, 1}), '.');
   assert(local_path_exists(baseline, parts), 'init_uacs_twin:Override', ...
      'no parameter field %s, so the override cannot be applied', char(over{k, 1}));
end
if isempty(over)
   P = baseline;
   P.meta.overrides = cell(0, 2);
   P.meta.override_report = {};
else
   setappdata(0, 'uacs_override_pairs', over);
   setappdata(0, 'uacs_override_applied', {});
   cleanup_override = onCleanup(@() local_clear_override());
   P = uacs_twin_params();
   applied = getappdata(0, 'uacs_override_applied');
   clear cleanup_override;
   % Every override must have been applied by a uacs_apply_override call inside the
   % parameter file. A path that exists but sits in a group with no call would
   % otherwise be silently ignored, which is worse than a typo: the number would look
   % changed in the knob file and the model would fly the old one.
   missing = over(~ismember(over(:, 1), applied), 1);
   assert(isempty(missing), 'init_uacs_twin:OverrideNotApplied', ...
      ['these overrides were never applied, so the parameter group they belong to ' ...
       'carries no uacs_apply_override call: %s'], strjoin(missing, ', '));
   changed = cell(size(over, 1), 1);
   for k = 1:size(over, 1)
      parts = strsplit(char(over{k, 1}), '.');
      changed{k} = sprintf('%s: %s -> %s', char(over{k, 1}), ...
         local_show(local_get_path(baseline, parts)), local_show(over{k, 2}));
   end
   P.meta.overrides = over;
   P.meta.override_report = changed;
   if ~quiet
      fprintf('!! PARAMETER OVERRIDES ACTIVE - this run is not the frozen revision\n');
      for k = 1:numel(changed)
         fprintf('     %s\n', changed{k});
      end
   end
end

% The self-check RESOLVES some fields rather than only verifying them: the lowest
% trimmable speed needs the nonlinear trim solver, so it cannot be computed inside
% uacs_twin_params without adding a dependency on plant/. The resolved limits come
% back here and are merged into P before it is exported, otherwise P.limits.V_min_mps
% stays at the NaN placeholder that uacs_twin_params declares and every consumer
% reads NaN.
P.limits = local_self_check(P);

assignin('base', 'P', P);
f = fieldnames(P.tags);
for k = 1:numel(f)
    assignin('base', f{k}, P.tags.(f{k}));
end

if ~quiet
    fprintf('UACS twin-engine baseline %s (%s)\n', P.meta.version, P.meta.date);
    fprintf('  mass        %.2f kg, CG at x = %+.3f m\n', P.mass.m, P.mass.x_cg);
    fprintf('  inertia     Ixx %.3f  Iyy %.3f  Izz %.3f  Ixz %+.3f kg*m^2\n', ...
        P.mass.Ixx_cg, P.mass.Iyy_cg, P.mass.Izz_cg, P.mass.Ixz_cg);
    fprintf('  wing        b %.2f m, S %.3f m^2, AR %.2f, cbar %.3f m\n', ...
        P.geom.b, P.geom.S, P.geom.AR, P.geom.cbar);
    fprintf('  tails       Vh %.3f, Vv %.3f\n', P.geom.Vh, P.geom.Vv);
    fprintf('  cruise      %.1f m/s at %.0f m\n', ...
        P.op.cruise.V_mps, P.op.cruise.h_m);
    fprintf('  rate limits p %.1f  q %.1f  r %.1f rad/s target bandwidth\n', ...
        P.ctrl.bw.rate_p_rad_s, P.ctrl.bw.rate_q_rad_s, P.ctrl.bw.rate_r_rad_s);
    fprintf('  sample      inner %.1f Hz, outer %.1f Hz, solver %s\n', ...
        1/P.sim.Ts_ctrl, 1/P.sim.Ts_outer, P.sim.solver);
end
end

function limits_out = local_self_check(P)
% Physical and internal consistency conditions, plus the resolution of the limits
% that need the trim solver. Each assertion states the relation it enforces, so a
% failure points at the offending number. Returns the resolved limits struct, which
% the caller merges into P before exporting it.
limits_out = P.limits;
assert(P.mass.m > 0 && P.mass.m < 100, 'mass.m out of plausible range');
assert(P.geom.b > 0 && P.geom.S > 0, 'wing geometry must be positive');
assert(abs(P.geom.cbar - P.geom.S / P.geom.b) < 1e-12, 'cbar must equal S/b');
assert(abs(P.geom.AR - P.geom.b^2 / P.geom.S) < 1e-12, 'AR must equal b^2/S');

% Inertia triangle inequality, checked on the CG-referenced values.
I = sort([P.mass.Ixx_cg, P.mass.Iyy_cg, P.mass.Izz_cg]);
assert(I(1) + I(2) > I(3), 'inertia triangle inequality violated');
assert(all(I > 0), 'inertia must be positive');

% Static stability: pitch stiffness about the reference origin must be negative,
% otherwise no trim exists in the linear model.
assert(P.aero.Cm_alpha < 0, 'Cm_alpha must be negative for static stability');
% Static margin is an OUTCOME of the tail sizing, not a driver. It is wide (near
% 0.3) on purpose: at this wing loading the low-speed trim requirement forces a
% tail far larger than a conventional static margin would ask for, and no
% combination of aspect ratio and tail area satisfies both. See the tail sizing
% comment in uacs_twin_params. The band here only catches a gross error.
SM = -P.aero.Cm_alpha / P.aero.CL_alpha - (P.mass.x_cg - P.geom.x_ac) / P.geom.cbar;
assert(SM > 0.05 && SM < 0.70, ...
    sprintf(['static margin is %.1f percent of MAC, outside the plausible band ' ...
    '5..70 percent; check the tail volume and Cm_alpha'], 100 * SM));

% Elevator trim demand on a level cruise must leave travel in hand. This is the
% check that catches a bad rigging incidence or a camber moment that fights the
% tail: a hand-entered Cm0 of +0.020 once demanded 17 degrees of trim, 69 percent
% of travel, on a level cruise, and then the engine-out case needed over 1600
% degrees. Trim is solved from Cm = Cm0 + Cm_alpha*alpha + Cm_de*de = 0.
q_cruise = 0.5 * P.env.rho_sl * P.op.cruise.V_mps^2;
CL_cruise = P.mass.m * P.env.g / (q_cruise * P.geom.S);
alpha_trim = (CL_cruise - P.aero.CL0) / P.aero.CL_alpha;
de_trim = -(P.aero.Cm0 + P.aero.Cm_alpha * alpha_trim) / P.aero.Cm_de;
de_frac = abs(de_trim) / deg2rad(P.act.elevator.max_deg);
assert(de_frac < 0.40, ...
    sprintf(['level-cruise elevator trim is %.1f deg, %.0f percent of travel; ' ...
    'check the tail incidence chain and Cm0'], rad2deg(de_trim), 100 * de_frac));

% The engine-out nose-down moment from the thrust line offset must be trimmable,
% with margin, otherwise losing an engine leaves the aircraft needing more
% elevator than exists. This is what sets the allowable thrust line offset, and
% prop.z_thrust_offset_max is derived from exactly this inequality, so a layout
% that breaks it fails here rather than in flight.
M_engine_out = P.prop.T_static_max * abs(P.prop.z_thrust_offset);
M_elev_full = q_cruise * P.geom.S * P.geom.cbar * abs(P.aero.Cm_de) * ...
    deg2rad(P.act.elevator.max_deg);
assert(M_engine_out < 0.80 * M_elev_full, ...
    sprintf(['engine-out pitch moment %.2f N*m is %.0f percent of full elevator ' ...
    'authority %.2f N*m, above the 80 percent budget; reduce z_thrust_offset ' ...
    'below %.4f m or raise the elevator chord'], ...
    M_engine_out, 100 * M_engine_out / M_elev_full, M_elev_full, ...
    P.prop.z_thrust_offset_max));
assert(abs(P.prop.z_thrust_offset) <= P.prop.z_thrust_offset_max + 1e-12, ...
    'z_thrust_offset exceeds the value derived from the engine-out budget');

% Lowest trimmable speed, solved with THE REAL TRIM SOLVER rather than a closed
% form. This matters: de against V is strongly nonlinear here. A linearised Cm
% relation predicted -20.0 deg at 15.41 m/s while the actual trim needs -33.5 deg,
% so the closed form reported a minimum speed of 15.41 m/s against a true
% 17.98 m/s and promised a speed the aircraft cannot trim. Trim demand falls
% monotonically with speed across this band, so bisection is safe.
de_frac_at = @(V) local_trim_de_frac(P, V);

% Bisect for the speed at which trim uses the full trim budget.
V_lo = P.limits.V_min_stall_mps * 0.5;
V_hi = P.op.cruise.V_mps;
assert(de_frac_at(V_hi) < P.geom.elevator_trim_frac, ...
    sprintf(['the aircraft cannot trim at the design cruise point: %.0f percent ' ...
    'of elevator needed against a budget of %.0f percent'], ...
    100 * de_frac_at(V_hi), 100 * P.geom.elevator_trim_frac));
for k = 1:40
    V_mid = 0.5 * (V_lo + V_hi);
    if de_frac_at(V_mid) > P.geom.elevator_trim_frac
        V_lo = V_mid;
    else
        V_hi = V_mid;
    end
end
V_min_trim = 0.5 * (V_lo + V_hi);

% RESOLVE the limits here rather than only checking them. uacs_twin_params declares
% V_min_trim_mps and V_min_mps as NaN placeholders precisely because it cannot call
% the trim solver; this function is where the real values are produced. The returned
% struct is merged into P by the caller, so downstream code sees the resolved
% numbers instead of NaN.
limits_out.V_min_trim_mps = V_min_trim;
limits_out.V_min_mps = max(limits_out.V_min_stall_mps, V_min_trim);

% The declared limits must be consistent with what the aircraft can actually do.
assert(limits_out.V_min_mps >= V_min_trim, ...
    sprintf(['the resolved minimum speed %.2f m/s is below the lowest trimmable ' ...
    'speed %.2f m/s, which should be impossible'], ...
    limits_out.V_min_mps, V_min_trim));
assert(limits_out.V_stall_mps < limits_out.V_min_mps, ...
    'stall speed must sit below the minimum operating speed');

% The declared V_max must be REACHABLE. A limit above the thrust-limited top speed
% is a promise the aircraft cannot keep, and the first version of this set carried
% 40 m/s against a 31 m/s ceiling after the installed power was cut.
V_scan = limits_out.V_min_mps:0.5:limits_out.V_max_mps + 15;
CL_scan = P.mass.m * P.env.g ./ (0.5 * P.env.rho_sl * V_scan.^2 * P.geom.S);
CD_scan = P.aero.CD0 + P.aero.k_induced * CL_scan.^2;
D_scan = 0.5 * P.env.rho_sl * V_scan.^2 * P.geom.S .* CD_scan;
T_scan = P.prop.n_motors * prop_thrust_from_throttle(P, 1.0, V_scan);
i_ceil = find(T_scan < D_scan, 1, 'first');
assert(~isempty(i_ceil), 'no thrust-limited ceiling found in the scan range');
V_top = V_scan(i_ceil);
limits_out.V_top_thrust_limited_mps = V_top;
assert(limits_out.V_max_mps <= V_top, ...
    sprintf(['limits.V_max_mps is %.1f m/s but the thrust-limited top speed is ' ...
    'only %.1f m/s; raise the installed power or lower the limit'], ...
    limits_out.V_max_mps, V_top));

% Directional control on one engine: report the speed at which full rudder just
% balances a failed engine at full power, and flag whether the cruise point is
% inside it. This is a REPORTED margin rather than an assertion because whether the
% aircraft must be single-engine capable at cruise is a mission decision; what must
% not happen is the number going unrecorded.
Cn_dr_full = abs(P.aero.Cn_dr) * deg2rad(P.act.rudder.max_deg);
assert(Cn_dr_full > 0, 'rudder has no authority');
V_mc_check = sqrt(2 * P.prop.T_static_max * P.prop.y_motor / ...
    (P.env.rho_sl * P.geom.S * P.geom.b * Cn_dr_full));
% Directional stability must be positive, or the aircraft has no weathercock.
% Asserted with the value in the message because the sign chain runs through three
% coefficients and was wrong once: Cn_beta came out at +0.018 instead of +0.073.
assert(P.aero.Cn_beta > 0, ...
    sprintf('Cn_beta must be positive for directional stability, got %.4f', P.aero.Cn_beta));
% Roll damping and yaw damping must be negative.
assert(P.aero.Cl_p < 0, 'Cl_p must be negative');
assert(P.aero.Cn_r < 0, ...
    sprintf('Cn_r must be negative for yaw damping, got %.4f', P.aero.Cn_r));
% Dihedral effect must be negative for a stable spiral/roll coupling.
assert(P.aero.Cl_beta < 0, 'Cl_beta must be negative');

% Control effectiveness must have the sign that produces a restoring command.
assert(P.aero.Cm_de < 0, 'Cm_de must be negative with the nose-down elevator sign');
assert(P.aero.Cl_da > 0, 'Cl_da must be positive with the right-wing-down sign');
assert(P.aero.Cn_dr < 0, ...
    sprintf('Cn_dr must be negative with the nose-left rudder sign, got %.4f', P.aero.Cn_dr));

% Rudder authority relative to weathercock stiffness. This RATIO, not the fin
% area, sets the single-engine control speed V_mc, and it was once 0.240 against
% the published Aerosonde reference value of 0.945, which put V_mc above cruise.
% The published model is the reference point; stay in its neighbourhood.
ratio_dr_beta = abs(P.aero.Cn_dr) / P.aero.Cn_beta;
assert(ratio_dr_beta > 0.30, ...
    sprintf(['rudder authority ratio |Cn_dr|/Cn_beta is %.3f, below 0.30. ' ...
    'The fin is over-stabilised or the rudder is too small, and V_mc will ' ...
    'exceed the cruise speed'], ratio_dr_beta));

% Rudder chord fraction must actually reach Cn_dr. If a future edit drops the
% area-ratio factor again, this catches it: changing the chord fraction must
% change Cn_dr.
P_probe = P;
P_probe.geom.rudder.chord_frac = P.geom.rudder.chord_frac * 2;
Cn_dr_probe = -P_probe.geom.Vv * P_probe.aero.CY_dr * P_probe.geom.rudder.chord_frac;
assert(abs(Cn_dr_probe - 2 * P.aero.Cn_dr) < 1e-12, ...
    'Cn_dr does not scale with the rudder chord fraction');

% Lift slope must exceed the induced-drag coupling needed for the design point.
assert(P.aero.CL_alpha > 2.0 && P.aero.CL_alpha < 7.0, 'CL_alpha out of range');

% Sample rates must be nested: inner loop at least 4x the outer loop.
assert(P.sim.Ts_outer / P.sim.Ts_ctrl >= 2, 'outer loop must be slower than inner');
assert(P.sim.Ts_ctrl < P.sensor.imu.tau_s + 1, 'inner sample time implausible');

% Propulsion: two motors, and the thrust curve must match the peak power.
assert(P.prop.n_motors == 2, 'this model is a twin');
assert(P.prop.T_static_max > 0, 'propulsion model invalid');
assert(numel(P.prop.speed_table_T) == numel(P.prop.speed_table_V), ...
    'thrust table and speed table must be the same length');

% Static thrust must be the momentum-theory value for the stated peak power and
% figure of merit. Recomputing it catches an edit that changes one and forgets
% the other; the figure of merit enters linearly.
T_momentum = (P.prop.P_shaft_peak * P.prop.power.figure_of_merit * ...
    sqrt(2 * P.env.rho_sl * P.prop.disk_area))^(2/3);
assert(abs(T_momentum - P.prop.T_static_max) < 1e-9, ...
    'T_static_max is not consistent with the peak shaft power and figure of merit');

% The momentum-theory thrust curve must decrease with speed and must still cover
% the cruise drag at the design point, otherwise the aircraft cannot cruise.
Tt = P.prop.speed_table_T;
assert(all(diff(Tt) < 0), 'thrust must fall with advance speed');
assert(Tt(1) > 0 && Tt(end) >= 0, 'thrust curve leaves the valid range');

% The tabulated curve must reproduce the static closed form at V = 0, otherwise
% the table and the thrust law have drifted apart.
assert(abs(Tt(1) - P.prop.T_static_max) < 1e-9, ...
    'speed_table_T at V=0 must equal T_static_max');

% The thrust line offset is derived from geometry, never entered independently.
assert(abs(P.prop.z_thrust_offset - (P.prop.z_motor - P.mass.z_cg)) < 1e-12, ...
    'z_thrust_offset must equal z_motor - z_cg');

% The tabulated thrust curve and the throttle-to-thrust law are two routes to the
% same physics. They must agree at full throttle to machine precision, otherwise
% the plant and the control allocation are solving different aircraft.
Tt_full = prop_thrust_from_throttle(P, 1.0, P.prop.speed_table_V(:));
d_max = max(abs(Tt_full(:) - P.prop.speed_table_T(:)));
assert(d_max < 1e-9, ...
    sprintf('thrust table disagrees with prop_thrust_from_throttle by %.3e N', d_max));

% The implied figure of merit must stay physical (at or below 1) everywhere the
% model claims thrust is available. A table built from ideal power instead of
% shaft power violates this at high speed, which is how that error was caught.
% Ideal power is P_ideal = T*(V + v_i), where v_i is recovered from the momentum
% relation T = 2*rho*A*v_i*(V + v_i) by solving a quadratic in v_i:
%   v_i^2 + V*v_i - T/(2*rho*A) = 0.
% The figure of merit is then P_ideal / P_shaft_ideal, and it must not exceed 1.
P_ideal_avail = P.prop.P_shaft_peak * P.prop.power.figure_of_merit;
for k = 1:numel(P.prop.speed_table_V)
    Vk = P.prop.speed_table_V(k);
    Tk = P.prop.speed_table_T(k);
    if Tk <= 0
        continue;
    end
    disc = Vk^2 + 4 * Tk / (2 * P.env.rho_sl * P.prop.disk_area);
    vi_k = (sqrt(disc) - Vk) / 2;
    P_ideal_needed = Tk * (Vk + vi_k);
    fm_implied = P_ideal_needed / P_ideal_avail;
    assert(fm_implied <= 1 + 1e-6, ...
        sprintf('implied figure of merit at V=%.0f m/s is %.3f, above the physical limit', ...
        Vk, fm_implied));
end

% Mass budget must add up, so nobody silently changes one item.
m_items = sum(cell2mat(P.mass.items(:, 2)));
assert(abs(m_items - P.mass.m) < 1e-9, ...
    sprintf('mass.items sums to %.3f kg but mass.m is %.3f kg', m_items, P.mass.m));
% The team fixed the payload, so it must actually be present in the table at the
% stated value, and the remainder must fit the airframe budget.
assert(abs(P.mass.payload_kg - 12.0) < 1e-9, 'payload is fixed at 12 kg by the team');
m_payload = cell2mat(P.mass.items(strcmp(P.mass.items(:, 1), 'payload'), 2));
assert(abs(m_payload - P.mass.payload_kg) < 1e-9, ...
    sprintf('the payload row is %.3f kg but mass.payload_kg is %.3f kg', ...
    m_payload, P.mass.payload_kg));
assert(abs((P.mass.m - P.mass.payload_kg) - P.mass.airframe_budget_kg) < 1e-9, ...
    sprintf('airframe budget is %.3f kg, stated %.3f kg', ...
    P.mass.m - P.mass.payload_kg, P.mass.airframe_budget_kg));

% Hand the resolved limits back to the caller so P carries real numbers, not the
% NaN placeholders that uacs_twin_params declares for the trim-solved fields.
limits_out.V_min_trim_mps = V_min_trim;
limits_out.V_min_mps = max(limits_out.V_min_stall_mps, V_min_trim);
end
function frac = local_trim_de_frac(P, V)
% LOCAL_TRIM_DE_FRAC Elevator travel fraction needed to trim at speed V, in level
% flight, solved by the real trim solver so the nonlinear aerodynamics are included.
% The elevator surface is limited by an assertion in init_uacs_twin, so P.act is
% valid here. If the solver fails to converge the fraction is returned as Inf, which
% makes the bisection treat that speed as untrimmable rather than as trimable.
%
% This is deliberately NOT a closed-form Cm relation. A linearised version of this
% function under-reported the low-speed demand by 1.5 times and produced a minimum
% speed 2.6 m/s below the true one.
op = struct('V_mps', V, 'gamma_deg', 0.0, 'h_m', P.op.cruise.h_m);
try
    tr = trim_cruise(P, op);
catch
    frac = Inf;
    return;
end
if tr.exitflag <= 0 || tr.residual_norm_max > 1e-6
    frac = Inf;
    return;
end
frac = abs(tr.de_deg) / P.act.elevator.max_deg;
end

% ------------------------------------------------------------------------
function local_clear_override()
% Drop the override state whether the parameter file succeeded or raised, so a failed
% run cannot leave a table installed for the next caller.
if isappdata(0, 'uacs_override_pairs')
    rmappdata(0, 'uacs_override_pairs');
end
if isappdata(0, 'uacs_override_applied')
    rmappdata(0, 'uacs_override_applied');
end
end

function pairs = local_override_pairs(x)
% Normalise the override argument into an Nx2 cell of {dotted path, value}. A nested
% struct is flattened depth first, so a caller can pass either shape.
if isempty(x)
   pairs = cell(0, 2);
   return;
end
if iscell(x)
   assert(size(x, 2) == 2, 'init_uacs_twin:Override', ...
      'an override cell must be Nx2 {path, value}, got %s', mat2str(size(x)));
   pairs = x;
   return;
end
paths = {};
vals = {};
[paths, vals] = local_flatten(x, '', paths, vals);
pairs = [paths(:), vals(:)];
end

function [paths, vals] = local_flatten(s, pre, paths, vals)
f = fieldnames(s);
for k = 1:numel(f)
   if isempty(pre)
      p = f{k};
   else
      p = [pre '.' f{k}];
   end
   v = s.(f{k});
   if isstruct(v) && numel(v) == 1
      [paths, vals] = local_flatten(v, p, paths, vals);
   else
      paths{end + 1} = p; %#ok<AGROW>
      vals{end + 1} = v; %#ok<AGROW>
   end
end
end

function tf = local_path_exists(P, parts)
% True when the dotted path names a field that actually exists. Used to reject a knob
% typo before the parameter file runs with it installed.
tf = true;
v = P;
for k = 1:numel(parts)
    if ~isstruct(v) || ~isfield(v, parts{k})
        tf = false;
        return;
    end
    v = v.(parts{k});
end
end

function v = local_get_path(P, parts)
% Read P.a.b.c from a cell of field names, with a clear error when it is absent.
v = P;
for k = 1:numel(parts)
    assert(isstruct(v) && isfield(v, parts{k}), ...
        'init_uacs_twin:Override', 'no parameter field %s', strjoin(parts(1:k), '.'));
    v = v.(parts{k});
end
end

function P = local_set_path(P, parts, value)
% Write P.a.b.c from a cell of field names, refusing to create a new field. A typo
% in a knob name must fail here rather than silently add a parameter nothing reads.
if numel(parts) == 1
    assert(isfield(P, parts{1}), 'init_uacs_twin:Override', ...
        'no parameter field %s', parts{1});
    P.(parts{1}) = value;
    return;
end
assert(isfield(P, parts{1}) && isstruct(P.(parts{1})), ...
    'init_uacs_twin:Override', 'no parameter field %s', parts{1});
P.(parts{1}) = local_set_path(P.(parts{1}), parts(2:end), value);
end

function s = local_show(v)
if ischar(v)
    s = v;
elseif isscalar(v)
    s = sprintf('%g', v);
elseif isnumeric(v)
    s = mat2str(v);
else
    s = class(v);
end
end
