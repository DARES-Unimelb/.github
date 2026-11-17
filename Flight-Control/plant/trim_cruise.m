function trim = trim_cruise(P, op)
% TRIM_CRUISE Steady symmetric-flight trim of the UACS twin plant.
%
% op is one field of P.op (for example P.op.cruise); V_mps, gamma_deg,
% bank_deg and h_m are the fields used. Banked trim is not implemented and is
% rejected instead of silently returning an unbalanced point.
%
% Unknowns  z = [alpha; de; throttle], one throttle for both motors
% Equations Fx = 0, Fz = 0, My = 0 with beta = da = dr = 0, phi = 0 and
%           theta = alpha + gamma (so theta = alpha for level flight)
% Residual  r = [Fx Fy Fz Mx My Mz]' taken from eom_6dof, normalised as
%           forces / (m*g) and moments / (qbar*S*cbar); fsolve works on the
%           subset [Fx Fz My] / (m*g, m*g, qbar*S*cbar)
%
% Fields: x_star, u_star, residual, exitflag, alpha_deg, theta_deg, de_deg,
% throttle, plus the normalised residual and the operating point used

if nargin < 2 || isempty(op)
   op = P.op.cruise;
end
if ~isfield(op, 'bank_deg')
   op.bank_deg = 0;
end

V = op.V_mps;
gamma = deg2rad(op.gamma_deg);
bank = deg2rad(op.bank_deg);
h_m = op.h_m;
assert(abs(bank) < 1e-9, 'trim_cruise: banked trim is not implemented in v1');

m = P.mass.m;
g = P.env.g;
S = P.geom.S;
cbar = P.geom.cbar;
A = P.aero;
qbar0 = 0.5 * P.env.rho_sl * V^2;

% Analytic first guess for fsolve
CL_guess = m * g * cos(gamma) / (qbar0 * S);
alpha_guess = (CL_guess - A.CL0) / A.CL_alpha;
de_guess = -(A.Cm0 + A.Cm_alpha * alpha_guess) / A.Cm_de;
alpha_zl = -A.CL0 / A.CL_alpha;
CD_guess = A.CD0 + A.k_induced * CL_guess^2 ...
   + A.CD_alpha * (alpha_guess - alpha_zl)^2 + A.CD_de * abs(de_guess);
T_guess = qbar0 * S * CD_guess + m * g * sin(gamma);
% Throttle first guess by inverting the authoritative propulsion map
th_guess = local_throttle_for_thrust(P, T_guess / P.prop.n_motors, V);

ctx = struct('P', P, 'V', V, 'gamma', gamma, 'h_m', h_m, ...
   'norm_f', m * g, 'norm_m', qbar0 * S * cbar);

opts = optimoptions('fsolve', 'Display', 'off', ...
   'Algorithm', 'levenberg-marquardt', ...
   'FunctionTolerance', 1e-14, 'StepTolerance', 1e-14, ...
   'MaxIterations', 200, 'MaxFunctionEvaluations', 4000);

z0 = [alpha_guess; de_guess; th_guess];
[z, ~, exitflag, output] = fsolve(@(zz) local_res3(zz, ctx), z0, opts);
if exitflag <= 0
   % One restart from a larger angle of attack before reporting failure
   z0b = [deg2rad(5); de_guess; min(max(th_guess, 0.05), 0.9)];
   [z, ~, exitflag, output] = fsolve(@(zz) local_res3(zz, ctx), z0b, opts);
end

alpha = z(1);
de = z(2);
th = z(3);
theta = alpha + gamma;

x_star = [V * cos(alpha); 0; V * sin(alpha); ...
          0; 0; 0; ...
          0; theta; 0; ...
          0; 0; -h_m];
u_star = [0; de; 0; th; th; 0];

[~, F_body, M_body] = eom_6dof(0, x_star, u_star, P);
[~, ~, ~, qbar_t, rho_t] = flow_state(P, x_star);
res_raw = [F_body; M_body];
norm_f = m * g;
norm_m = qbar_t * S * cbar;
res_norm = [res_raw(1:3) / norm_f; res_raw(4:6) / norm_m];

trim = struct();
trim.x_star = x_star;
trim.u_star = u_star;
trim.residual = res_raw;
trim.residual_norm = res_norm;
trim.residual_norm_max = max(abs(res_norm));
trim.norm_force_N = norm_f;
trim.norm_moment_Nm = norm_m;
trim.exitflag = exitflag;
trim.iterations = output.iterations;
trim.func_count = output.funcCount;
if isfield(output, 'firstorderopt')
   trim.first_order_opt = output.firstorderopt;
else
   trim.first_order_opt = NaN;
end
trim.solver = 'fsolve/levenberg-marquardt';
trim.alpha_rad = alpha;
trim.alpha_deg = rad2deg(alpha);
trim.theta_rad = theta;
trim.theta_deg = rad2deg(theta);
trim.de_rad = de;
trim.de_deg = rad2deg(de);
trim.throttle = th;
trim.throttle_l = th;
trim.throttle_r = th;
trim.V_mps = V;
trim.gamma_deg = op.gamma_deg;
trim.bank_deg = op.bank_deg;
trim.h_m = h_m;
% The density the plant actually used at this point; the first guess above still
% uses the sea level density, which only steers fsolve and scales its residual
trim.rho = rho_t;
trim.qbar = qbar_t;

% Reference quantities of the trimmed point, for reports and cross-checks
dbg = aero_forces(P, x_star, u_star, V, alpha, 0, qbar_t);
T_per_motor = throttle_to_thrust(P, th, V);
trim.CL = dbg.CL;
trim.CD = dbg.CD;
trim.drag_N = dbg.D;
trim.L_over_D = dbg.L / dbg.D;
trim.thrust_per_motor_N = T_per_motor;
trim.thrust_total_N = P.prop.n_motors * T_per_motor;
trim.x_cg_m = P.mass.x_cg;
trim.gamma_rad = gamma;

% Two alpha windows are carried: the frozen criterion and the revised one
trim.window_alpha_frozen_deg = [2 10];
trim.window_alpha_revised_deg = [0 8];
trim.check_residual = trim.residual_norm_max < 1e-6;
trim.check_alpha = (trim.alpha_deg >= 2) && (trim.alpha_deg <= 10);
trim.check_alpha_revised = (trim.alpha_deg >= 0) && (trim.alpha_deg <= 8);
trim.check_de = abs(trim.de_deg) <= 15;
trim.check_all = trim.exitflag > 0 && trim.check_residual && trim.check_alpha && trim.check_de;
trim.check_all_revised = trim.exitflag > 0 && trim.check_residual ...
   && trim.check_alpha_revised && trim.check_de;

fprintf(['trim V %5.1f m/s gamma %+5.2f deg: alpha %7.4f deg, theta %7.4f deg, ' ...
   'de %+7.4f deg, throttle %.4f, |r|max %.3e, exitflag %d\n'], ...
   V, op.gamma_deg, trim.alpha_deg, trim.theta_deg, trim.de_deg, th, ...
   trim.residual_norm_max, exitflag);
end

%% Local helpers
function r = local_res3(zz, ctx)
% Normalised residuals [Fx Fz My] handed to fsolve
r6 = local_res6(zz, ctx);
r = r6([1 3 5]);
end

function r = local_res6(zz, ctx)
% Normalised 6x1 residual at an arbitrary [alpha; de; throttle] candidate
P = ctx.P;
alpha = zz(1);
de = zz(2);
th = zz(3);
theta = alpha + ctx.gamma;

x = [ctx.V * cos(alpha); 0; ctx.V * sin(alpha); ...
     0; 0; 0; ...
     0; theta; 0; ...
     0; 0; -ctx.h_m];
u = [0; de; 0; th; th; 0];

[~, F_body, M_body] = eom_6dof(0, x, u, P);
r = [F_body / ctx.norm_f; M_body / ctx.norm_m];
end

function th = local_throttle_for_thrust(P, T_target, V)
% Invert the authoritative propulsion map by bisection; thrust is monotone
% increasing in throttle, so the bracket is safe
if T_target <= 0
   th = 0;
   return;
end
lo = P.prop.esc.min_throttle;
hi = P.prop.esc.max_throttle;
if prop_thrust_from_throttle(P, hi, V) <= T_target
   th = hi;
   return;
end
for iter = 1:60
   mid = 0.5 * (lo + hi);
   if prop_thrust_from_throttle(P, mid, V) < T_target
      lo = mid;
   else
      hi = mid;
   end
end
th = 0.5 * (lo + hi);
end
