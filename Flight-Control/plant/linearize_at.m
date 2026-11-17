function [lin, info] = linearize_at(P, trim, opts)
% LINEARIZE_AT Central-difference linearisation of eom_6dof at a trim point.
%
%   [lin, info] = linearize_at(P, trim)
%   [lin, info] = linearize_at(P, trim, struct('step', 1e-5, 'verbose', false))
%
%   lin.A, lin.B   12x12 and 12x6 numerical Jacobian about trim.x_star/u_star
%   lin.C, lin.D   eye(12) and zeros(12,6): the plant model outputs all 12
%                  states and carries no feedthrough
%   lin.A_long     A([1 3 5 8], [1 3 5 8])  states u w q theta  ("dV" = u)
%   lin.B_long     B([1 3 5 8], [2 6])      inputs de dT
%   lin.A_lat      A([2 4 6 7], [2 4 6 7])  states v p r phi
%   lin.B_lat      B([2 4 6 7], [1 3])      inputs da dr
%   lin.modes_long, lin.modes_lat  named modes with freq_rad_s and damping
%                  freq_rad_s follows the Control System Toolbox damp()
%                  convention and is the pole magnitude |lambda|
%
%   Step rule: h_i = step * max(1, |x_i|) for states and step * max(1, |u_i|)
%   for inputs; the submatrices are extracted from lin.A and lin.B, never
%   re-linearised

if nargin < 3
   opts = struct();
end
if ~isfield(opts, 'step')
   opts.step = 1e-6;
end
if ~isfield(opts, 'verbose')
   opts.verbose = true;
end

x0 = trim.x_star(:);
u0 = trim.u_star(:);
n = numel(x0);
m_in = numel(u0);
step = opts.step;

hx = step * max(1, abs(x0));
hu = step * max(1, abs(u0));

A = zeros(n, n);
B = zeros(n, m_in);
n_eval = 0;
t0 = tic;
for i = 1:n
   xp = x0; xp(i) = xp(i) + hx(i);
   xm = x0; xm(i) = xm(i) - hx(i);
   A(:, i) = (eom_6dof(0, xp, u0, P) - eom_6dof(0, xm, u0, P)) / (2 * hx(i));
   n_eval = n_eval + 2;
end
for j = 1:m_in
   up = u0; up(j) = up(j) + hu(j);
   um = u0; um(j) = um(j) - hu(j);
   B(:, j) = (eom_6dof(0, x0, up, P) - eom_6dof(0, x0, um, P)) / (2 * hu(j));
   n_eval = n_eval + 2;
end

idx_long_x = [1 3 5 8];
idx_long_u = [2 6];
idx_lat_x = [2 4 6 7];
idx_lat_u = [1 3];

lin = struct();
lin.A = A;
lin.B = B;
lin.C = eye(n);
lin.D = zeros(n, m_in);
lin.x_star = x0;
lin.u_star = u0;
lin.state_names = {'u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'pn', 'pe', 'pd'};
lin.input_names = {'da', 'de', 'dr', 'dth_l', 'dth_r', 'dT'};
lin.idx_long_x = idx_long_x;
lin.idx_long_u = idx_long_u;
lin.idx_lat_x = idx_lat_x;
lin.idx_lat_u = idx_lat_u;
% Submatrices by direct extraction from A and B
lin.A_long = A(idx_long_x, idx_long_x);
lin.B_long = B(idx_long_x, idx_long_u);
lin.A_lat = A(idx_lat_x, idx_lat_x);
lin.B_lat = B(idx_lat_x, idx_lat_u);
lin.long_state_names = lin.state_names(idx_long_x);
lin.long_input_names = lin.input_names(idx_long_u);
lin.lat_state_names = lin.state_names(idx_lat_x);
lin.lat_input_names = lin.input_names(idx_lat_u);
lin.modes_long = local_modes(eig(lin.A_long), 'long', x0, P);
lin.modes_lat = local_modes(eig(lin.A_lat), 'lat', x0, P);

info = struct();
info.step = step;
info.hx = hx;
info.hu = hu;
info.n_eval = n_eval;
info.elapsed_s = toc(t0);
info.idx_long_x = idx_long_x;
info.idx_long_u = idx_long_u;
info.idx_lat_x = idx_lat_x;
info.idx_lat_u = idx_lat_u;
info.eig_A = eig(A);
info.verbose = opts.verbose;

if opts.verbose
   fprintf('linearize_at: step %.1e, %d eom evaluations, %.3f s\n', ...
      step, n_eval, info.elapsed_s);
   fprintf('  A_long = A(%s, %s), B_long = B(%s, %s)  states [%s]  inputs [%s]\n', ...
      mat2str(idx_long_x), mat2str(idx_long_x), mat2str(idx_long_x), ...
      mat2str(idx_long_u), strjoin(lin.long_state_names, ' '), ...
      strjoin(lin.long_input_names, ' '));
   fprintf('  A_lat  = A(%s, %s), B_lat  = B(%s, %s)  states [%s]  inputs [%s]\n', ...
      mat2str(idx_lat_x), mat2str(idx_lat_x), mat2str(idx_lat_x), ...
      mat2str(idx_lat_u), strjoin(lin.lat_state_names, ' '), ...
      strjoin(lin.lat_input_names, ' '));
   fprintf('  %-16s %14s %14s %12s %12s %10s\n', ...
      'mode', 'real', 'imag', 'wn rad/s', 'wd rad/s', 'zeta');
   local_print_modes(lin.modes_long, 'longitudinal');
   local_print_modes(lin.modes_lat, 'lateral');
end
end

%% Local helpers
function local_print_modes(S, banner)
fprintf('  [%s]\n', banner);
for k = 1:numel(S)
   fprintf('  %-16s %14.6f %14.6f %12.6f %12.6f %10.4f\n', ...
      S(k).name, S(k).real_part, S(k).imag_part, S(k).freq_nat_rad_s, ...
      S(k).freq_damped_rad_s, S(k).damping);
end
end

function S = local_modes(lam, tag, x0, P)
% Named modes with frequency, damping and time constant from eigenvalues
lam = lam(:);
n = numel(lam);
used = false(n, 1);
S = struct('name', {}, 'lambda', {}, 'real_part', {}, 'imag_part', {}, ...
   'freq_rad_s', {}, 'freq_nat_rad_s', {}, 'freq_damped_rad_s', {}, ...
   'freq_hz', {}, 'damping', {}, 'tau_s', {}, ...
   'is_oscillatory', {}, 'stable', {});
tol = 1e-9;
k = 0;
for i = 1:n
   if used(i)
      continue;
   end
   if abs(imag(lam(i))) > tol
      d = abs(lam - conj(lam(i)));
      d(used) = Inf;
      d(i) = Inf;
      j = find(d < tol * max(1, abs(lam(i))), 1);
      used(i) = true;
      if ~isempty(j)
         used(j) = true;
      end
      l = lam(i);
      k = k + 1;
      S(k) = local_entry(l, true);
   else
      used(i) = true;
      k = k + 1;
      S(k) = local_entry(real(lam(i)), false);
   end
end
if isempty(S)
   return;
end

switch tag
   case 'long'
      % Name longitudinal modes against the analytic estimates below; matching on
      % "highest oscillatory frequency" mislabels an overdamped short period
      %   short period  sqrt(qbar*S*cbar*|Cm_alpha|/Iyy)
      %   phugoid       sqrt(2)*g/V
      V0 = max(abs(x0(1)), 1e-6);
      qbar0 = 0.5 * P.env.rho_sl * V0^2;
      wn_sp = sqrt(qbar0 * P.geom.S * P.geom.cbar * abs(P.aero.Cm_alpha) / P.mass.Iyy_cg);
      wn_ph = sqrt(2) * P.env.g / V0;
      [~, which_sp] = min(abs([S.freq_nat_rad_s] - wn_sp));
      [~, which_ph] = min(abs([S.freq_nat_rad_s] - wn_ph));
      S(which_sp).name = 'short_period';
      if which_ph ~= which_sp
         S(which_ph).name = 'phugoid';
      end
      % Any remaining mode gets a name stating its character
      for t = 1:numel(S)
         if isempty(S(t).name)
            if S(t).is_oscillatory
               S(t).name = sprintf('long_osc_%.3f', S(t).freq_nat_rad_s);
            else
               S(t).name = sprintf('long_real_tau_%.4f', S(t).tau_s);
            end
         end
      end
   case 'lat'
      osc = find([S.is_oscillatory]);
      for t = 1:numel(osc)
         S(osc(t)).name = 'dutch_roll';
      end
      re = find(~[S.is_oscillatory]);
      [~, ord] = sort(abs([S(re).real_part]), 'descend');
      re = re(ord);
      if numel(re) == 1
         if abs(S(re(1)).real_part) > 1
            S(re(1)).name = 'roll_subsidence';
         else
            S(re(1)).name = 'spiral';
         end
      else
         for t = 1:numel(re)
            if t == 1
               S(re(t)).name = 'roll_subsidence';
            elseif t == numel(re)
               S(re(t)).name = 'spiral';
            else
               S(re(t)).name = sprintf('lat_real_%d', t);
            end
         end
      end
   otherwise
      for t = 1:numel(S)
         S(t).name = sprintf('%s_%d', tag, t);
      end
end
end

function E = local_entry(l, osc)
% One mode entry from a complex or real eigenvalue; freq_rad_s is the pole
% magnitude, matching the Control System Toolbox damp() convention
E = struct('name', '', 'lambda', l, 'real_part', real(l), 'imag_part', imag(l), ...
   'freq_rad_s', abs(l), 'freq_nat_rad_s', abs(l), 'freq_damped_rad_s', abs(imag(l)), ...
   'freq_hz', abs(l) / (2 * pi), 'damping', 1, 'tau_s', Inf, ...
   'is_oscillatory', osc, 'stable', real(l) < 0);
if osc
   E.damping = -real(l) / abs(l);
else
   E.imag_part = 0;
   E.freq_damped_rad_s = 0;
end
if real(l) < 0
   E.tau_s = -1 / real(l);
else
   E.tau_s = Inf;
end
end
