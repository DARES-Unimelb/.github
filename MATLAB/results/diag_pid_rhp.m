% diag_pid_rhp Inspect the right half plane structure of the phi and theta loops.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw;
Tf = P.ctrl.rate_filter_tau_s;
lag.da = tf(1, [P.act.aileron.tau_s, 1]);
lag.de = tf(1, [P.act.elevator.tau_s, 1]);
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag.de;
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
ctf_p = minreal(tf([Kp_p * Tf, Kp_p], [Tf, 1]));
ctf_q = minreal(tf([Kp_q * Tf, Kp_q], [Tf, 1]));
Gphi = feedback(ctf_p * Gp, 1) * tf(1, [1 0]);
Gtheta = feedback(ctf_q * Gq, 1) * tf(1, [1 0]);

fprintf('plant RHP zeros: Gp %s | Gq %s\n', mat2str(zero(Gp)), mat2str(zero(Gq)));

cases = {'phi kp1 f0.5', Gphi, 1, 0.5, wc.att_phi_rad_s; ...
         'phi kp1 f0', Gphi, 1, 0, wc.att_phi_rad_s; ...
         'phi kp0.2 f0.5', Gphi, 0.2, 0.5, wc.att_phi_rad_s; ...
         'theta kp1 f0.5', Gtheta, 1, 0.5, wc.att_theta_rad_s; ...
         'theta kp0.2 f0.5', Gtheta, 0.2, 0.5, wc.att_theta_rad_s};
for k = 1:size(cases, 1)
   nm = cases{k, 1}; G = cases{k, 2}; kp = cases{k, 3}; f = cases{k, 4};
   w = cases{k, 5};
   ctf = minreal(tf([kp, kp * f * w], [1, 0]));
   for s = [1, -1]
      L = minreal(s * ctf * G);
      p = pole(feedback(L, 1));
      z = zero(L);
      ps = sort(p, 'ComparisonMethod', 'real');
      fprintf('\n--- %s sgn %+d ---\n', nm, s);
      fprintf('  cl poles (rightmost 4): %s\n', mat2str(ps(max(1, end - 3):end).', 8));
      fprintf('  ol zeros (rightmost 4): %s\n', mat2str(sort(z, 'ComparisonMethod', 'real').', 8));
      pr = sort(real(p), 'descend');
      pr = pr(1:min(4, numel(pr)));
      fprintf('  top real parts: %s\n', mat2str(pr.', 8));
      % cancellation error of the rightmost positive pair
      pos = ps(real(ps) > -1e-12);
      if ~isempty(pos)
         for j = 1:numel(pos)
            [dmin, jz] = min(abs(z - pos(j)));
            fprintf('  pos pole %+.8e  nearest ol zero %+.8e  |d| %.3e\n', ...
               real(pos(j)), real(z(jz)), dmin);
         end
      end
   end
end
