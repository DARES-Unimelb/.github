% _DIAG_PID_LOOPS Print the closed-loop pole structure of every loop design_pid
% must close.
init_uacs_twin('quiet', true);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
wc = P.ctrl.bw;
Tf = P.ctrl.rate_filter_tau_s;

lag.da = tf(1, [P.act.aileron.tau_s, 1]);
lag.de = tf(1, [P.act.elevator.tau_s, 1]);
lag.dr = tf(1, [P.act.rudder.tau_s, 1]);
lag.T = tf(1, [P.prop.esc.tau_s, 1]);

Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * lag.da;
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * lag.de;
Gr = ss(lc.A_lat, lc.B_lat(:, 2), [0 0 1 0], 0) * lag.dr;

fprintf('--- plant zeros ---\n');
fprintf('Gp zeros: %s\n', mat2str(sort(zero(Gp)), 6));
fprintf('Gq zeros: %s\n', mat2str(sort(zero(Gq)), 6));
fprintf('Gr zeros: %s\n', mat2str(sort(zero(Gr)), 6));

loops = {'p', Gp, wc.rate_p_rad_s; 'q', Gq, wc.rate_q_rad_s; 'r', Gr, wc.rate_r_rad_s};
for k = 1:size(loops, 1)
   nm = loops{k, 1};
   G = loops{k, 2};
   w = loops{k, 3};
   Kp = 1 / abs(freqresp(G, 1i * w));
   fprintf('\n=== rate loop %s, wc %.3g, Kp_nom %.6g ===\n', nm, w, Kp);
   for ks = [1, 0.5, 0.1]
      ctf = minreal(tf([ks * Kp * Tf, ks * Kp], [Tf, 1]));
      for s = [1, -1]
         p = pole(feedback(s * ctf * G, 1));
         ps = sort(p, 'ComparisonMethod', 'real');
         fprintf('  kp_scale %5.2f sgn %+d worst %12.4e poles %s\n', ks, s, ...
            max(real(p)), mat2str(ps(1:min(6, numel(ps))).', 5));
      end
   end
end

% outer: phi loop built on the closed roll-rate loop
Kp_p = 1 / abs(freqresp(Gp, 1i * wc.rate_p_rad_s));
ctf_p = minreal(tf([Kp_p * Tf, Kp_p], [Tf, 1]));
Gphi = feedback(ctf_p * Gp, 1) * tf(1, [1 0]);
fprintf('\n=== phi loop plant ===\n poles %s\n zeros %s\n', ...
   mat2str(sort(pole(Gphi)).', 5), mat2str(sort(zero(Gphi)).', 5));
for kp = [0.2, 0.5, 1, 2]
   for s = [1, -1]
      p = pole(feedback(s * kp * Gphi, 1));
      fprintf('  kp %5.2f sgn %+d worst %12.4e\n', kp, s, max(real(p)));
   end
end

% theta loop
Kp_q = 1 / abs(freqresp(Gq, 1i * wc.rate_q_rad_s));
ctf_q = minreal(tf([Kp_q * Tf, Kp_q], [Tf, 1]));
Gtheta = feedback(ctf_q * Gq, 1) * tf(1, [1 0]);
fprintf('\n=== theta loop plant ===\n poles %s\n zeros %s\n', ...
   mat2str(sort(pole(Gtheta)).', 5), mat2str(sort(zero(Gtheta)).', 5));
for kp = [0.2, 1, 2.5]
   for s = [1, -1]
      p = pole(feedback(s * kp * Gtheta, 1));
      fprintf('  kp %5.2f sgn %+d worst %12.4e\n', kp, s, max(real(p)));
   end
end
