% diag_rate_probe Probe the rate-loop machinery candidate by candidate.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_rate_probe.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
lc = lin_contract(lin);
Tf = P.ctrl.rate_filter_tau_s;
Gp = ss(lc.A_lat, lc.B_lat(:, 1), [0 1 0 0], 0) * tf(1, [P.act.aileron.tau_s, 1]);
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]);
for pair = {{'p', Gp, 12}, {'q', Gq, 8}}
   nm = pair{1}{1}; G = pair{1}{2}; wc = pair{1}{3};
   dp = struct();
   [Ag, Bg, Cg, Dg] = ssdata(G);
   [num, den] = ss2tf(Ag, Bg, Cg, Dg, 1);
   num = num / den(1); den = den / den(1);
   fprintf(fid, '\n=== %s target %.1f, plant num %s, den %s ===\n', nm, wc, ...
      mat2str(num, 4), mat2str(den, 4));
   for kp = [0.01 0.05 0.1 0.2 0.3 0.3679]
      T = minreal(feedback(tf(kp, 1) * tf(num, den), 1), 1e-9);
      p = pole(T);
      w = logspace(-5, 4, 120); w = w(:);
      H = (polyval(kp, 1i * w) ./ polyval(1, 1i * w)) .* ...
         (polyval(num, 1i * w) ./ polyval(den, 1i * w));
      mag = abs(H);
      ii = find(mag(1:end - 1) >= 1 & mag(2:end) < 1, 1);
      wcross = NaN;
      if ~isempty(ii), wcross = w(ii); end
      fprintf(fid, 'kp %8.4f poles %d worst %+10.4e |L|max %9.5f |L(0)| %9.2e crossover %10.5f ncross %d\n', ...
         kp, numel(p), max(real(p)), max(mag), mag(1), wcross, numel(ii));
   end
end
fclose(fid);
disp(fileread(out));
