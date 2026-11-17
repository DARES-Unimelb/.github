% diag_qdebug Run the q loop and dump the full error stack plus local state.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'), '-begin');
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_qdebug.log');
fid = fopen(out, 'w');
P = uacs_twin_params();
assignin('base', 'P', P);
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
try
   gp = design_pid(P, lin, struct('which', {{'q'}}, 'verbose', false));
   g = gp.q; v = gp.verify.q;
   fprintf(fid, 'OK q Kp %.6g pol %+d stable %d wc %.4f pm %.2f\n', g.Kp, g.sign, v.stable, ...
      v.wc_rad_s, v.pm_deg);
catch ME
   fprintf(fid, 'FAILED %s\n  %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
% Manual reproduction of the failing call path for one candidate.
lc = lin_contract(lin);
Gq = ss(lc.A_long, lc.B_long(:, 1), [0 0 1 0], 0) * tf(1, [P.act.elevator.tau_s, 1]);
[Ag, Bg, Cg, Dg] = ssdata(Gq);
[num, den] = ss2tf(Ag, Bg, Cg, Dg, 1);
num = num / den(1);
den = den / den(1);
fprintf(fid, 'plant num %s (size %s), den %s (size %s)\n', mat2str(num, 4), ...
   mat2str(size(num)), mat2str(den, 4), mat2str(size(den)));
for kp = [0.05 0.2 0.4]
   cn = kp;
   cd = 1;
   a = conv(cn, num);
   b = conv(cn, num) + conv(cd, den);
   fprintf(fid, 'kp %.3f: size a %s, size b %s\n', kp, mat2str(size(a)), mat2str(size(b)));
   try
      T = minreal(feedback(tf(cn, cd) * tf(num, den), 1), 1e-7);
      p = pole(T);
      fprintf(fid, '   poles %d, worst %+.4e\n', numel(p), max(real(p)));
   catch ERR
      fprintf(fid, '   tf path failed %s | %s\n', ERR.identifier, ERR.message);
   end
end
fclose(fid);
disp(fileread(out));
