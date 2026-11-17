% diag_pid_zero Where the p/da right half plane zero comes from.
root = 'C:\Ash\Projects\DARES_UACS\MATLAB';
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'run'), fullfile(root, 'sim'));
clear functions; rehash path;
out = fullfile(root, 'results', '_diag_zero.log');
fid = fopen(out, 'w');
try
   init_uacs_twin('quiet', true);
   trim = trim_cruise(P, P.op.cruise);
   lin = linearize_at(P, trim, struct('step', 1e-6, 'verbose', false));
   lc = lin_contract(lin);
   A = lc.A_lat; B = lc.B_lat(:, 1); C = [0 1 0 0];
   fprintf(fid, 'rev %g mass %.2f, lateral poles %s\n', P.meta.revision, P.mass.m, ...
      mat2str(sort(eig(A), 'ComparisonMethod', 'real').', 6));
   fprintf(fid, 'raw p/da zeros (no actuator lag): %s\n', mat2str(sort(zero(ss(A, B, C, 0)), 'ComparisonMethod', 'real').', 8));

   syms s real
   nc = sym(0);
   for k = 1:4
      M = s * eye(4) - A;
      M(k, :) = C;
      nc = nc + B(k) * det(M);
   end
   np = sym2poly(expand(nc));
   fprintf(fid, 'numerator coefficients, s^3..s^0: %s\n', mat2str(np, 8));
   r = roots(np);
   fprintf(fid, 'numerator roots: %s\n', mat2str(sort(r, 'ComparisonMethod', 'real').', 8));

   % sensitivity of the small positive root to each input entry
   n0 = double(subs(nc, s, 0));
   fprintf(fid, 'n(0) = %.6e\n', n0);
   names = {'da->Cl', 'da->Cn'};
   for k = 1:4
      Bp = B;
      Bp(k) = 1.0;
      ncp = sym(0);
      for j = 1:4
         M = s * eye(4) - A;
         M(j, :) = C;
         ncp = ncp + Bp(j) * det(M);
      end
      rp = roots(sym2poly(expand(ncp)));
      rp = sort(rp, 'ComparisonMethod', 'real');
      fprintf(fid, '  unit input on B(%d): roots %s\n', k, mat2str(rp.', 8));
   end
   if isfield(lin, 'lat_state_names')
      fprintf(fid, 'lat states: %s\n', strjoin(lin.lat_state_names(:).', ','));
   end
   if isfield(lin, 'lat_input_names')
      fprintf(fid, 'lat inputs: %s\n', strjoin(lin.lat_input_names(:).', ','));
   end
catch ME
   fprintf(fid, 'FAILED %s | %s\n', ME.identifier, ME.message);
   for k = 1:numel(ME.stack)
      fprintf(fid, '  at %s line %d\n', ME.stack(k).name, ME.stack(k).line);
   end
end
fclose(fid);
type(out);
