% diag_vmin Locate the exact V_min defect in init_uacs_twin.
P = uacs_twin_params();
m = P.mass.m; g = P.env.g; rho = P.env.rho_sl; S = P.geom.S;
de_max = deg2rad(P.act.elevator.max_deg);
dfun = @(V) abs(-(P.aero.Cm0 + P.aero.Cm_alpha * ((m * g / (0.5 * rho * V^2 * S) - P.aero.CL0) ...
   / P.aero.CL_alpha)) / P.aero.Cm_de) / de_max;
out = fullfile(fileparts(mfilename('fullpath')), '_diag_vmin.log');
fid = fopen(out, 'w');
fprintf(fid, 'elevator_trim_frac %.6f, de_max %.2f deg, V_stall %.4f, stall_margin_factor %.3f\n', ...
   P.geom.elevator_trim_frac, P.act.elevator.max_deg, P.limits.V_stall_mps, P.limits.stall_margin_factor);
fprintf(fid, 'published V_min_stall %.4f, V_min %.4f\n', P.limits.V_min_stall_mps, P.limits.V_min_mps);
for V = [14 15 15.41 16 17 17.5 17.9 17.98 18.0 18.5 20 25 30]
   fprintf(fid, 'V %6.2f  de_frac %.6f\n', V, dfun(V));
end
lo = 1.0; hi = P.op.cruise.V_mps;
for k = 1:60
   mid = 0.5 * (lo + hi);
   if dfun(mid) > P.geom.elevator_trim_frac
      lo = mid;
   else
      hi = mid;
   end
end
fprintf(fid, 'recomputed V_min_trim %.4f\n', 0.5 * (lo + hi));
% what V_min would each fix give
fprintf(fid, 'option A: V_min_mps = 18.5 -> margin over stall %.3f\n', 18.5 / P.limits.V_stall_mps);
fprintf(fid, 'option B: need Cm_de or Sh/%s larger, or elevator_trim_frac above %.4f\n', ...
   'act.elevator.max_deg', dfun(P.limits.V_min_mps));
fclose(fid);
type(out);
