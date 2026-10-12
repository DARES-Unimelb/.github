function L = lqr_report_block(label, ch, P, trim, mode)
% LQR_REPORT_BLOCK Text lines for one LQR channel, for the design report.
%
%   L = lqr_report_block(label, ch, P, trim)          full block
%   L = lqr_report_block(label, ch, P, trim, 'alt')   one-line alternative
%
% ch is gains.long or gains.lat from design_lqr.m.

if nargin < 5
   mode = 'full';
end

if strcmp(mode, 'alt')
   L = {};
   L{end+1} = sprintf('%s: K = %s', label, mat2str(ch.K, 6));
   L{end+1} = sprintf('  poles %s', mat2str(ch.poles, 5));
   L{end+1} = sprintf('  worst margins: gm %.3f, pm %.2f deg, dm %.5f s', ...
      min([ch.margins.gm]), min([ch.margins.pm_deg]), min([ch.margins.dm_s]));
   L{end+1} = sprintf('  steady-state error without integrator %.4f, with %.2e', ...
      ch.ss_test.unaug.error, abs(ch.ss_test.aug.error));
   return;
end

L = {};
L{end+1} = sprintf('--- %s ---', label);
L{end+1} = sprintf('A = %s', mat2str(ch.A, 5));
L{end+1} = sprintf('B = %s', mat2str(ch.B, 5));
L{end+1} = sprintf('Q = diag(%s)', mat2str(diag(ch.Q)', 6));
L{end+1} = sprintf('R = diag(%s)', mat2str(diag(ch.R)', 6));
L{end+1} = sprintf('K augmented   (%dx%d) = %s', size(ch.K, 1), size(ch.K, 2), mat2str(ch.K, 8));
L{end+1} = sprintf('K unaugmented (%dx%d) = %s', size(ch.K_unaug, 1), size(ch.K_unaug, 2), ...
   mat2str(ch.K_unaug, 8));
L{end+1} = sprintf('closed-loop poles augmented   : %s', mat2str(ch.poles, 6));
L{end+1} = sprintf('closed-loop poles unaugmented : %s', mat2str(ch.poles_unaug, 6));
L{end+1} = sprintf('K = R\\B''X from care, max difference %.3e', ch.dK_care);
L{end+1} = sprintf('operating point (absolute) x_op = %s  [%s]', mat2str(ch.x_op', 6), ...
   strjoin(ch.xop_names, ' '));
L{end+1} = sprintf('trim input u_op = %s  [%s]', mat2str(ch.u_op', 6), strjoin(ch.u_names, ' '));
L{end+1} = 'margins, one input broken at a time with the other loop closed:';
L{end+1} = sprintf('  %-4s %8s %8s %9s %9s %9s %9s %6s', 'loop', 'gm', 'gm_dB', 'pm_deg', ...
   'dm_s', 'wcg', 'wcp', 'stable');
for j = 1:numel(ch.margins)
   m = ch.margins(j);
   L{end+1} = sprintf('  %-4s %8.3f %8.2f %9.2f %9.5f %9.3f %9.3f %6d', m.loop, m.gm, ...
      m.gm_db, m.pm_deg, m.dm_s, m.wcg_rad_s, m.wcp_rad_s, m.loop_stable);
end
L{end+1} = 'the same loops without the integrator, for comparison:';
for j = 1:numel(ch.margins_unaug)
   m = ch.margins_unaug(j);
   L{end+1} = sprintf('  %-4s gm %8.3f, pm %8.2f deg, dm %9.5f s', m.loop, m.gm, m.pm_deg, m.dm_s);
end
L{end+1} = sprintf('minimum delay margin %.5f s at %.3f rad/s', ...
   min([ch.margins.dm_s]), ch.margins(1).wcp_rad_s);
L{end+1} = '';
end
