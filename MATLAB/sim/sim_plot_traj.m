function figs = sim_plot_traj(traj, tag, root)
% SIM_PLOT_TRAJ One figure per scenario, saved as PNG, closed straight after.
%
% Line widths are 1 or 2 and marker sizes 1 or 2 by project convention, and the
% tile layout uses compact spacing. Only the figures created here are closed,
% by handle, so nothing else on the desktop is touched.

figs = {};
for k = 1:numel(traj)
   tr = traj(k);
   if ~isfield(tr, 'cmd') || isempty(tr.cmd)
      continue;
   end
   f = figure('Visible', 'off', 'Position', [100 100 1100 700]);
   tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
   t = tr.t;
   lw = 1.2;

   nexttile(tl);
   plot(t, tr.cmd(:, 1) * 180 / pi, 'k--', 'LineWidth', 1);
   hold on;
   plot(t, tr.x(:, 7) * 180 / pi, 'b-', 'LineWidth', lw);
   grid on; ylabel('phi (deg)'); legend({'cmd', 'response'}, 'Location', 'best');
   title('roll attitude');

   nexttile(tl);
   plot(t, tr.rate_cmd(:, 1), 'k--', 'LineWidth', 1);
   hold on;
   plot(t, tr.x(:, 4), 'b-', 'LineWidth', lw);
   plot(t, tr.x(:, 6), 'r-', 'LineWidth', lw);
   grid on; ylabel('rate (rad/s)'); legend({'p_c', 'p', 'r'}, 'Location', 'best');
   title('roll and yaw rate');

   nexttile(tl);
   plot(t, tr.cmd(:, 2) * 180 / pi, 'k--', 'LineWidth', 1);
   hold on;
   plot(t, tr.x(:, 8) * 180 / pi, 'b-', 'LineWidth', lw);
   grid on; ylabel('theta (deg)'); legend({'cmd', 'response'}, 'Location', 'best');
   title('pitch attitude');

   nexttile(tl);
   plot(t, tr.cmd(:, 4), 'k--', 'LineWidth', 1);
   hold on;
   plot(t, sqrt(sum(tr.x(:, 1:3).^2, 2)), 'b-', 'LineWidth', lw);
   grid on; ylabel('V (m/s)'); legend({'cmd', 'V'}, 'Location', 'best');
   title('airspeed');

   nexttile(tl);
   plot(t, tr.cmd(:, 5), 'k--', 'LineWidth', 1);
   hold on;
   plot(t, -tr.x(:, 12), 'b-', 'LineWidth', lw);
   grid on; ylabel('h (m)'); legend({'cmd', 'h'}, 'Location', 'best');
   title('altitude');

   nexttile(tl);
   plot(t, tr.act(:, 1) * 180 / pi, 'LineWidth', lw);
   hold on;
   plot(t, tr.act(:, 2) * 180 / pi, 'LineWidth', lw);
   plot(t, tr.act(:, 3) * 180 / pi, 'LineWidth', lw);
   grid on; ylabel('surface (deg)');
   legend({'da', 'de', 'dr'}, 'Location', 'best');
   title('actuator commands');

   title(tl, sprintf('%s: %s', strrep(tag, '_', '\_'), strrep(tr.name, '_', '\_')));
   xlabel(tl, 'time (s)');
   file = fullfile(root, 'results', sprintf('sim_fig_%s_%s.png', tag, tr.name));
   exportgraphics(f, file, 'Resolution', 110);
   close(f);
   figs{end + 1} = file; %#ok<AGROW>
end
end
