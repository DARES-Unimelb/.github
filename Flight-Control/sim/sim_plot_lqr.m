function file = sim_plot_lqr(traj, root)
% SIM_PLOT_LQR One figure per LQR scenario, saved as PNG, closed by handle.
%
% The LQR model logs a reference in deviation coordinates rather than a pilot
% command vector, so it gets its own panels instead of the manual model's.
% Line widths 1 or 2, compact tiles, only this figure is closed.

files = {};
for k = 1:numel(traj)
   tr = traj(k);
   f = figure('Visible', 'off', 'Position', [100 100 1100 700]);
   tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
   t = tr.t;
   lw = 1.2;
   V = sqrt(sum(tr.x(:, 1:3).^2, 2));

   nexttile(tl);
   plot(t, V, 'b-', 'LineWidth', lw);
   grid on; ylabel('V (m/s)'); title('airspeed');
   nexttile(tl);
   plot(t, tr.x(:, 7) * 180 / pi, 'b-', 'LineWidth', lw);
   grid on; ylabel('phi (deg)'); title('bank angle');
   nexttile(tl);
   plot(t, tr.u_dev(:, 1), 'LineWidth', lw);
   hold on;
   plot(t, tr.u_dev(:, 2), 'LineWidth', lw);
   plot(t, tr.u_dev(:, 3), 'LineWidth', lw);
   plot(t, tr.u_dev(:, 4), 'LineWidth', lw);
   grid on; ylabel('u dev'); legend({'dda', 'dde', 'ddr', 'dT'}, 'Location', 'best');
   title('control increments');
   nexttile(tl);
   plot(t, tr.xi(:, 1), 'LineWidth', lw);
   hold on;
   plot(t, tr.xi(:, 2), 'LineWidth', lw);
   grid on; ylabel('z'); legend({'z_V', 'z_phi'}, 'Location', 'best');
   title('integral states');
   nexttile(tl);
   plot(t, tr.act(:, 1) * 180 / pi, 'LineWidth', lw);
   hold on;
   plot(t, tr.act(:, 2) * 180 / pi, 'LineWidth', lw);
   plot(t, tr.act(:, 3) * 180 / pi, 'LineWidth', lw);
   grid on; ylabel('surface (deg)'); legend({'da', 'de', 'dr'}, 'Location', 'best');
   title('actuator commands');
   nexttile(tl);
   plot(t, tr.act(:, 4), 'LineWidth', lw);
   hold on;
   plot(t, tr.act(:, 5), 'LineWidth', lw);
   grid on; ylabel('throttle'); legend({'left', 'right'}, 'Location', 'best');
   title('throttles');

   title(tl, sprintf('lqr: %s', strrep(tr.name, '_', '\_')));
   xlabel(tl, 'time (s)');
   file = fullfile(root, 'results', sprintf('sim_fig_lqr_%s.png', tr.name));
   exportgraphics(f, file, 'Resolution', 110);
   close(f);
   files{end + 1} = file; %#ok<AGROW>
end
if isempty(files)
   file = '';
else
   file = files{1};
end
end
