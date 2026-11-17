function file = sim_plot_wind(case_wind, case_nowind, root)
% SIM_PLOT_WIND Write the crosswind comparison PNG and return its path
%
%   case_wind, case_nowind   trajectory structs with a field x
%   root                     project root; the figure goes to <root>/results

f = figure('Visible', 'off', 'Position', [100 100 1100 700]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
lw = 1.2;
tw = case_wind.t;
tn = case_nowind.t;

nexttile(tl);
plot(case_nowind.x(:, 11), case_nowind.x(:, 10), 'k--', 'LineWidth', 1);
hold on;
plot(case_wind.x(:, 11), case_wind.x(:, 10), 'b-', 'LineWidth', lw);
grid on; axis equal;
xlabel('east (m)'); ylabel('north (m)');
legend({'no wind', '5 m/s crosswind'}, 'Location', 'best');
title('ground track');

nexttile(tl);
plot(tn, case_nowind.x(:, 7) * 180 / pi, 'k--', 'LineWidth', 1);
hold on;
plot(tw, case_wind.x(:, 7) * 180 / pi, 'b-', 'LineWidth', lw);
grid on; xlabel('time (s)'); ylabel('phi (deg)');
legend({'no wind', 'crosswind'}, 'Location', 'best');
title('bank angle convergence');

nexttile(tl);
be = asin(max(-1, min(1, case_wind.x(:, 2) ./ max(sqrt(sum(case_wind.x(:, 1:3).^2, 2)), 1e-9))));
plot(tw, be * 180 / pi, 'b-', 'LineWidth', lw);
grid on; xlabel('time (s)'); ylabel('beta (deg)');
title('sideslip in the crosswind');

nexttile(tl);
plot(tn, case_nowind.x(:, 11) - case_nowind.x(1, 11), 'k--', 'LineWidth', 1);
hold on;
plot(tw, case_wind.x(:, 11) - case_wind.x(1, 11), 'b-', 'LineWidth', lw);
grid on; xlabel('time (s)'); ylabel('east displacement (m)');
legend({'no wind', 'crosswind'}, 'Location', 'best');
title('east drift');

title(tl, 'crosswind scenario');
file = fullfile(root, 'results', 'sim_fig_wind_crosswind.png');
exportgraphics(f, file, 'Resolution', 110);
close(f);
end
