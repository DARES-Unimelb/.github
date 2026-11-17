function report_sizing()
% REPORT_SIZING Preliminary sizing pass for the 20 kg twin-engine UAV.
%
% Prints the sizing table, the authority budget and the implied modes, and draws
% the drag and thrust-required curves used by the design notes.
%
% Run:  run_matlab_file('report_sizing.m')  after init_uacs_twin on the path.

P = uacs_twin_params();
S = sizing_summary(P);

fprintf('==== WING LOADING AND STALL ====\n');
fprintf('W              %8.2f N   (%.2f kg)\n', P.mass.m * P.env.g, P.mass.m);
fprintf('wing loading   %8.2f N/m^2  (%.2f kg/m^2)\n', ...
    S.wing_loading_Npm2, S.wing_loading_kgpm2);
fprintf('V_stall        %8.2f m/s  (CL_max %.2f)\n', S.V_stall, P.aero.CL_max);

fprintf('\n==== OPERATING POINTS ====\n');
fprintf('%6s %6s %8s %7s %7s %7s %8s %9s %9s\n', ...
    'case', 'V', 'q', 'n', 'CL', 'L/D', 'T_req', 'P_elec', 'Re');
for k = 1:numel(S.op)
    o = S.op(k);
    fprintf('%6s %6.1f %8.0f %7.2f %7.3f %7.2f %8.2f %9.0f %9.2e\n', ...
        o.name, o.V, o.q, o.n, o.CL, o.L_over_D, o.thrust_req_N, o.P_elec_W, o.Re);
end

fprintf('\n==== ENDURANCE AT CRUISE ====\n');
fprintf('electric power at cruise      %8.0f W total, %.0f W per motor\n', ...
    S.endurance.cruise_P_elec_W, S.op(1).P_per_motor_W);
fprintf('energy for %.0f min            %8.1f Wh\n', ...
    S.endurance.target_s / 60, S.endurance.energy_Wh);
fprintf('pack at 80%% DoD               %8.1f Wh\n', S.endurance.pack_Wh);
fprintf('pack mass at 200 Wh/kg        %8.2f kg  (%.1f%% of MTOW)\n', ...
    S.endurance.pack_mass_kg_at_200Whpkg, 100 * S.endurance.pack_mass_fraction);

fprintf('\n==== PROPULSION SANITY ====\n');
fprintf('peak electrical per motor     %8.0f W\n', P.prop.power.peak_per_motor_W);
fprintf('static thrust per motor       %8.1f N  (derived from peak power)\n', P.prop.T_static_max);
fprintf('static thrust to weight       %8.2f\n', S.prop.thrust_to_weight_static);
fprintf('static rpm / tip Mach         %8.0f rpm / %.2f\n', ...
    S.prop.n_static_rpm, S.prop.tip_mach_static);
fprintf('cruise thrust per motor       %8.2f N\n', S.prop.thrust_cruise_per_motor_N);
fprintf('cruise throttle setting       %8.3f  (margin to full %.3f)\n', ...
    S.prop.throttle_cruise, S.prop.throttle_margin_to_full);
fprintf('peak power loading            %8.0f W/kg\n', S.prop.power_loading_W_per_kg);

fprintf('\n==== CONTROL AUTHORITY AT CRUISE ====\n');
A = S.authority;
fprintf('rudder    full      yaw       %8.1f N*m\n', A.rudder.yaw_moment_Nm);
fprintf('aileron   full      roll      %8.1f N*m\n', A.aileron.roll_moment_Nm);
fprintf('elevator  full      pitch     %8.1f N*m\n', A.elevator.pitch_moment_Nm);
fprintf('diff thr  d_eff %.3f yaw       %8.1f N*m   (%.3f of rudder, limited by %s)\n', ...
    A.diff_thrust.d_effective, A.diff_thrust.yaw_moment_Nm, ...
    A.diff_thrust.ratio_to_rudder, A.diff_thrust.limiting_factor);
fprintf('diff thr  at cap %.3f yaw      %8.1f N*m   (%.3f of rudder, not reachable in cruise)\n', ...
    A.diff_thrust.d_cap, A.diff_thrust.yaw_moment_at_cap_Nm, ...
    A.diff_thrust.ratio_to_rudder_at_cap);
fprintf('engine out          yaw       %8.1f N*m   (%.2f of rudder)\n', ...
    A.engine_out.yaw_moment_Nm, A.engine_out.ratio_to_rudder);
fprintf('engine out          V_mc      %8.2f m/s  (%.2f x cruise, %.2f x stall)\n', ...
    A.engine_out.V_mc, A.engine_out.V_mc_vs_cruise, A.engine_out.V_mc_vs_stall);
fprintf('engine out          pitch     %8.1f N*m   (%.2f of elevator)\n', ...
    A.engine_out.pitch_moment_Nm, A.engine_out.pitch_ratio_to_elevator);

fprintf('\n==== STABILITY AND MODES ====\n');
fprintf('neutral point ahead of ref    %8.3f m\n', S.stability.x_np_minus_ref_m);
fprintf('static margin                 %8.1f %% cbar\n', ...
    100 * S.stability.static_margin_frac_cbar);
fprintf('short period (approx)         %8.2f rad/s (%.2f Hz)\n', ...
    S.modes.short_period_approx_rad_s, S.modes.short_period_approx_rad_s / (2 * pi));
fprintf('roll subsidence time constant %8.3f s\n', S.modes.roll_subsidence_tau_s);
fprintf('dutch roll (approx)           %8.2f rad/s (%.2f Hz)\n', ...
    S.modes.dutch_roll_approx_rad_s, S.modes.dutch_roll_approx_rad_s / (2 * pi));

%% ---- figure: drag and thrust required versus speed ----
Vv = linspace(max(1.15 * S.V_stall, 8), 45, 200);
CLv = (P.mass.m * P.env.g) ./ (0.5 * P.env.rho_sl * Vv.^2 * P.geom.S);
CDv = P.aero.CD0 + P.aero.k_induced * CLv.^2;
Dv = 0.5 * P.env.rho_sl * Vv.^2 * P.geom.S .* CDv;
Pshaft = Dv .* Vv;
Pelec = Pshaft / P.prop.power.eta_total_cruise;
LDv = CLv ./ CDv;

% Available thrust from the same momentum-theory map the plant uses, both motors
% at full throttle. No fitted ram-drag slope any more: an independently entered
% k_ram disagreed with the theory by a factor of four.
Tavail = 2 * prop_thrust_from_throttle(P, 1.0, Vv);
Tavail(Tavail < 0) = NaN;

f = figure('Name', 'UACS twin sizing', 'Color', 'w');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl);
plot(Vv, Dv, 'LineWidth', 2); hold on;
yline(P.prop.n_motors * P.prop.T_static_max, 'LineWidth', 1);
plot(Vv, Tavail, 'LineWidth', 2);
yline(0, 'LineWidth', 1);
grid on;
xlabel('true airspeed (m/s)'); ylabel('force (N)');
title('Drag and available thrust');
legend({'drag at 20 kg', 'static thrust limit', 'available thrust', ''}, ...
    'Location', 'northwest', 'FontSize', 8);

nexttile(tl);
plot(Vv, Pelec, 'LineWidth', 2); hold on;
yline(P.prop.power.peak_per_motor_W * P.prop.n_motors, 'LineWidth', 1);
grid on;
xlabel('true airspeed (m/s)'); ylabel('electrical power (W)');
title('Power required in level flight');
legend({'power required', 'installed peak'}, 'Location', 'northeast', 'FontSize', 8);

nexttile(tl);
plot(Vv, LDv, 'LineWidth', 2);
grid on;
xlabel('true airspeed (m/s)'); ylabel('L/D');
title('Lift to drag ratio');

nexttile(tl);
plot(Vv, CLv, 'LineWidth', 2); hold on;
yline(P.aero.CL_max, 'LineWidth', 1);
xline(S.V_stall, 'LineWidth', 1);
grid on;
xlabel('true airspeed (m/s)'); ylabel('C_L required');
title('Lift coefficient required');
legend({'C_L', 'C_L_{max}', 'stall speed'}, 'Location', 'northeast', 'FontSize', 8);

%% ---- print the summary block that the report quotes ----
fprintf('\n==== REPORT BLOCK ====\n');
fprintf('| item | value |\n|---|---|\n');
fprintf('| take-off mass | %.1f kg |\n', P.mass.m);
fprintf('| wing span / area | %.2f m / %.2f m^2 (AR %.2f) |\n', ...
    P.geom.b, P.geom.S, P.geom.AR);
fprintf('| wing loading | %.1f N/m^2 |\n', S.wing_loading_Npm2);
fprintf('| stall speed | %.1f m/s |\n', S.V_stall);
fprintf('| cruise speed | %.1f m/s |\n', P.op.cruise.V_mps);
fprintf('| cruise C_L | %.3f (%.0f%% of C_Lmax) |\n', ...
    S.op(1).CL, 100 * S.op(1).CL_utilisation);
fprintf('| cruise L/D | %.1f |\n', S.op(1).L_over_D);
fprintf('| cruise electrical power | %.0f W total |\n', S.endurance.cruise_P_elec_W);
fprintf('| 20 min energy | %.0f Wh, pack %.2f kg at 200 Wh/kg |\n', ...
    S.endurance.energy_Wh, S.endurance.pack_mass_kg_at_200Whpkg);
fprintf('| peak electrical power | %.1f kW installed, %.0f W/kg |\n', ...
    P.prop.power.peak_per_motor_W * P.prop.n_motors / 1000, S.prop.power_loading_W_per_kg);
fprintf('| static thrust | %.1f N per motor (T/W %.2f) |\n', ...
    P.prop.T_static_max, S.prop.thrust_to_weight_static);
fprintf('| cruise throttle | %.3f |\n', S.prop.throttle_cruise);
fprintf('| rudder yaw authority | %.1f N*m |\n', A.rudder.yaw_moment_Nm);
fprintf('| differential thrust yaw | %.1f N*m (%.1f%% of rudder) |\n', ...
    A.diff_thrust.yaw_moment_Nm, 100 * A.diff_thrust.ratio_to_rudder);
fprintf('| differential thrust, structural cap | %.1f N*m (%.1f%% of rudder) |\n', ...
    A.diff_thrust.yaw_moment_at_cap_Nm, 100 * A.diff_thrust.ratio_to_rudder_at_cap);
fprintf('| engine-out yaw at full power | %.0f N*m (%.2f x rudder) |\n', ...
    A.engine_out.yaw_moment_Nm, A.engine_out.ratio_to_rudder);
fprintf('| V_mc on rudder alone | %.1f m/s (%.1f x stall) |\n', ...
    A.engine_out.V_mc, A.engine_out.V_mc_vs_stall);
fprintf('| static margin | %.1f %% cbar |\n', 100 * S.stability.static_margin_frac_cbar);
fprintf('DONE report_sizing\n');
end
