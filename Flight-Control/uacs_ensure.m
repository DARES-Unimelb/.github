function uacs_ensure()
% UACS_ENSURE Re-initialise the base workspace only when something is actually missing.
%
% Set as a model InitFcn by run/build_bench_models.m, so it runs before every simulation.
% A complete workspace is left alone: after a knob edit, run uacs_go from the editor.

need = {'P', 'sim_t_end', 'sim_flight_mode', 'sim_thr_trim', 'sim_stick_ts', ...
   'sim_wind_ned', 'sim_u_trim_surf', 'sim_T_trim', 'sim_pid_theta_Isat', ...
   'sim_rate_cmd_lim_p', 'uacs_last_design', ...
   'sim_stick_roll', 'sim_stick_pitch', 'sim_stick_yaw', 'sim_stick_thr', ...
   'sim_ctrl_sel', 'sim_js_on', 'sim_dist_wind_ned', 'sim_alpha_stall_abs', ...
   'sim_bench_t_end'};
missing = {};
for k = 1:numel(need)
   if evalin('base', sprintf('exist(''%s'', ''var'')', need{k})) == 0
      missing{end + 1} = need{k}; %#ok<AGROW>
   end
end
if isempty(missing)
   return;
end
fprintf('uacs_ensure: base 工作区缺少 %s，正在运行 uacs_go 初始化...\n', ...
   strjoin(missing, ', '));
here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'uacs_go.m'));
end
