function uacs_ensure()
% UACS_ENSURE Re-initialise the base workspace only when something is actually missing.
%
% Set as a model's InitFcn by run/build_bench_models.m. PreLoadFcn runs uacs_go when the
% model is opened, which is what makes a double-click on the .slx work; InitFcn runs
% before EVERY simulation, and pointing it straight at uacs_go meant every press of Run
% repeated the whole parameter chain, the trim solve and its forty lines of console
% output. On a bench that is meant to be flown by dragging sliders that delay is the
% difference between a tool and a chore.
%
% The check is deliberately on variables the models cannot compile without, taken from
% the five ways a model reads base: the parameter struct, the scenario inputs, the design
% push, the trim's feedforward and the interactive bench's own knobs. If every one of them
% is present nothing runs, and a stale-but-complete workspace is left alone: the remedy for
% a knob edit is to run uacs_go from the editor, which is a deliberate act, not a side
% effect of pressing Run.

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
