function uacs_apply_config(uacs)
% UACS_APPLY_CONFIG Turn the uacs_go.m knobs into a ready-to-run environment.
%
% uacs_go.m only declares the knobs; every mechanical step happens here:
% locate the directories and add the search path, map the knobs onto dotted
% field paths of params/uacs_twin_params.m and keep only the entries that really
% changed, call init_uacs_twin to build the parameters, obtain the design
% (delivery cache with no overrides, recomputed trim, linearisation and gains
% with overrides), push the design variables into the base workspace, and
% publish the bench's own variables
%
% The override happens inside uacs_twin_params.m, where each group's literals end
% and its derivation begins, so the SIZED quantities below recompute from the
% overridden numbers; a typo in a knob name errors instead of adding a parameter,
% and overridden derivations never overwrite the delivered caches

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), ...
   fullfile(root, 'model'), root, '-begin');

%% Override table
[k, paths, vals] = local_knob_paths(uacs);
frozen = uacs_twin_params();
over = cell(0, 2);
for ii = 1:numel(paths)
   if ~local_same(vals{ii}, local_get(frozen, paths{ii}))
      over(end + 1, :) = {paths{ii}, vals{ii}}; %#ok<AGROW>
   end
end
clear k paths vals frozen ii;

init_uacs_twin('quiet', true, 'override', over);
P = evalin('base', 'P');
D = local_design(root, P, over);

sim_push_design_vars(P, D);
local_publish_bench(P, uacs);

% Neutral default scenario, so the models that read stick and wind from the base
% workspace also compile and run at this point
sim_push_scenario(struct('t_end', P.sim.t_end_s, 'x0', D.trim.x_star, ...
   'stick', [0 0 0 0 0; P.sim.t_end_s 0 0 0 0], ...
   'wind', [0 0 0 0; P.sim.t_end_s 0 0 0], 'flight_mode', 0));
assignin('base', 'uacs_last_design', D);

local_banner(P, D, over, uacs);
end

%% Local helpers
function [k, paths, vals] = local_knob_paths(uacs)
% Knob struct to dotted field paths of params/uacs_twin_params.m; a misspelled
% field name errors in local_get
%
% The aerodynamic derivatives (uacs_aero) are not knobs: their literals and
% derived values are interleaved in that parameter file, and changing them means
% editing params/uacs_twin_params.m directly
k = struct();
k.mass.items = uacs.mass;
k.mass.m = sum(cell2mat(uacs.mass(:, 2)));
k.geom.b = uacs.wing.span_m;
k.geom.AR = uacs.wing.aspect_ratio;
k.geom.taper = uacs.wing.taper;
k.geom.dihedral_deg = uacs.wing.dihedral_deg;
k.geom.iwing_deg = uacs.wing.incidence_deg;
k.geom.Sh_over_S = uacs.tail.h_area_ratio;
k.geom.lh = uacs.tail.h_arm_m;
k.geom.ih_deg = uacs.tail.h_incidence_deg;
k.geom.Sv = uacs.tail.v_area_m2;
k.geom.lv = uacs.tail.v_arm_m;
k.act.aileron = struct('max_deg', uacs.act.aileron_max_deg, ...
   'rate_deg_s', uacs.act.aileron_rate_deg_s, 'tau_s', uacs.act.aileron_tau_s);
k.act.elevator = struct('max_deg', uacs.act.elevator_max_deg, ...
   'rate_deg_s', uacs.act.elevator_rate_deg_s, 'tau_s', uacs.act.elevator_tau_s);
k.act.rudder = struct('max_deg', uacs.act.rudder_max_deg, ...
   'rate_deg_s', uacs.act.rudder_rate_deg_s, 'tau_s', uacs.act.rudder_tau_s);
k.prop.power.peak_per_motor_W = uacs.prop.peak_power_per_motor_W;
k.prop.power.figure_of_merit = uacs.prop.figure_of_merit;
k.prop.D = uacs.prop.diameter_m;
k.prop.esc.tau_s = uacs.prop.esc_tau_s;
k.prop.y_motor = uacs.prop.y_motor_m;
k.prop.z_motor = uacs.prop.z_motor_m;
k.prop.differential_limit = uacs.prop.differential_limit;
k.ctrl.bw = struct('rate_p_rad_s', uacs.ctrl.bw_rate_p_rad_s, ...
   'rate_q_rad_s', uacs.ctrl.bw_rate_q_rad_s, ...
   'rate_r_rad_s', uacs.ctrl.bw_rate_r_rad_s, ...
   'att_phi_rad_s', uacs.ctrl.bw_att_phi_rad_s, ...
   'att_theta_rad_s', uacs.ctrl.bw_att_theta_rad_s, ...
   'speed_rad_s', uacs.ctrl.bw_speed_rad_s, ...
   'height_rad_s', uacs.ctrl.bw_height_rad_s);
k.ctrl.rate_filter_tau_s = uacs.ctrl.rate_filter_tau_s;
k.ctrl.anti_windup_Kb = uacs.ctrl.anti_windup_Kb;
k.ctrl.int_state_margin = uacs.ctrl.int_state_margin;
k.ctrl.rate_cmd_frac = uacs.ctrl.rate_cmd_frac;
k.ctrl.int_limit = struct('phi', deg2rad(uacs.ctrl.int_limit_phi_deg), ...
   'theta', deg2rad(uacs.ctrl.int_limit_theta_deg), ...
   'V', uacs.ctrl.int_limit_V_mps, 'h', uacs.ctrl.int_limit_h_m);
k.lqr.long.q_diag = uacs.lqr.long_q;
k.lqr.long.r_diag = uacs.lqr.long_r;
k.lqr.lat.q_diag = uacs.lqr.lat_q;
k.lqr.lat.r_diag = uacs.lqr.lat_r;
k.limits.bank_max_deg = uacs.limits.bank_max_deg;
k.limits.pitch_max_deg = uacs.limits.pitch_max_deg;
k.limits.V_max_mps = uacs.limits.V_max_mps;
k.limits.climb_rate_max_mps = uacs.limits.climb_rate_max_mps;
k.limits.h_max_m = uacs.limits.h_max_m;
k.limits.nz_max = uacs.limits.nz_max;
k.limits.nz_min = uacs.limits.nz_min;
k.op.cruise.V_mps = uacs.cruise.V_mps;
k.op.cruise.h_m = uacs.cruise.h_m;
k.atmo.model = uacs.atmo.model;
k.sim.Ts_ctrl = uacs.sim.Ts_ctrl_s;
k.sim.Ts_outer = uacs.sim.Ts_outer_s;
k.sim.Ts_alloc = uacs.sim.Ts_alloc_s;
k.sim.Ts_sensor = uacs.sim.Ts_sensor_s;
k.sim.t_end_s = uacs.sim.t_end_s;
[paths, vals] = local_flatten(k, '', {}, {});
end

function D = local_design(root, P, over)
% Overridden parameters need the trim, linearisation and gains recomputed; the
% overridden derivation is cached separately and hits when the override table
% matches entry for entry
cache = fullfile(root, 'results', 'bench_design_cache.mat');
D = [];
if ~isempty(over) && exist(cache, 'file')
   S = load(cache);
   if isfield(S, 'over') && isequaln(S.over, over) && isfield(S, 'D')
      D = S.D;
      fprintf('bench 设计缓存命中（覆盖项与上次相同）\n');
   end
end
if ~isempty(D)
   return
end
if isempty(over)
   D = sim_load_design(P);
   return
end
fprintf('检测到参数覆盖，正在重算配平、线性化与增益，约 1 到 4 分钟...\n');
trim = trim_cruise(P, P.op.cruise);
lin = linearize_at(P, trim, struct('step', 1e-6));
D = struct('trim', trim, 'lin', lin, 'gains_pid', design_pid(P, lin), ...
   'gains_lqr', design_lqr(P, lin), 'src', struct('branch', 'uacs_go_override_live'));
save(cache, 'over', 'D');
fprintf('bench 设计缓存已写入 %s\n', cache);
end

function local_publish_bench(P, uacs)
% The bench's own variables; they are not P fields, because stick trim, mode,
% controller selection and command source are choices about how to view the
% aircraft rather than parameters
%
% The throttle slider default is the STICK position that trims the aircraft at
% cruise, not the solved trim throttle: pilot_to_command maps stick linearly to
% the speed command, so the trim throttle would request a lower speed and the
% aircraft would decelerate away from trim
V_min = P.limits.V_min_mps;
V_max = P.limits.V_max_mps;
thr = (P.op.cruise.V_mps - V_min) / (V_max - V_min);
assignin('base', 'sim_stick_roll', 0);
assignin('base', 'sim_stick_pitch', 0);
assignin('base', 'sim_stick_yaw', 0);
assignin('base', 'sim_stick_thr', thr);
assignin('base', 'sim_thr_trim', thr);
assignin('base', 'sim_flight_mode', 0);
assignin('base', 'sim_ctrl_sel', 0);
assignin('base', 'sim_js_on', 0);
assignin('base', 'sim_js_id', uacs.joystick.id);
assignin('base', 'sim_js_pitch_sign', uacs.joystick.pitch_sign);
assignin('base', 'sim_dist_wind_ned', uacs.dist.wind_ned_mps(:));
assignin('base', 'sim_dist_wind_e', 0);
assignin('base', 'sim_dist_gust_amp', double(uacs.dist.gust.on) * uacs.dist.gust.amp_mps);
assignin('base', 'sim_dist_gust_freq', 2 * pi * uacs.dist.gust.freq_hz);
assignin('base', 'sim_dist_gust_dir', uacs.dist.gust.dir_ned(:));
assignin('base', 'sim_alpha_stall_abs', abs(deg2rad(P.aero.alpha_stall_deg)));
assignin('base', 'sim_bench_t_end', uacs.sim.bench_run_s);
end

function local_banner(P, D, over, uacs)
fprintf('\n================ UACS 已就绪 ================\n');
fprintf('基准 rev %g / %s，设计分支 %s\n', P.meta.revision, P.meta.version, D.src.branch);
if isempty(over)
   fprintf('参数与冻结基准一致（无覆盖）\n');
else
   fprintf('!! 本轮有 %d 项参数覆盖，结果不得引用 rev %g 的结论：\n', ...
      size(over, 1), P.meta.revision);
   for ii = 1:numel(P.meta.override_report)
      fprintf('     %s\n', P.meta.override_report{ii});
   end
end
fprintf('总重 %.3f kg，重心 x = %+.4f m，翼展 %.3f m，面积 %.4f m^2\n', ...
   P.mass.m, P.mass.x_cg, P.geom.b, P.geom.S);
fprintf('惯量 Ixx %.4f  Iyy %.4f  Izz %.4f  Ixz %+.4f kg*m^2\n', ...
   P.mass.Ixx_cg, P.mass.Iyy_cg, P.mass.Izz_cg, P.mass.Ixz_cg);
fprintf('包线 V %.2f 到 %.2f m/s，滚转 ±%.0f 度，俯仰 ±%.0f 度\n', ...
   P.limits.V_min_mps, P.limits.V_max_mps, P.limits.bank_max_deg, P.limits.pitch_max_deg);
fprintf('配平 V %.2f m/s，攻角 %.4f 度，升降舵 %.4f 度，油门 %.4f\n', ...
   D.trim.V_mps, D.trim.alpha_deg, D.trim.de_deg, D.trim.throttle);
fprintf('仿真时长：受验模型 sim_t_end = %.0f s，交互台架 sim_bench_t_end = %.0f s\n', ...
   P.sim.t_end_s, uacs.sim.bench_run_s);
fprintf('扰动：常值风 [%.2f %.2f %.2f] m/s，阵风 %s，幅度 %.2f m/s，频率 %.2f Hz\n', ...
   uacs.dist.wind_ned_mps(1), uacs.dist.wind_ned_mps(2), uacs.dist.wind_ned_mps(3), ...
   local_onoff(uacs.dist.gust.on), uacs.dist.gust.amp_mps, uacs.dist.gust.freq_hz);
fprintf('手柄：JoystickID = %s，俯仰符号 %+g（辅助通道，操纵台默认走虚拟杆）\n', ...
   uacs.joystick.id, uacs.joystick.pitch_sign);
fprintf('PID 增益（极性与符号已折入）：\n');
loops = {'p', 'q', 'r', 'phi', 'theta', 'V', 'h'};
for ii = 1:numel(loops)
   g = D.gains_pid.(loops{ii});
   fprintf('   %-6s Kp %+11.6f  Ki %+11.6f  Kd %+11.6f\n', g.name, g.Kp, g.Ki, g.Kd);
end
fprintf('LQR 纵向 K = %s\n', mat2str(D.gains_lqr.long.K, 5));
fprintf('LQR 横航 K = %s\n', mat2str(D.gains_lqr.lat.K, 5));
fprintf('现在可以：\n');
fprintf('   open_system(''uacs_flight_bench'')   飞行台架，拖操纵台上的旋钮与滑杆直接飞\n');
fprintf('   open_system(''uacs_aero_test'')      气动测试，拖迎角/侧滑角/速度看力与力矩\n');
fprintf('   open_system(''uacs_manual_pid'')     受验的 PID 模型（跑预置场景用）\n');
fprintf('   也可以直接双击 model 目录下的 .slx，模型会自己调本文件\n');
fprintf('=============================================\n\n');
end

function s = local_onoff(tf)
if tf
   s = '开';
else
   s = '关';
end
end

function tf = local_same(a, b)
% Whether a knob was changed. Numeric values compare on a relative tolerance
% rather than isequaln: a mass summed over 12 rows gives 17.000000000000004,
% and a bitwise comparison would report a phantom change and push a default run
% onto the minutes-long recompute path; non-numeric values must still match
if isnumeric(a) && isnumeric(b) && isequal(size(a), size(b))
   tf = all(abs(a(:) - b(:)) <= 1e-12 * max(1, abs(b(:))));
else
   tf = isequaln(a, b);
end
end

function [paths, vals] = local_flatten(s, pre, paths, vals)
% Flatten a nested struct into a depth-first {dotted path, value} list; a dotted
% path cannot be a struct field name, so the override table is an Nx2 cell
f = fieldnames(s);
for k = 1:numel(f)
   if isempty(pre)
      p = f{k};
   else
      p = [pre '.' f{k}];
   end
   v = s.(f{k});
   if isstruct(v) && numel(v) == 1
      [paths, vals] = local_flatten(v, p, paths, vals); %#ok<ASGLU>
   else
      paths{end + 1} = p; %#ok<AGROW>
      vals{end + 1} = v; %#ok<AGROW>
   end
end
end

function v = local_get(P, path)
% Read a P field by dotted path, with a readable error when it does not exist
parts = strsplit(path, '.');
v = P;
for jj = 1:numel(parts)
   assert(isstruct(v) && isfield(v, parts{jj}), ...
      'uacs_apply_config:BadKnob', '参数里没有 %s 这个字段', strjoin(parts(1:jj), '.'));
   v = v.(parts{jj});
end
end
