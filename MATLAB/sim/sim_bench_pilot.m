function info = sim_bench_pilot(mdl, opts)
% SIM_BENCH_PILOT 台架的驾驶员子系统：操纵输入到参考指令。
%
%   驾驶员/   出 1 路 cmd 5x1 = [phi_c theta_c r_c V_c h_c]，出 2 路 state 1x1 是操纵源档位
%
% 两路操纵输入，用 Switch 选一，画布上的"操纵源"拨动开关绑的就是选择变量
%   虚拟操纵杆  四个 Constant 的 Value 分别指向基础工作区变量 sim_stick_roll /
%               sim_stick_pitch / sim_stick_yaw / sim_stick_thr。画布上的三个旋钮与
%               油门滑杆绑的就是这四个变量，拖旋钮等于拖杆。
%   游戏手柄    Aerospace Blockset 的 Pilot Joystick，四路输出依次是 roll / pitch /
%               yaw [-1,1] 与 throttle [0,1]，与虚拟杆同一量纲，直接进 Mux。
%
% 手柄通道串了一个符号增益 sim_js_pitch_sign。Pilot Joystick 的俯仰正负号与
% FlightGear 的摇杆接口相反，与本台架的约定未必一致，所以留成参数，默认 +1，
% 觉得拉杆方向反了就在 uacs_go.m 里把它改成 -1。
%
% Pilot Joystick 的采样时间必须显式设。它的默认值是 1/30 s，与定步长 0.0025 s 不成
% 整数倍，不设的话 update 报 Simulink:SampleTime:BlockTsNotAMultipleOfFixedStep。
% 这里设成 Ts_outer。
%
% opts.joystick_id  手柄块的 JoystickID，默认 'None'，此时不碰硬件、输出恒零，所以
%                   没有手柄时模型照样能跑。要用真手柄就改成 'Joystick1'。

if nargin < 2
   opts = struct();
end
if ~isfield(opts, 'joystick_id') || isempty(opts.joystick_id)
   opts.joystick_id = 'None';
end

sub = [mdl '/驾驶员'];
sim_add_block(mdl, '驾驶员', 'simulink/Ports & Subsystems/Subsystem', [860 80 1010 220]);
sim_clear_model(sub);

% ---------------- 虚拟操纵杆 ----------------
sim_add_block(sub, 'k_roll', 'simulink/Sources/Constant', sim_pos(0, 0, 90, 30), ...
   'Value', 'sim_stick_roll');
sim_add_block(sub, 'k_pitch', 'simulink/Sources/Constant', sim_pos(0, 1, 90, 30), ...
   'Value', 'sim_stick_pitch');
sim_add_block(sub, 'k_yaw', 'simulink/Sources/Constant', sim_pos(0, 2, 90, 30), ...
   'Value', 'sim_stick_yaw');
sim_add_block(sub, 'k_thr', 'simulink/Sources/Constant', sim_pos(0, 3, 90, 30), ...
   'Value', 'sim_stick_thr');
sim_add_block(sub, 'stick_mux', 'simulink/Signal Routing/Mux', sim_pos(1, 1, 20, 110), ...
   'Inputs', '4');
sim_connect(sub, 'k_roll/1', 'stick_mux/1', 'k_pitch/1', 'stick_mux/2', ...
   'k_yaw/1', 'stick_mux/3', 'k_thr/1', 'stick_mux/4');

% ---------------- 游戏手柄 ----------------
js = [sub '/js'];
add_block('aerolibanimutils/Pilot Joystick', js, 'Position', sim_pos(0, 5, 110, 110), ...
   'JoystickID', opts.joystick_id, 'OutputConfiguration', 'FourAxis', ...
   'SampleTime', 'sim_ts_outer');
sim_add_block(sub, 'js_pitch_sign', 'simulink/Math Operations/Gain', sim_pos(1, 6, 40, 30), ...
   'Gain', 'sim_js_pitch_sign');
sim_add_block(sub, 'js_mux', 'simulink/Signal Routing/Mux', sim_pos(2, 5, 20, 110), ...
   'Inputs', '4');
sim_connect(sub, 'js/1', 'js_mux/1', 'js/2', 'js_pitch_sign/1', ...
   'js_pitch_sign/1', 'js_mux/2', 'js/3', 'js_mux/3', 'js/4', 'js_mux/4');

% ---------------- 操纵源选择与驾驶员模型 ----------------
sim_add_block(sub, 'js_on', 'simulink/Sources/Constant', sim_pos(3, 0, 80, 30), ...
   'Value', 'sim_js_on');
sim_add_block(sub, 'src_sw', 'simulink/Signal Routing/Switch', sim_pos(3, 1, 60, 60), ...
   'Criteria', 'u2 ~= 0', 'Threshold', 'sim_c_zero');
sim_add_block(sub, 'pilot', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(4, 1, 120, 60), 'FunctionName', 'sim_pilot_sfun');
sim_add_block(sub, 'cmd', 'simulink/Sinks/Out1', sim_pos(5, 1, 30, 14), 'Port', '1');
% 操纵源的档位单独送出去一路，顶层那块状态读数接的就是它。
sim_add_block(sub, 'state', 'simulink/Sinks/Out1', sim_pos(5, 5, 30, 14), 'Port', '2');
sim_connect(sub, 'js_mux/1', 'src_sw/1', 'js_on/1', 'src_sw/2', ...
   'stick_mux/1', 'src_sw/3', 'src_sw/1', 'pilot/1', 'pilot/1', 'cmd/1');
sim_connect(sub, 'js_on/1', 'state/1');
sim_name_line(sub, 'js_mux', '1 手柄：roll/pitch/yaw/throttle');
sim_name_line(sub, 'stick_mux', '0 虚拟操纵杆：画布上的旋钮与滑杆');
sim_name_line(sub, 'pilot', 'cmd 5x1 [phi_c theta_c r_c V_c h_c]');

info = struct('name', '驾驶员', 'path', sub, 'joystick_id', opts.joystick_id);
end
