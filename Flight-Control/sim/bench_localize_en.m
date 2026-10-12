function info = bench_localize_en(varargin)
% BENCH_LOCALIZE_EN 台架模型的英文副本：把画布上所有中文换成英文，位置与连线一根不动。
%
%   info = bench_localize_en()
%   info = bench_localize_en('src', 'uacs_flight_bench', 'dst', 'uacs_flight_bench_en')
%
% 产出 model/uacs_flight_bench_en.slx。
%
% 为什么做成改名而不是重新生成：重新生成会把用户摆好的位置与连线全部覆盖。这个函数只动
% 三类字符串，块名、线名、注释文本，块的位置、尺寸、参数、连线关系一律不碰。
%
% 改名的次序是按路径深度从深到浅。先改子块再改父系统，否则改父系统名字之后子块的完整
% 路径就失效了。
%
% 三个回调也跟着改：PostLoadFcn 指向 uacs_bench_seed_en，那个函数用的是英文路径表；其余
% 两个只带模型名，换名字即可。
%
% 块参数里没有中文，这一点是查过的，所以不需要翻参数。

ip = inputParser;
ip.addParameter('src', 'uacs_flight_bench', @(x) ischar(x) || isstring(x));
ip.addParameter('dst', 'uacs_flight_bench_en', @(x) ischar(x) || isstring(x));
ip.parse(varargin{:});
src = char(ip.Results.src);
dst = char(ip.Results.dst);

root = fileparts(fileparts(mfilename('fullpath')));
src_file = fullfile(root, 'model', [src '.slx']);
dst_file = fullfile(root, 'model', [dst '.slx']);
% exist 对 .slx 返回 4，不是 2。这里以及下面的判断都按「非零即存在」写。
assert(exist(src_file, 'file') > 0, 'bench_localize_en:NoSrc', '找不到 %s', src_file);

if bdIsLoaded(dst)
   close_system(dst, 0);
end
if exist(dst_file, 'file') > 0
   delete(dst_file);
end
copyfile(src_file, dst_file);
load_system(dst_file);

blk_map = local_block_map();
line_map = local_line_map();
ann_map = local_ann_map();

% ---------------- 块名 ----------------
blks = find_system(dst, 'Type', 'Block');
depth = cellfun(@(b) sum(b == '/'), blks);
[~, ord] = sort(depth, 'descend');
n_blk = 0;
for k = 1:numel(ord)
   b = blks{ord(k)};
   nm = get_param(b, 'Name');
   if isKey(blk_map, nm)
      set_param(b, 'Name', blk_map(nm));
      n_blk = n_blk + 1;
   end
end

% ---------------- 线名 ----------------
n_line = 0;
allb = find_system(dst, 'FindAll', 'on', 'Type', 'block');
done_line = containers.Map('KeyType', 'char', 'ValueType', 'logical');
for k = 1:numel(allb)
   ph = get_param(allb(k), 'PortHandles');
   for fld = {'Outport', 'Inport'}
      hs = ph.(fld{1});
      for j = 1:numel(hs)
         try
            lh = get_param(hs(j), 'Line');
         catch
            continue
         end
         if lh <= 0
            continue
         end
         ln = get_param(lh, 'Name');
         if isempty(ln) || isKey(done_line, ln)
            continue
         end
         if isKey(line_map, ln)
            set_param(lh, 'Name', line_map(ln));
            done_line(ln) = true;
            n_line = n_line + 1;
         end
      end
   end
end

% ---------------- 注释 ----------------
ann = find_system(dst, 'FindAll', 'on', 'Type', 'annotation');
n_ann = 0;
for k = 1:numel(ann)
   t = get_param(ann(k), 'Text');
   first = strtrim(strtok(t, newline));
   if isKey(ann_map, first)
      set_param(ann(k), 'Text', ann_map(first));
      n_ann = n_ann + 1;
   end
end

% ---------------- 回调 ----------------
% 用相对定位，不写绝对路径。原来的 PreLoadFcn 是 run('C:\...\uacs_go.m')，把整个 MATLAB
% 目录搬到别处、或者换一台机器，那条路就断了，模型打不开也跑不起来。这里改成从 .slx 自己
% 的位置往上找两级拿到工程根，与模型放在 根/model/ 下面这个布局绑定。InitFcn 同样先补
% 路径再跑 uacs_ensure，这样即使 PreLoadFcn 没跑成，按一次 Run 也能自己站起来。
% PreLoadFcn 与 InitFcn 里 bdroot 与 get_param(bdroot,'FileName') 都是可用的，实测过。
boot = ['p=fileparts(fileparts(get_param(bdroot,''FileName'')));' ...
   'addpath(p,fullfile(p,''params''),fullfile(p,''plant''),fullfile(p,''control''),' ...
   'fullfile(p,''sim''),fullfile(p,''run''),fullfile(p,''verify''),''-begin'');'];
set_param(dst, 'PreLoadFcn', [boot 'uacs_go']);
set_param(dst, 'InitFcn', [boot 'uacs_ensure']);
set_param(dst, 'PostLoadFcn', sprintf('uacs_bench_seed_en(''%s'')', dst));
set_param(dst, 'StopFcn', sprintf('uacs_bench_record(''%s'')', dst));

% ---------------- 线名自带的端口属性 ----------------
n_sig = local_fix_signalnames(dst, line_map);

% ---------------- 开关档位标签 ----------------
n_state = local_fix_states(dst, local_state_map());
save_system(dst);

% ---------------- 绑定里缓存的块路径 ----------------
% 这一步必须放到关闭再重新打开之后。改名当步把 ParamSourceInfo 的路径改成新名，保存时
% Simulink 会按旧的 SID 记录把中文路径写回去，改了个空。重新打开再改才留得住，所以这里
% 走第二遍，并且逐块回读校验。
close_system(dst, 0);
load_system(dst_file);
[n_bind, n_bad] = local_fix_bindings(dst, blk_map);
% 端口上还留着一份编译期算出来的传播信号名，它是按旧线名缓存的，编译一次再存才刷新。
try
   set_param(dst, 'SimulationCommand', 'update');
catch me
   fprintf('   编译刷新传播信号名没成功：%s\n', me.message);
end
save_system(dst);

info = struct('src', src, 'dst', dst, 'file', dst_file, 'blocks', n_blk, ...
   'lines', n_line, 'annotations', n_ann, 'states', n_state, ...
   'signalnames', n_sig, 'bindings', n_bind, 'bindings_failed', n_bad);
fprintf('英文副本 %s：块名 %d、线名 %d、注释 %d、端口信号名 %d、开关档位 %d、绑定 %d\n', ...
   dst_file, n_blk, n_line, n_ann, n_sig, n_state, n_bind);
if n_bad > 0
   fprintf('!! 有 %d 个绑定回读对不上，要人工看\n', n_bad);
end
end

% ------------------------------------------------------------------------
function n = local_fix_signalnames(mdl, line_map)
% 有些端口上存着一份 SignalName，值与它那根线的名字相同，是当初连好线时留下的。线名换
% 了它不会跟着换，得单独改。Outport 块上有这个属性，其余块没有，取不到就跳过。
n = 0;
blks = find_system(mdl, 'FindAll', 'on', 'Type', 'Block');
for k = 1:numel(blks)
   b = blks(k);
   sn = '';
   try
      sn = get_param(b, 'SignalName');
   catch
      continue
   end
   if ~ischar(sn) || isempty(sn) || ~isKey(line_map, sn)
      continue
   end
   try
      set_param(b, 'SignalName', line_map(sn));
      n = n + 1;
   catch
   end
end
end

% ------------------------------------------------------------------------
function n = local_fix_states(mdl, state_map)
% 开关的档位标签存在块的 States 结构里，Value 是档位号，Label 是画在开关上的字。它在
% Values_NNN.mxarray 里，是画布上看得见的字，改块名碰不到。
n = 0;
blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(blks)
   b = blks{k};
   st = [];
   try
      st = get_param(b, 'States');
   catch
      continue
   end
   if ~isstruct(st) || isempty(st) || ~isfield(st, 'Label')
      continue
   end
   hit = false;
   for j = 1:numel(st)
      if isKey(state_map, st(j).Label)
         st(j).Label = state_map(st(j).Label);
         hit = true;
      end
   end
   if hit
      set_param(b, 'States', st);
      n = n + 1;
   end
end
end

% ------------------------------------------------------------------------
function [n, n_bad] = local_fix_bindings(mdl, blk_map)
% 每一块仪表与每一个控件的 Binding 里缓存着被绑块的完整路径，路径按改名之前的中文名存，
% 形如 模型/驾驶员/k_pitch。路径不换，仪表与控件显示的名字就还是中文，双击打开一次属性
% 对话框才刷新。控件的绑定类型是 ParamSourceInfo，仪表是 SignalSpecification，后者在
% get_param 里可能是对象也可能包在元胞里，两种都认。
%
% 仪表侧重建一个新对象而不是改旧对象：旧对象里带着一个按旧线名解析出来的 SignalName，
% 只改路径它不掉。
n = 0;
n_bad = 0;
blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(blks)
   b = blks{k};
   bd = [];
   try
      bd = get_param(b, 'Binding');
   catch
      continue
   end
   if isempty(bd)
      continue
   end
   was_cell = iscell(bd);
   if was_cell
      items = bd;
   else
      items = {bd};
   end
   p = '';
   try
      p = char(items{1}.BlockPath.getBlock(1));
   catch
      continue
   end
   np = local_en_path(p, blk_map);
   is_spec = isa(items{1}, 'Simulink.HMI.SignalSpecification');
   % 仪表侧无条件重建：它的对象里带着一个按旧线名解析出来的 SignalName，路径已经是英文
   % 也照样带着，只有重建才掉。控件侧只在路径确实不同时改，改了没差别的不动它。
   if strcmp(np, p) && ~is_spec
      continue
   end
   try
      if isa(items{1}, 'Simulink.HMI.SignalSpecification')
         opi = items{1}.OutputPortIndex;
         fpm = [];
         try
            fpm = items{1}.FrameProcessingMode;
         catch
         end
         fresh = Simulink.HMI.SignalSpecification;
         fresh.BlockPath = Simulink.BlockPath(np);
         fresh.OutputPortIndex = opi;
         if ~isempty(fpm)
            try
               fresh.FrameProcessingMode = fpm;
            catch
            end
         end
         items{1} = fresh;
      else
         items{1}.BlockPath = Simulink.BlockPath(np);
      end
      if was_cell
         set_param(b, 'Binding', items);
      else
         set_param(b, 'Binding', items{1});
      end
   catch
      n_bad = n_bad + 1;
      fprintf('   绑定改不动：%s\n', get_param(b, 'Name'));
      continue
   end
   back = get_param(b, 'Binding');
   if iscell(back)
      back = back{1};
   end
   if strcmp(char(back.BlockPath.getBlock(1)), np)
      n = n + 1;
   else
      n_bad = n_bad + 1;
      fprintf('   回读对不上：%s\n', get_param(b, 'Name'));
   end
end
end

% ------------------------------------------------------------------------
function p2 = local_en_path(p, blk_map)
% 把绑定里缓存的块路径逐段换成英文名。模型名那一段已经是英文，不动。
parts = strsplit(p, '/');
for i = 1:numel(parts)
   if isKey(blk_map, parts{i})
      parts{i} = blk_map(parts{i});
   end
end
p2 = strjoin(parts, '/');
end

% ------------------------------------------------------------------------
function m = local_state_map()
% 开关档位标签。画布上看得见的字，就这三块开关。PID 与 LQR 本来就是英文，不入表。
pairs = {
   '姿态',    'Attitude'
   '高度',    'Altitude'
   '虚拟杆',  'Virtual stick'
   '手柄',    'Gamepad'
   };
m = containers.Map(pairs(:, 1), pairs(:, 2));
end

% ------------------------------------------------------------------------
function m = local_block_map()
% 画布上 39 个含中文的块名。叶子名在本模型里互不重复，所以按叶子名映射就够。
% 子系统名带空格：画布上可读优先，路径里的空格在 Simulink 里合法。
pairs = {
   '驾驶员',            'Pilot'
   '制导与模式',         'Guidance and Mode'
   'PID控制器',         'PID Controller'
   'LQR控制器',         'LQR Controller'
   '控制器选择',         'Controller Select'
   '混合与分配',         'Mixing and Allocation'
   '六自由度对象',        '6-DOF Plant'
   '导航与测量',         'Navigation and Measurement'
   '大气与风扰动',        'Atmosphere and Wind'
   '俯仰杆',            'Pitch Stick'
   '滚转杆',            'Roll Stick'
   '方向舵',            'Rudder Pedal'
   '速度指令',           'Airspeed Command'
   '侧风 东向 mps',      'Crosswind East mps'
   '模式',              'Mode'
   '控制器',            'Controller'
   '操纵源',            'Stick Source'
   '空速表',            'Airspeed'
   '空速指令表',         'Airspeed Command Gauge'
   '高度表',            'Altitude'
   '迎角表',            'Angle of Attack'
   '滚转表',            'Roll'
   '滚转指令表',         'Roll Command'
   '俯仰表',            'Pitch'
   '俯仰指令表',         'Pitch Command'
   '偏航率表',           'Yaw Rate'
   '方向舵表 右舵正',     'Rudder Gauge Right Positive'
   '副翼表 原始偏度',     'Aileron Gauge Raw'
   '升降舵表 原始偏度',   'Elevator Gauge Raw'
   '失速灯',            'Stall Lamp'
   '模式读数',           'Mode Readout'
   '控制器读数',         'Controller Readout'
   '操纵源读数',         'Source Readout'
   '姿态跟踪',           'Attitude Tracking'
   '速率跟踪 pq',        'Rate Tracking p q'
   '速率跟踪 r 方向舵',   'Rate Tracking r Rudder'
   '三个舵面',           'Three Surfaces'
   '左右油门',           'Left and Right Throttle'
   '兜底示波器 三段',     'Fallback Scope Three Signals'
   };
m = containers.Map(pairs(:, 1), pairs(:, 2));
end

function m = local_line_map()
pairs = {
   '油门 右 0..1',                      'throttle right 0..1'
   'theta_c 俯仰指令 [rad]',             'theta_c pitch command [rad]'
   'r 实际偏航率 [rad/s]',               'r actual yaw rate [rad/s]'
   'r_c 偏航率指令 [rad/s]',             'r_c yaw rate command [rad/s]'
   'theta_c 有效：模式 0 取杆位，模式 1 取 pid_h 输出', 'theta_c effective: mode 0 from stick, mode 1 from pid_h'
   'wind_ned 3x1 [北 东 地] m/s',        'wind_ned 3x1 [north east down] m/s'
   'V 空速 [m/s]',                      'V airspeed [m/s]'
   'h 高度 [m]',                        'h altitude [m]'
   'M_cmd 3x1 力矩指令 [roll pitch yaw]', 'M_cmd 3x1 moment command [roll pitch yaw]'
   'da 副翼偏度 [deg]',                  'da aileron deflection [deg]'
   'de 升降舵偏度 [deg]',                'de elevator deflection [deg]'
   'dr 方向舵偏度 右舵正 [deg]',          'dr rudder deflection, right positive [deg]'
   '1 手柄：roll/pitch/yaw/throttle',    '1 joystick: roll pitch yaw throttle'
   '0 虚拟操纵杆：画布上的旋钮与滑杆',     '0 virtual stick: the knobs and sliders on the canvas'
   };
m = containers.Map(pairs(:, 1), pairs(:, 2));
end

function m = local_ann_map()
% 按注释的第一行匹配，不按序号：注释在模型里的存放次序不保证。
games = [ ...
   'Gamepad, optional and off by default' newline newline ...
   'The Pilot subsystem contains a Pilot Joystick block. Its JoystickID is None by default, ' ...
   'so it touches no hardware and outputs zero; the virtual stick on the canvas is the source ' ...
   'that is actually used.' newline newline ...
   'To fly with a real gamepad, set uacs_joystick.id in section 1.11 of uacs_go.m, then ' ...
   'reopen the model. The pitch sign of the joystick may need flipping; that is ' ...
   'uacs_joystick.pitch_sign in the same section.'];
wind = [ ...
   'What the crosswind slider means' newline newline ...
   'It injects a constant wind in the earth-fixed east direction. The wind does not rotate ' ...
   'with the aircraft, so as the heading changes, the direction it arrives from in body axes ' ...
   'changes with it. The slider sets magnitude and sign only; the direction is always east.' newline newline ...
   'To change the direction, or to use a three-component wind, edit uacs_dist.wind_ned_mps in ' ...
   'section 1.10 of uacs_go.m, which holds the three NED components. To add a gust, set ' ...
   'uacs_dist.gust.on to true in the same section; amplitude, frequency and direction live ' ...
   'there too. All three are wired into the Atmosphere and Wind subsystem, they are not decoration.'];
limits = [ ...
   'Why roll and pitch both have a limit' newline newline ...
   'The limit is the design envelope, not a fault. The roll command tops out at 45 deg and the ' ...
   'pitch command at 30 deg, set by uacs_limits.bank_max_deg and uacs_limits.pitch_max_deg in ' ...
   'section 1.8 of uacs_go.m. Stick position maps linearly onto that range, so full stick is the ' ...
   'envelope boundary.' newline newline ...
   'There is a second and earlier limit that does not come from the stick. The attitude loop ' ...
   'output passes through a rate-command limiter set to 70 percent of the steady rate a full ' ...
   'surface can hold. Drag the roll stick to the stop and what you see is not the aircraft ' ...
   'refusing to turn: p_c flattens at some value and the aircraft rolls steadily at that rate. ' ...
   'Before that limiter existed, full roll stick asked for 17.73 rad/s while the airframe could ' ...
   'hold 1.23; the aileron saturated, sideslip built to 9 deg, and the aircraft rolled inverted.' newline newline ...
   'To widen it, change the two envelope angles in section 1.8 of uacs_go.m, or ' ...
   'uacs_ctrl.rate_cmd_frac in section 1.6. Note that the V1 to V13 acceptance results were ' ...
   'obtained with the present envelope; widen it and those results no longer apply.'];
data = [ ...
   'Where the run data goes, and what the four logged channels are' newline newline ...
   'Every time you press Run the StopFcn hook saves that run to results/bench_last_run.mat and ' ...
   'writes results/bench_report.txt. Every row of that table carries the channel name, so there ' ...
   'is no need to guess indices.' newline newline ...
   'For the raw logs the channel order is:' newline ...
   'inst_log, 7 channels: airspeed m/s, altitude m, angle of attack deg, roll deg, pitch deg, ' ...
   'stall flag, yaw rate deg/s' newline ...
   'att_log, 4 channels: roll command, roll, pitch command, pitch, all rad' newline ...
   'rate_log, 6 channels: p command, p, q command, q, r command, r, all rad/s' newline ...
   'act_log, 5 channels: aileron, elevator, rudder, all rad; left throttle, right throttle, 0 to 1' newline newline ...
   'The raw rudder deflection is channel 3 of act_log. Rudder tracking is the r command against ' ...
   'r in rate_log, which is the canvas scope named Rate Tracking r Rudder.'];
good = [ ...
   'What to look at, and what counts as good' newline newline ...
   '1. Attitude Tracking. Drag the roll or pitch stick; the command trace moves first and the ' ...
   'response follows. Four traces: the first two are roll command and roll, the last two pitch ' ...
   'command and pitch. Tracking is good when the pair meets within one second and does not ' ...
   'swing back after you release the stick.' newline newline ...
   '2. Rate Tracking p q. Four traces: p and q, command against actual. Drag the roll stick to ' ...
   'the stop and p_c flattens at the limiter value while p cannot follow that fast. That flat ' ...
   'segment is the rate-command limiter at work; it is what pulled the aircraft back from rolling ' ...
   'inverted. Only the PID produces these commands. The LQR hands surface increments straight ' ...
   'out, with no rate command in between, so after switching to the LQR those two command traces ' ...
   'no longer describe the active controller.' newline newline ...
   '3. Rate Tracking r Rudder. Two traces: r command against r, that is rudder tracking.' newline newline ...
   '4. Three Surfaces and Left and Right Throttle. The three surfaces in one scope, the two ' ...
   'throttles in another. The throttles splitting apart means differential thrust is working.' newline newline ...
   '5. The twelve gauges. On the left airspeed, airspeed command, altitude, angle of attack, ' ...
   'roll, roll command, pitch, pitch command; on the right yaw rate, rudder, aileron, elevator. ' ...
   'Each command gauge sits immediately to the right of the gauge it should match, so tracking ' ...
   'quality is readable without the scopes. The rudder gauge is already flipped to right-positive: ' ...
   'turn the knob right and both the rudder gauge and the yaw rate gauge move right. The aileron ' ...
   'and elevator gauges show the raw aerodynamic deflection, whose sign does not necessarily ' ...
   'match the stick. The stall lamp lights when the angle of attack passes the stall angle.' newline newline ...
   '6. The three state readouts. Mode Readout 0 is attitude mode and 1 is altitude mode; ' ...
   'Controller Readout 0 is PID and 1 is LQR; Source Readout 0 is the virtual stick and 1 is the ' ...
   'gamepad. The toggle and rotary switches do not display their own position, so read the numbers.' newline newline ...
   '7. If the five canvas scopes draw nothing, double-click the Fallback Scope Three Signals. It ' ...
   'is configured not to open automatically.' newline newline ...
   '8. Holding full stick is a fault injection. The aircraft should converge, keep sideslip ' ...
   'bounded and not lose altitude out of control, rather than enter a spiral.'];
fly = [ ...
   'How to fly it' newline newline ...
   'Press Run, then drag the knobs and sliders on the control panel at the left. Pitch Stick, ' ...
   'Roll Stick and Rudder Pedal are knobs; Airspeed Command and Crosswind East m/s are sliders. ' ...
   'Dragging takes effect immediately, there is no need to stop the run.' newline newline ...
   'The Airspeed Command slider is not a throttle. It sets the commanded airspeed, V_c = V_min + ' ...
   'stick * (V_max - V_min), which with the present parameters is 15.47 to 30 m/s. The default ' ...
   'stick position 0.656 corresponds to the 25 m/s cruise. So pulling the slider to 0 does not ' ...
   'throttle back, it asks the aircraft to hold 15.47 m/s and the speed loop keeps feeding thrust ' ...
   'to hold it there; pushing it to 1 asks for 30 m/s. The Airspeed Command gauge sits right next ' ...
   'to the Airspeed gauge, so command and actual line up directly.' newline newline ...
   'With Mode set to attitude the pitch stick drives the theta command directly. Set Mode to ' ...
   'altitude and the pitch stick becomes a climb-rate command, with the height loop pulling the ' ...
   'altitude slowly to whatever you worked it up to.' newline newline ...
   'The Controller switch selects between PID and LQR. Both run in parallel, so switching is ' ...
   'instant and needs no recompile. The LQR channel has no height loop, so altitude mode does ' ...
   'nothing for it; use attitude mode on both sides when comparing.' newline newline ...
   'The run length is the StopTime box in the toolbar, sim_bench_t_end. To change it, edit ' ...
   'uacs_sim.bench_run_s in section 1.9 of uacs_go.m, run once and press Run again; or type a ' ...
   'number straight into that box.'];
pairs = {
   '游戏手柄（辅助，默认关）',        games
   '侧风滑杆是什么意思',             wind
   '为什么滚转和俯仰都有"限位"',      limits
   '跑完的数据在哪，四路记录的通道是什么', data
   '看什么，什么算好',               good
   '怎么飞',                       fly
   '仪表与示波器：仪表绑信号，状态读数与兜底示波器接实线', ...
      'Instruments and scopes: gauges bind to signals, the state readouts and the fallback scope are wired'
   '控制律与对象：九个子系统，主线一条', ...
      'Control law and plant: nine subsystems, one main path'
   '操纵台：拖动控件即改指令，仿真运行中也是活的', ...
      'Control panel: dragging a control changes a command, live while the run is going'
   };
m = containers.Map(pairs(:, 1), pairs(:, 2));
end
