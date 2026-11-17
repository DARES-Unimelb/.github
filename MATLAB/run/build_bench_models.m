function rep = build_bench_models()
% BUILD_BENCH_MODELS Build the two interactive models from uacs_go's parameters.
%
%   rep = build_bench_models()
%
%   model/uacs_flight_bench.slx   飞行台架：画布上的八个 Dashboard 控件（三个旋钮、
%                                 两个滑杆、三个开关）直接操控，十二块圆形仪表、一块
%                                 失速灯、三块状态读数、三条画布内示波器看结果。里面跑
%                                 的是受验的那套 PID 与 LQR，两者并联同跑，一个旋转开关
%                                 瞬时切换。
%   model/uacs_aero_test.slx      气动测试页：迎角/侧滑角/速度/高度/三个舵面，
%                                 系数与力、力矩读数来自同一个 plant/aero_forces.m
%
% 两个模型都带 PreLoadFcn 与 InitFcn，回调从 .slx 自己的位置往上两级推出工程根再 addpath，
% 所以**双击 .slx 就能用**，不需要先手工 addpath 或初始化；把整个目录搬到别的机器上也是
% 一样的。前提是模型留在 根/model/ 下面，回调依赖这个布局。
%
% 台架不参与 V1 到 V13 的验收，也不写任何交付证据文件。它读的是同一套参数与同一套控制
% 律函数，所以不会与受验模型漂开。
%
% 这个函数必须走 run_job 跑，不能在 MCP 的单次调用里跑：save_system 会去生成缩略图，
% 那条路经 Live Editor 桥，在 MCP 的求值通道里必失败。

this = mfilename('fullpath');
if isempty(this)
   this = which('build_bench_models');
end
root = fileparts(fileparts(this));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;

% 先把参数、增益和默认场景推到 base，模型里的变量才有值。
run(fullfile(root, 'uacs_go.m'));
P = evalin('base', 'P');
js_id = evalin('base', 'sim_js_id');

% 模型回调：先把工程根加到搜索路径上，再跑 uacs_go。
%
% 早先这里写的是 run('<绝对路径>\uacs_go.m')，把整个 MATLAB 目录搬到别处、或者换一台机器，
% 那条路就断了：run 只是临时切目录，uacs_go 里第一个函数调用 uacs_apply_config 就找不到，
% PreLoadFcn 报错退出，基础工作区是空的，接着按 Run 时 InitFcn 找不到 uacs_ensure，报
% 「Unrecognized function or variable 'uacs_ensure'」。用户实测撞到过一次。
%
% 现在从 .slx 自己的位置往上两级拿工程根，只依赖「模型在 根/model/ 下」这个布局。PreLoadFcn
% 与 InitFcn 都带上这一段，所以即使 PreLoadFcn 没跑成，按一次 Run 也能自己站起来。
boot = ['p=fileparts(fileparts(get_param(bdroot,''FileName'')));' ...
   'addpath(p,fullfile(p,''params''),fullfile(p,''plant''),fullfile(p,''control''),' ...
   'fullfile(p,''sim''),fullfile(p,''run''),fullfile(p,''verify''),''-begin'');'];
reload = [boot 'uacs_go'];
ensure = [boot 'uacs_ensure'];

fprintf('\n构建交互模型...\n');
bench = sim_build_bench(P, struct('name', 'uacs_flight_bench', ...
   'joystick_id', js_id, 'stop_time', 'sim_bench_t_end', ...
   'reload_fcn', reload, 'ensure_fcn', ensure));
aero = sim_build_aero_test(P, struct('reload_fcn', reload, 'ensure_fcn', ensure));
rep = struct('bench', bench, 'aero', aero, 'reload_fcn', reload, ...
   'joystick_id', js_id, 'bench_stop_s', evalin('base', 'sim_bench_t_end'));
fprintf('  %-20s top %3d total %3d  %s\n', bench.model, bench.census.top_count, ...
   bench.census.total_count, bench.file);
fprintf('  %-20s top %3d total %3d  %s\n', aero.model, aero.census.top_count, ...
   aero.census.total_count, aero.file);
fprintf('台架手柄 JoystickID = %s\n', js_id);
fprintf('构建完成。双击 model 目录下的 .slx 即可使用。\n\n');
end
