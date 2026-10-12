function pairs = uacs_bench_seed_list()
% UACS_BENCH_SEED_LIST 台架操纵量与 uacs_go.m 变量的一一对应表。
%
% 左边是模型里的块路径（相对模型根），右边是基础工作区变量名。
%
% 这张表是操纵量初值的唯一权威。构建时 sim_build_bench 按它把变量写进块的字面值参数，
% 打开模型时 uacs_bench_seed 按它再推一次。两处用同一张表，所以不会出现"构建时一个
% 值、打开时另一个值"。
%
% 为什么要落字面量：Dashboard 控件绑基础工作区变量是假通路，运行时改变量仿真不理。
% 实测见 verify/bench_live_check.m。

pairs = {
   '驾驶员/k_roll',         'sim_stick_roll'
   '驾驶员/k_pitch',        'sim_stick_pitch'
   '驾驶员/k_yaw',          'sim_stick_yaw'
   '驾驶员/k_thr',          'sim_stick_thr'
   '驾驶员/js_on',          'sim_js_on'
   '制导与模式/mode_const', 'sim_flight_mode'
   '控制器选择/k_sel',      'sim_ctrl_sel'
   '大气与风扰动/wind_e',   'sim_dist_wind_e'
   };
end
