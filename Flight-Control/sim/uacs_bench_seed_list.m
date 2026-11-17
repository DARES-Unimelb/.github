function pairs = uacs_bench_seed_list()
% UACS_BENCH_SEED_LIST Map from bench block paths to base workspace variable names
%
%   pairs   n x 2 cell array: block path relative to the model root, variable name
%
% This table is the single authority for the initial values of the bench inputs:
% the build writes them into the block literal values and uacs_bench_seed pushes
% them again when the model opens
%
% The values have to reach the blocks as literals; a Dashboard control bound to a
% base workspace variable does not reach a running simulation

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
