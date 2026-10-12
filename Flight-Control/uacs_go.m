% UACS_GO  唯一的人机交互入口。改参数、开模型都从这一个文件走。
%
% 用法，三种任选：
%   1. 在本文件里改第 1 节的数字，然后按 F5（运行）。
%   2. 直接双击 model\uacs_flight_bench.slx（飞行台架）或 model\uacs_aero_test.slx
%      （气动测试）。模型打开时会自动运行本文件，不需要任何前置命令。
%   3. 命令行敲 uacs_go。
%
% 本文件只有旋钮与时间。展平、比对、配平、线性化、增益、缓存这些机械动作全部在
% params/uacs_apply_config.m 里，末尾那一段只负责把旋钮打包交过去。
%
% 三条规矩：
%   - 本文件不新增参数，只覆盖 params/uacs_twin_params.m 里已有的字段。写错字段名会
%     当场报错，不会静默多出一个没人读的参数。
%   - 只填"原值"（原参数文件里标 ESTIMATE 的那些）。派生量（标 SIZED 的）由脚本算出，
%     在本文件里改它们没有意义：会被覆盖，或者被自检断言拦下。
%   - 只要有任何一项与冻结基准不同，启动时就会打出 PARAMETER OVERRIDES ACTIVE 横幅。
%     看到横幅就说明这一轮的结果不能引用 rev 11 的结论。

%% ==================== 第 1 节：参数 ====================

% ---- 1.1 质量与重心 ----------------------------------------------------
% 每行 {名称, 质量 kg, x m, y m, z m}。机体坐标：x 向前、y 向右、z 向下，
% 原点是机翼气动中心。总重、重心、三个惯量全部由这张表算出，不要另外填 mass.m。
% 12 kg 载荷就在 payload 那一行，改它就能做装载变化试验。
uacs_mass = {
   'wing',             1.10,  -0.05,  0.00,  0.00
   'fuselage_boom',    0.55,   0.00,  0.00,  0.00
   'payload',         12.00,  -0.22,  0.00,  0.02
   'battery',          2.10,  -0.15,  0.00,  0.03
   'tail',             0.20,   1.95,  0.00,  0.08
   'motor_esc_left',   0.17,  -0.05,  0.90, -0.10
   'motor_esc_right',  0.17,  -0.05, -0.90, -0.10
   'prop_left',        0.03,  -0.05,  0.90, -0.10
   'prop_right',       0.03,  -0.05, -0.90, -0.10
   'landing_gear',     0.20,  -0.10,  0.00,  0.18
   'servos_links',     0.20,  -0.10,  0.00,  0.05
   'margin',           0.25,   0.00,  0.00,  0.00
   };

% ---- 1.2 机翼与尾翼几何 ------------------------------------------------
% 机翼只有两个自由量：翼展与展弦比。面积 S = b^2/AR、平均气动弦 cbar = S/b、
% 尾容量 Vh/Vv 全部由它们算出。翼展上限 3.5 m 由团队要求钉死，自检会拦。
uacs_wing.span_m       = 3.40;    % m，翼展（上限 3.50）
uacs_wing.aspect_ratio = 5.00;    % 展弦比，机翼形状的自由参数
uacs_wing.taper        = 0.75;    % 梢根比
uacs_wing.dihedral_deg = 3.0;     % 上反角
uacs_wing.incidence_deg= 2.0;     % 机翼安装角

uacs_tail.h_area_ratio = 0.16;    % 平尾面积 / 机翼面积
uacs_tail.h_arm_m      = 1.95;    % 机翼气动中心到平尾气动中心的力臂
uacs_tail.h_incidence_deg = -1.0; % 平尾安装角
uacs_tail.v_area_m2    = 0.600;   % 垂尾面积
uacs_tail.v_arm_m      = 1.95;    % 垂尾力臂

% ---- 1.3 气动导数（只读抄录，改这里不生效） ----------------------------
% 这一节不参与覆盖。在原参数文件里它们的原值与派生量是逐条交错的（CL_de 紧跟
% CL_alpha、k_induced 紧跟 e_oswald，共八处），做成旋钮要在那八处各插一行覆盖调用，
% 这一版没做。要改气动导数就直接改 params/uacs_twin_params.m 第 222 行起。
% 下面这张表只是当前值的抄录，方便核对。
uacs_aero.CL0           = 0.220;    % 零迎角升力系数
uacs_aero.CL_alpha      = 4.65;     % 1/rad，升力线斜率
uacs_aero.CL_max        = 1.33;     % 失速升力系数
uacs_aero.alpha_stall_deg = 14.0;   % 失速迎角
uacs_aero.CD0           = 0.030;    % 零升阻力系数
uacs_aero.e_oswald      = 0.780;    % 奥斯瓦尔德效率，决定诱导阻力
uacs_aero.CD_alpha      = 0.100;    % 迎角平方项阻力
uacs_aero.CD_de         = 0.010;    % 升降舵配平阻力
uacs_aero.Cm0_wing      = -0.012;   % 翼身零升俯仰力矩
uacs_aero.Cm_alpha_wing_body = 0.020;  % 翼身俯仰刚度
uacs_aero.Cm_q          = -12.236;  % 俯仰阻尼
uacs_aero.Cm_alphadot   = -2.500;   % 下洗滞后导数
uacs_aero.tau_elevator  = 0.650;    % 升降舵效率因子
uacs_aero.tau_rudder    = 0.500;    % 方向舵效率因子
uacs_aero.CY_beta       = -0.750;   % 侧力导数
uacs_aero.Cl_beta       = -0.090;   % 上反效应
uacs_aero.Cl_p          = -0.388;   % 滚转阻尼
uacs_aero.Cl_r          = 0.050;    % 滚转-偏航交叉
uacs_aero.Cl_da         = 0.0412;   % 副翼操纵效能
uacs_aero.Cl_dr         = 0.010;    % 方向舵滚转交叉
uacs_aero.Cn_p          = -0.030;   % 偏航-滚转交叉
uacs_aero.Cn_da         = -0.006;   % 副翼不利偏航

% ---- 1.4 执行机构 ------------------------------------------------------
% tau_s 是一阶舵机时间常数，rate_deg_s 是最大角速率。
uacs_act.aileron_max_deg  = 25.0;   uacs_act.aileron_rate_deg_s  = 120.0;  uacs_act.aileron_tau_s  = 0.045;
uacs_act.elevator_max_deg = 25.0;   uacs_act.elevator_rate_deg_s = 100.0;  uacs_act.elevator_tau_s = 0.045;
uacs_act.rudder_max_deg   = 25.0;   uacs_act.rudder_rate_deg_s   = 100.0;  uacs_act.rudder_tau_s   = 0.045;

% ---- 1.5 推进 ----------------------------------------------------------
% 油门是峰值轴功率的比例，不是转速。螺旋桨直径与静拉力上限都由这里算出。
uacs_prop.peak_power_per_motor_W = 1500.0;  % 每台电机峰值轴功率
uacs_prop.diameter_m      = 0.4064;  % 螺旋桨直径
uacs_prop.figure_of_merit = 0.550;   % 桨的品质因数
uacs_prop.esc_tau_s       = 0.060;   % 电调一阶滞后
uacs_prop.y_motor_m       = 0.900;   % 电机横向站位（差动推力力臂）
uacs_prop.z_motor_m       = -0.100;  % 电机垂向站位（推力线偏移）
uacs_prop.differential_limit = 0.600;% 差动油门结构上限

% ---- 1.6 控制目标（PID） ----------------------------------------------
% 这七个带宽是设计目标。注意内环速率环的实际目标不取 rate_*，而是由
% control/design_pid.m 的 local_inner_target 从机体结构模态另算（当前 10.0 rad/s）。
uacs_ctrl.bw_rate_p_rad_s     = 12.0;   % 滚转速率环
uacs_ctrl.bw_rate_q_rad_s     =  8.0;   % 俯仰速率环
uacs_ctrl.bw_rate_r_rad_s     =  5.0;   % 偏航速率环
uacs_ctrl.bw_att_phi_rad_s    =  3.0;   % 滚转姿态环
uacs_ctrl.bw_att_theta_rad_s  =  2.5;   % 俯仰姿态环
uacs_ctrl.bw_speed_rad_s      =  0.8;   % 空速环
uacs_ctrl.bw_height_rad_s     =  0.4;   % 高度环
uacs_ctrl.rate_filter_tau_s   = 0.010;  % D 项一阶滤波时间常数
uacs_ctrl.anti_windup_Kb      = 1.0;    % 反算抗积分饱和增益
uacs_ctrl.int_state_margin    = 8.0;    % 积分状态界 = margin * 误差界 / 带宽
uacs_ctrl.rate_cmd_frac       = 0.70;   % 速率指令限幅取满舵稳态速率的比例
uacs_ctrl.int_limit_phi_deg   = 15.0;   % 滚转环允许吸收的持续误差
uacs_ctrl.int_limit_theta_deg = 10.0;   % 俯仰环
uacs_ctrl.int_limit_V_mps     =  5.0;   % 空速环
uacs_ctrl.int_limit_h_m       = 20.0;   % 高度环

% ---- 1.7 LQR 权重 ------------------------------------------------------
% Bryson 型。q 是状态惩罚（越大越紧），r 是控制惩罚（越大越省舵）。
uacs_lqr.long_q = [1/2.0^2, 1/deg2rad(5)^2, 1/deg2rad(15)^2, 1/deg2rad(10)^2];  % [dV dalpha dq dtheta]
uacs_lqr.long_r = [1/deg2rad(20)^2, 1/0.30^2];                                  % [dde dT]
uacs_lqr.lat_q  = [1/deg2rad(5)^2, 1/deg2rad(60)^2, 1/deg2rad(30)^2, 1/deg2rad(20)^2]; % [dbeta dp dr dphi]
uacs_lqr.lat_r  = [1/deg2rad(20)^2, 1/deg2rad(20)^2];                           % [dda ddr]

% ---- 1.8 包线、配平点、大气 --------------------------------------------
uacs_limits.bank_max_deg       = 45.0;   % 最大滚转角
uacs_limits.pitch_max_deg      = 30.0;   % 最大俯仰角
uacs_limits.V_max_mps          = 30.0;   % 最大空速（必须可达，自检会拦）
uacs_limits.climb_rate_max_mps =  6.0;   % 最大爬升率
uacs_limits.h_max_m            = 400.0;  % 最大高度
uacs_limits.nz_max             =  3.0;   % 最大过载
uacs_limits.nz_min             = -1.5;   % 最小过载
uacs_cruise.V_mps              = 25.0;   % 设计配平速度
uacs_cruise.h_m                = 150.0;  % 设计配平高度
uacs_atmo.model                = 'const';% 'const' 常数海平面密度，或 'isa' 随高度

% ---- 1.9 仿真时长与节拍 ------------------------------------------------
% 这两个数就是 Simulink 工具栏 StopTime 框里的内容：
%   uacs_sim.t_end_s      -> 变量 sim_t_end，      受验模型与预置场景作业用
%   uacs_sim.bench_run_s  -> 变量 sim_bench_t_end，交互台架 uacs_flight_bench 用
% 想临时改一次，也可以直接在模型里把 StopTime 那个框改成数字，不必回来改这里。
uacs_sim.t_end_s     = 60.0;     % 仿真时长 s，预置场景作业
uacs_sim.bench_run_s = 20.0;     % 仿真时长 s，交互台架。0.0025 s 步长下每秒约 7 秒真实时间
uacs_sim.Ts_ctrl_s   = 0.0025;   % 内环采样，也是定步长求解器步长
uacs_sim.Ts_outer_s  = 0.0100;   % 姿态/空速/高度环采样
uacs_sim.Ts_alloc_s  = 0.0100;   % 分配器采样
uacs_sim.Ts_sensor_s = 0.0025;   % 传感器采样

% ---- 1.10 扰动：常值风与阵风 ------------------------------------------
% 画布上还有两个东西可以边跑边加：一个"侧风"滑杆，只加到东向分量上。
uacs_dist.wind_ned_mps  = [0; 0; 0];  % 常值风，机体坐标 NED 三分量 [北 东 地] m/s
uacs_dist.gust.on       = false;      % 阵风总开关
uacs_dist.gust.amp_mps  = 3.0;        % 阵风幅度 m/s
uacs_dist.gust.freq_hz  = 0.5;        % 阵风频率 Hz（模型里按 2*pi*f 用）
uacs_dist.gust.dir_ned  = [1; 0; 0];  % 阵风方向，NED 三分量，会被归一化成单位向量使用

% ---- 1.11 游戏手柄（辅助通道） ----------------------------------------
% 手柄块 JoystickID 的可选值是 'None' 与 'Joystick1' 到 'Joystick16'。默认 'None'
% 表示不碰硬件、输出恒零，所以没有手柄时台架照常跑。要用真手柄：改成 'Joystick1'，
% 重跑 run/build_bench_models.m（模型里那个块的参数是构建时写死的），再在台架的
% 操纵台上把"操纵源"拨到"手柄"。
% pitch_sign 用来修俯仰方向。Pilot Joystick 的俯仰正负号与 FlightGear 的摇杆接口
% 相反，与本台架的约定未必一致，拉杆方向反了就把它改成 -1。
uacs_joystick.id         = 'None';
uacs_joystick.pitch_sign = 1.0;

%% ==================== 机械部分（不要动） ====================

% 上面每一组旋钮在这里打包，交给 params/uacs_apply_config.m。这个文件里从此不再有
% 展平、比对、配平、线性化、增益与缓存。uacs_aero 也打包进去，但它在那边不参与覆盖，
% 只用于核对，原因见第 1.3 节。
uacs = struct();
uacs.mass = uacs_mass;
uacs.wing = uacs_wing;
uacs.tail = uacs_tail;
uacs.aero = uacs_aero;
uacs.act = uacs_act;
uacs.prop = uacs_prop;
uacs.ctrl = uacs_ctrl;
uacs.lqr = uacs_lqr;
uacs.limits = uacs_limits;
uacs.cruise = uacs_cruise;
uacs.atmo = uacs_atmo;
uacs.sim = uacs_sim;
uacs.dist = uacs_dist;
uacs.joystick = uacs_joystick;
uacs_apply_config(uacs);

clear uacs uacs_mass uacs_wing uacs_tail uacs_aero uacs_act uacs_prop uacs_ctrl ...
   uacs_lqr uacs_limits uacs_cruise uacs_atmo uacs_sim uacs_dist uacs_joystick
