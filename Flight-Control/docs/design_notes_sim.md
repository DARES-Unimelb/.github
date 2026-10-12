# UACS Simulink 测试平台：设计说明

本文说明 `MATLAB\model\` 下三个 `.slx` 是怎么生成的、块与块之间接什么、每个块参数从哪个设计变量来、以及为什么这么选。接口契约在 `docs/architecture.md`，本文只写实现与判据。

## 1. 交付物与生成方式

| 文件 | 生成函数 | 入口作业 |
|---|---|---|
| `model/uacs_plant.slx` | `sim/sim_build_plant.m` | `run/uacs_job_sim_build_models.m` 经 `run/build_uacs_models.m` |
| `model/uacs_manual_pid.slx` | `sim/sim_build_manual_pid.m` | 同上 |
| `model/uacs_lqr.slx` | `sim/sim_build_lqr.m` | 同上 |
| `results/sim_build_census.mat` | `build_uacs_models.m` | 同上 |
| `results/simulink_build_report.txt` | `build_uacs_models.m` | 同上 |

作业入口按"一个作业只验一件事"拆开，另有 `run/uacs_job_sim_all.m` 把五个阶段串起来跑并在日志里逐阶段报成功失败，阶段之间互不阻断，便于共享会话被掐断时定位：

| 作业 | 做什么 |
|---|---|
| `uacs_job_sim_build_models` | 只生成、保存、普查三个模型，不仿真 |
| `uacs_job_sim_manual_pid` | PID 级联的八条场景 |
| `uacs_job_sim_lqr` | LQR 的五条场景 |
| `uacs_job_sim_scenarios` | V6 汇总、V9 侧风、V10 步长无关性、V12 跳过说明 |
| `uacs_job_sim_verify` | 跑 `verify/v_sim_*` 的独立断言 |
| `uacs_job_sim_all` | 上面五个阶段依次跑 |

三个模型一律 `new_system` / `add_block` / `add_line` / `set_param` 生成，每个 builder 结尾 `save_system`。可重复性靠三条：

1. `sim_new_model` 先 `close_system(mdl,0)`、再删掉盘上的 `.slx`、再 `new_system`，所以不存在「上一轮的块还在」这种状态，也就不会撞 `A new block named ... cannot be added`。
2. 清理模型或子系统用 `sim_clear_model`：先把全部连线句柄**一次**收齐再**一次**传给 `delete_line`，然后删顶层块。逐条循环删连线从第二个元素起句柄已失效。
3. 模型里没有字面量设计数字。每个块参数都是 base 工作区里的变量名，由 `sim/sim_push_design_vars.m` 与 `sim/sim_push_scenario.m` 推入。唯一的例外是**拓扑参数**：Mux 的输入个数、Demux 的输出个数、Sum 的符号串，这些是连线结构不是设计量，改外形不会动它们。

## 2. 为什么用 Level-2 MATLAB S-Function，不用 MATLAB Function 块

2026-10-08 的实测：MATLAB Function 块 + struct 型 Parameter 作用域数据 + `sim`，会让 MATLAB 会话**永久卡死**，18 分钟无响应，且没有异常抛出，把整个共享会话堵死。同批探针里参数类型写错的那次是正常抛异常的，所以普通的块编译错误不会挂会话，只有这条路进得去出不来。

因此模型里的非平凡计算一律走 Level-2 MATLAB S-Function（`sim/sim_plant_sfun.m`、`sim_meas_sfun.m`、`sim_pilot_sfun.m`、`sim_alloc_sfun.m`）。它按普通 MATLAB 解释执行：

- 不受 MATLAB Function 块代码生成子集限制，不需要前端做尺寸与类型推断，不会弹模态框；
- 天然支持连续状态，同一套块在定步长 `ode4` 与变步长 `ode45` 下都能跑，V10 不需要第二套模型；
- 依赖的 `P`、`gains`、`trim` 直接从 base 工作区取，不经过块参数解析。

`build_uacs_models.m` 保留 `blk_kind` 开关，但只接受 `'sfun'`；传 `'mbfcn'` 会直接报错并说明原因，防止有人悄悄切回去。

MATLAB Function 块一旦要用，规矩是：先在**独立短作业**里只验证那一件事，通过了再进模型。

## 3. 模型接口

### 3.1 `uacs_plant.slx`

| 端口 | 宽度 | 含义 |
|---|---|---|
| `u_act` | 6 | `[da de dr dth_l dth_r dT]`，rad, rad, rad, 0..1, 0..1, N |
| `wind_ned` | 3 | 定常风，NED，m/s |
| `x_state` | 12 | `[u v w p q r phi theta psi pn pe pd]` |

内部只有 `plant_eom` 一个 S-Function，12 个连续状态，`Derivatives` 调 `sim/sim_plant_rhs.m`，`Outputs` 直接回状态。

### 3.2 `uacs_manual_pid.slx`

杆量与风都从 base 工作区经 `From Workspace` 块给，场景由 `run/run_manual_pid.m` 生成。

| 块 | 类型 | 作用 |
|---|---|---|
| `stick_src` | From Workspace `sim_stick_ts` | `[t roll pitch yaw throttle]` |
| `wind_src` | From Workspace `sim_wind_ts` | `[t wn we wd]` |
| `cmd_pilot` | S-Function `sim_pilot_sfun` | `control/pilot_to_command.m`，输出 `[phi_c theta_c r_c V_c h_c]` |
| `meas` | S-Function `sim_meas_sfun` | `plant/flow_state.m`，输出 `[V alpha beta qbar]` |
| `pid_p` `pid_q` `pid_r` | 子系统 | 三个角速率环，`P.sim.Ts_ctrl` |
| `pid_phi` `pid_theta` | 子系统 | 两个姿态外环，`P.sim.Ts_outer` |
| `pid_V` `pid_h` | 子系统 | 速度与高度最外层，`P.sim.Ts_outer` |
| `theta_switch` | Switch `~=0` | `sim_flight_mode` 为 0 用时杆量给的 `theta_c`，为 1 用高度环给的 `theta_c` |
| `theta_lim` | Saturation | 高度环输出限幅 `±P.limits.pitch_max_deg` |
| `G_gain` | Gain 矩阵 `sim_G_surf` | 舵面偏度合成期望气动力矩 `M_cmd` |
| `T_sum` | Sum | `T_cmd = sim_T_trim + dV` |
| `alloc` | S-Function `sim_alloc_sfun` | `control/allocate.m`，输出五路执行机构 |
| `plant` | 子系统 | 与 3.1 同内容，由同一个 builder 生成 |

输出：`x_state`(12)、`u_act`(5)、`cmd`(5)、`rate_cmd`(3)、`M_cmd`(3)、`T_cmd`(1)。

速率环输出的是**舵面增量**，与 `design_pid` 的律 `da = Kp*(p_c - p) + Kd*Df(p_c - p)` 一致；加上配平舵偏后经 `G_surf` 变成绝对力矩指令再进 `allocate`，所以两个控制器模型共用同一条执行机构通路，限幅与差动推力只在 `allocate` 里发生一次。

`design_pid` 的每个律都写在相对设计点的偏差坐标里，所以模型里一律补回配平量：速率环补 `sim_u_trim_surf`，速度环补 `sim_T_trim`，高度环补的是俯仰姿态而不是力矩。这一点不补就会在配平点上产生常值偏差，而积分项会把它吃掉，看上去"能飞"但把设计裕度吃掉了。

`theta_switch` 的模式选择对应 `pilot_to_command` 头部注释：姿态模式直接用杆量给的 `theta_c`，高度模式把俯仰通道交给高度环，而高度环的 `h_c` 又是同一根俯仰杆积分出来的爬升率，两者不冲突。

### 3.3 `uacs_lqr.slx`

| 块 | 作用 |
|---|---|
| `ref_src` | From Workspace `sim_lqr_ref_ts`：`[t dV_c dtheta_c dphi_c dbeta_c]`，全部是相对设计点的偏差 |
| `Tref_long` `Uff_long` `Tref_lat` `Uff_lat` | 矩阵 Gain，把指令偏差映到参考状态与前馈，见第 5 节 |
| `long_abs` / `lat_abs` | 从 `meas` 与 `x_split` 拼出 `[V alpha q theta]` 与 `[beta p r phi]` |
| `long_dev` / `lat_dev` | 减 `sim_lqr_xop_long` / `sim_lqr_xop_lat` |
| `long_int` / `lat_int` | Discrete-Time Integrator，积分增广，输入 `C_int*x_dev - r` |
| `K_long` / `K_lat` | 2x5 增益矩阵，全状态反馈 |
| `long_uff_sum` / `lat_uff_sum` | `u_dev = u_ff + K*([x_ref;0] - [x_dev; z])` |
| `G_gain` `T_sum` `alloc` `plant` | 与手动模型同一条混合与执行机构通路 |

输出：`x_state`(12)、`u_act`(5)、`M_cmd`(3)、`T_cmd`(1)、`u_dev`(4, `[dda dde ddr dT]`)、`ref`(4)、`xi`(2, `[z_V z_phi]`)。

参考状态不放进模型算，而是在 `run/run_lqr.m` 里由 `control/lqr_reference.m` 解出来，再把两张线性映射矩阵推入 base。理由见第 5 节。

### 3.4 采样时间

| 环节 | 采样时间 | 来源 |
|---|---|---|
| 对象 | 连续 | `ode4`，步长 `P.sim.Ts_ctrl` |
| 速率环、LQR 积分 | `P.sim.Ts_ctrl` | 契约第 6 节 |
| 姿态、速度、高度环、杆量整形、分配 | `P.sim.Ts_outer` / `P.sim.Ts_alloc` | 契约第 6 节 |

`P.sim.Ts_ctrl` = 0.0025 s，`P.sim.Ts_outer` = 0.010 s，后者是前者的整数倍，定步长下不会出现采样点错位。

## 4. 风怎么进模型

`plant/eom_6dof.m` 没有风参数，也不读 `P.atmo.wind_ned`，所以风在模型边界处理一次，在 `sim/sim_plant_rhs.m` 里：

设 W 为 NED 下定常风，R 为机体系到 NED 的转换矩阵。机体系动量方程写成空速 `v_air = v_ground - R'*W` 时形式不变，因为 `omega × (R'*W)` 项在两侧相消：

```
m*(d(v_air)/dt + omega × v_air) = m*(d(v)/dt + omega × v)      （W 定常）
```

所以气动评估用 `[u v w] - R'*W`，但位置导数必须用**地速**，`dx(10:12)` 在调用之后被覆盖成 `R'*x(1:3)`。

前提是 W 在 NED 下定常。`P.atmo.gust_rms` 的阵风模型本版未接，`sim_wind_ts` 给的是定常风，符合 V9 的 5 m/s 侧风场景。

## 5. LQR 的参考状态为什么是两张矩阵

`design_lqr` 的律写在偏差坐标里：

```
u_dev = u_ff + K*([x_ref; 0] - [x_dev; z])
z     = integral(C_int*x_dev - r)
u_abs(u_idx_full) = u_star_full(u_idx_full) + u_dev
```

`control/lqr_reference.m` 解 `A*x_ref + B*u_ff = 0` 得到参考状态与前馈。这个方程对指令 `(dV, dtheta, dphi, dbeta)` 是**线性**的，所以在单位指令上解四次就得到整张映射，模型里用两个矩阵 Gain 复现即可，不需要在线解方程：

```
x_ref_long = sim_lqr_Tref_long * [dV_c; dtheta_c]     (4x2)
u_ff_long  = sim_lqr_Uff_long  * [dV_c; dtheta_c]     (2x2)
x_ref_lat  = sim_lqr_Tref_lat  * [dbeta_c; dphi_c]    (4x2)
u_ff_lat   = sim_lqr_Uff_lat   * [dbeta_c; dphi_c]    (2x2)
```

`sim/sim_lqr_refmap.m` 建这两张矩阵，并在一个非单位指令 `(0.37, -0.21, -0.07, 0.13)` 上把映射结果与直接调用 `lqr_reference` 的结果比一次，差值记进 `results/simulink_build_report.txt` 的 `reference map linearity residual`。参考点不可达时 `lqr_reference` 的残差也会被记下来。

LQR 输出的偏差量 `[dda dde ddr dT]` 加到配平舵偏上得到绝对舵面，再经 `G_surf` 合成力矩指令交给 `allocate`，与手动模型完全同路。

## 6. 设计变量映射表

`sim/sim_push_design_vars.m` 是唯一推入点，命名规则与来源如下。

| base 变量 | 来源 | 用途 |
|---|---|---|
| `sim_ts_ctrl` `sim_ts_outer` `sim_ts_alloc` `sim_ts_sensor` | `P.sim.Ts_*` | 各块采样时间 |
| `sim_t_end` | `P.sim.t_end_s` | 停止时间（场景可覆盖） |
| `sim_c_zero` `sim_c_one` `sim_c_neg1` | 结构常数 | 避免模型里出现字面量 |
| `sim_pid_<loop>_Kp/Ki` | `gains_pid.<loop>.Kp/Ki` | 比例、积分增益 |
| `sim_pid_<loop>_Dnum/Dden` | `sim_pid_coeffs` 由 `Kd`、`Tf`、`Ts` 算 | 微分项的两个多项式系数 |
| `sim_pid_<loop>_Ts` | `P.sim.Ts_ctrl`（p,q,r）或 `Ts_outer`（phi,theta,V,h） | 采样时间 |
| `sim_pid_<loop>_Isat/Isat_neg` | `gains_pid.<loop>.int_limit`，速率环取舵面限幅 | 积分限幅 |
| `sim_pid_h_lim` `sim_pid_h_lim_neg` | `±P.limits.pitch_max_deg` | 高度环输出限幅 |
| `sim_u_trim` `sim_u_trim_surf` | `trim.u_star` | 配平舵偏 |
| `sim_T_trim` | 由 `trim.u_star(4:6)` 与 `prop_thrust_from_throttle` 算 | 配平总推力 |
| `sim_G_surf` | `qbar_cruise*S*diag([b cbar b])` 乘三个舵效导数 | 舵面到力矩 |
| `sim_lqr_K_long/K_lat` | `gains_lqr.long/lat.K` | 全状态反馈增益 |
| `sim_lqr_C_long/C_lat` | `gains_lqr.long/lat.C_int` | 积分增广输出矩阵 |
| `sim_lqr_xop_long/lat` | `gains_lqr.long/lat.x_op` | 偏差坐标原点 |
| `sim_lqr_ustar` | `gains_lqr.long.u_star_full` | 配平输入 |
| `sim_lqr_Tref_*` `sim_lqr_Uff_*` | `sim_lqr_refmap` | 参考与前馈映射 |
| `sim_lqr_zlim_*` | `±P.ctrl.int_limit.V` / `.phi` | 积分状态限幅 |
| `sim_x0` `sim_stick_ts` `sim_wind_ts` `sim_lqr_ref_ts` `sim_flight_mode` | 场景 | 初始状态与输入 |

`P` 与短别名由 `init_uacs_twin` 推入，每个作业开头重新调用一次，不缓存旧值。

## 7. PID 块实现

每个 PID 是一个子系统，内部四个块：

```
e -> Kp(Gain)                -> sum
e -> Ki(Discrete-Time Integrator) -> sum     限幅 ±Isat
e -> Kd(Discrete Transfer Fcn)    -> sum     微分项
sum -> u
```

微分项按 Tustin 离散：`Kd*s/(Tf*s+1)` 代入 `s = (2/Ts)*(1-z^-1)/(1+z^-1)` 得

```
D(z) = [2*Kd/Ts, -2*Kd/Ts] / [1 + 2*Tf/Ts, -1 + 2*Tf/Ts]
```

两个向量在 `sim/sim_pid_coeffs.m` 里算好并按分母首项归一化，模型只读变量名。速率环的 `Kd = 0`，此时分子推成标量 0，块是纯零而不是两抽头相消。

积分抗饱和用 `Discrete-Time Integrator` 的输出限幅实现。`design_pid` 返回的 `gains.<loop>.Kb` 是反算抗饱和增益，离散积分器块没有反算模式，所以模型走限幅这条路；两者的差别只在饱和期间积分器回退的速度，闭环在非饱和区完全一致。

## 8. V10 做法

同一段杆量指令（俯仰双 t 加一次油门小阶跃，窗口 8 s，见 `run/run_scenarios.m` 的 `local_v10`），先定步长 `ode4` 走 `P.sim.Ts_ctrl`，再变步长 `ode45` 走 `RelTol = P.sim.rel_tol`、`AbsTol = P.sim.abs_tol`。

两者时间网格不同，所以先插值到公共轴再比。公共轴取定步长网格，变步长轨迹用 **pchip** 插值到该轴上：pchip 保形，单调段不会像普通三次样条那样凭空造出过冲。报告同时给出线性插值的结果作为敏感性，但判据用 pchip。

差值定义与判据：

- 逐状态取公共轴上 `|x_ode4 - x_ode45|` 的最大值；
- 主判据分母是 `ode4` 轨迹自身的峰峰值，`rel_ptp = max|Δ| / ptp`，要求 12 个状态全部 < 1%；
- 若某状态峰峰值小于其参考尺度的 `1e-4`（例如对称机动下不动的 `v`、`psi`），分母改用 `sim_state_scale.m` 给的参考尺度，并在报告里列出是哪些状态；
- 参考尺度：速度取巡航 25 m/s，角速率取 1 rad/s，角度取 1 rad，位置取巡航高度。

## 9. V11 做法

`build_uacs_models.m` 把每个模型的顶层块数、总块数、顶层块名清单、连线数与出入口数写进 `results/sim_build_census.mat`，并抄进 `results/simulink_build_report.txt`。

`verify/v_sim_v11_models.m` 独立复核：先 `close_system(mdl,0)` 丢掉内存里的副本，再 `load_system` 从盘上读回，用 `sim_census` 重新数一遍，与记录逐项比对，不一致直接 `assert` 失败。这条要防的是「模型只在内存里、没落盘」：那种情况下 `sim` 照样跑通而输出恒为零，事后从磁盘读回只会拿到残缺版本。

## 10. 已知限制与偏离

1. **对象在控制器模型里是子系统副本，不是 Model 块引用。** 架构第 6 节写 `uacs_plant.slx` 被引用。模型引用要求被引用模型与父模型求解器类型一致，而 V10 要在同一个控制器模型上跑定步长 `ode4` 与变步长 `ode45`，引用方式下每次都要改存被引用模型的求解器配置。两个副本由同一个 `sim_add_plant_subsystem` 生成，不可能漂开，V11 也把两份都对照同一份块普查。`uacs_plant.slx` 仍作为独立模型交付并单独验收。
2. **阵风未接**。`P.atmo.gust_rms` 的一阶成形阵风没有实现，V9 用的是定常侧风。
3. **伺服与 ESC 动态未进模型**。`P.act.*.tau_s`、`P.prop.esc.tau_s` 在 `design_pid` 里作为设计用滞后进了回路整形，但对象模型是 12 状态纯刚体，没有额外的舵机与电机状态。这会让闭环仿真比设计略微乐观，方向已知。
4. **`dT` 通道在手动模型里恒零**。`allocate` 用两路油门实现推力，第 6 路输入按契约留零。
5. **模型不覆盖执行机构速率限幅**。`P.act.*.rate_deg_s` 未建模，`allocate` 只做位置限幅。
6. **高度环与俯仰杆共用一根通道**，由 `sim_flight_mode` 选择，模式切换是阶跃而不是无扰切换。

## 11. 不依赖 MATLAB 的静态校验

模型代码最容易犯的错是块名重复、连线终点写错、块参数引用了没有推入 base 的变量，这三类错误只有在 `add_block` / `add_line` / 编译期才报，而每次试错都要占用共享 MATLAB 会话。所以在生成之前先做机器检查，检查项与判据：

| 检查 | 判据 |
|---|---|
| `function` 与 `end` 配对 | 每个文件块嵌套深度回到 0 |
| 括号与方括号配对 | 先合并 `...` 续行再逐逻辑行计数相等 |
| 块参数可解析 | builder 里出现的每个 `sim_*` 名字都在 `sim_push_design_vars` 或 `sim_push_scenario` 里有 `assignin` 或推入语句 |
| 连线终点存在 | 每条 `add_line` 的两端块名都在同一个 builder 里被 `sim_add_block` 建过 |
| 块名唯一 | 同一 builder 内不出现重名，重名正是 `A new block named ... cannot be added` 的成因 |

89 个 `.m` 文件全套通过；仅有的两条报警来自别的模块里正则字符串内的括号，属检查器误报。

检查器本体是 `sim/sim_static_check.py`，用 `C:\Python314\python.exe MATLAB\sim\sim_static_check.py` 跑，退出码 0 表示干净。放在 `sim/` 下是因为它属于建模工具链而不属于任何人独占的验收目录，MATLAB 不会去加载 `.py`，不影响路径搜索。

运行期另有两道兜底：`sim_lib_report` 在建模前解析全部 14 个库块路径，缺哪个就指名报出来；`sim_probe_plant` 在仿真前真调一次 `eom_6dof`，抛错就把对象切成占位实现并在报告里写明失败原因。

## 12. 实测结果

见 `results/simulink_build_report.txt`、`results/sim_scenarios_report.txt` 与 `docs/acceptance_report`（由 T4 汇总）。本文只保留方法，实测数值不在此重复。
