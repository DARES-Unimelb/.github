# UACS 控制测试平台：测试架构

本文件规定测试怎么分层、每层交什么、每条判据怎么执行、失败时看哪里。
判据的数值口径来自 `docs/architecture.md` 第 7 节；本文件不改判据，只把判据变成可执行的测法。
实测值与结论记录在 `docs/acceptance_report.md`。

## 1. 分层结构

五层串行，上一层不通过不进下一层。每层的接口签名由 `docs/architecture.md` 第 5 节冻结。

| 层 | 职责 | 输入 | 产物 | 权威文件 | 失败时的处置 |
|---|---|---|---|---|---|
| L0 参数 | 唯一参数源与自检 | 无 | 结构体 `P` 与短别名 `rho, g, m, b, S, cbar, AR` | `params/uacs_twin_params.m`、`params/init_uacs_twin.m` | 修参数或修断言，不允许绕过自检 |
| L1 配平与线性化 | 求巡航平衡点与数值雅可比 | `P` | `results/plant_baseline.mat`、`results/plant_trim_report.txt`、`results/plant_modes_report.txt` | `plant/trim_cruise.m`、`plant/linearize_at.m` | 先看残差分量与 `exitflag`，再动气动系数 |
| L2 控制律 | 增益、参考点、控制分配 | `P`、`lin` | `results/gains_baseline.mat`、`results/control_lqr_report.txt`、`results/control_pid_report.txt`、`results/control_allocation_report.txt` | `control/design_lqr.m`、`control/design_pid.m`、`control/allocate.m` | 查权重是否被改、参考点是否满足 `A*x_ref + B*u_ff = 0` |
| L3 闭环仿真 | 时域证据与场景 | `P`、`gains`、模型 | `results/sim_*.mat`、场景日志 | `run/`、`model/` | 查求解器设置、离散化、饱和与限幅链路 |
| L4 验收 | 逐条判据的可执行断言 | 上述全部产物 | `docs/acceptance_report.md` 中的实测值与结论 | `verify/` | 判据不过就记录失败与诊断，不改判据 |

层间的数值流动只有两个方向：L0 的 `P` 向下，L1 的 `lin` 与 L2 的 `gains` 向下。任何一层不得自行重算上一层的量。

## 2. 每层的接口与产物

### 2.1 L0 参数层

- 入口：`init_uacs_twin` 把 `P` 与七个短别名推入 base 工作区。
- 自检：`init_uacs_twin` 内部强制下列关系，任一条不成立即 V1 失败。
  - 质量预算：`sum(cell2mat(P.mass.items(:,2)))` 等于 `P.mass.m`，容差 1e-9。
  - 翼展硬上限：`b` 不超过 `geom.span_limit_m`，当前 3.40 m 对上限 3.50 m。
  - 几何自洽：面积由翼展与展弦比派生 `S = b^2/AR`，`cbar = S/b`。
  - 惯量：三个主惯量为正且满足三角不等式，判在重心参考值上。
  - 稳定性符号：`Cm_alpha < 0`、`Cn_beta > 0`、`Cl_p < 0`、`Cn_r < 0`、`Cl_beta < 0`。
  - 尾翼导数符号链：`Cn_beta = -Vv*CY_beta_tail` 为正，`Cn_r = 2*Vv*CY_beta_tail` 为负，`Cn_dr = -Vv*CY_dr` 为负，断言带实测值。
  - 操纵符号：`Cm_de < 0`、`Cl_da > 0`、`Cn_dr < 0`。
  - 采样嵌套：`Ts_outer/Ts_ctrl` 不小于 2。
  - 推进自洽：`T_static_max` 必须等于由峰值轴功率与品质因数按动量理论算出的值；速度表与权威映射在满油门逐点一致；反解出的隐含品质因数不得超过 1。
  - 限幅自洽：`limits.V_min_mps` 由 `limits.V_stall_mps` 与失速裕度因子派生，不得手写。
  - 尾翼与舵面自洽：`Cn_dr`、`Cn_beta`、`Cn_r` 必须由尾容量与舵面几何派生，`abs(Cn_dr)/Cn_beta` 大于 0.30。
  - 配平可行性：巡航配平所需升降舵不得超过行程的合理比例，超过即报错，不许返回工程上不可用的配平点。
- 判据：全部断言通过即 V1 通过。断言失败时消息本身指出冲突的两个量。
- 修订绑定：每次改参数文件必须递增修订号并写进 `P.meta`，作业日志与报告都要打印它。验收数字一律写成「量、值、修订号」三件套，没有修订号的数字不得引用。
- 已发生过一次反例：参数文件在核验期间被改三次，同一套脚本给出迎角 0.1554 与 0.4937、短周期 10.51 与 18.05 两组数，见 `docs/acceptance_report.md` 的 D16。
- 推进模型只有一个权威入口 `params/prop_thrust_from_throttle.m`，速度查询入口是 `params/thrust_from_speed.m`。已废止的 `T = T_static_max*dth^p - k_ram*V` 不许再出现，`P.prop.k_ram` 字段已删除。
- 推力随油门的指数是 4/9，不是 2/3。油门表示峰值轴功率的份额，推力是功率的 2/3 次方，所以 `T` 随油门按 `(2/3)*(2/3) = 4/9` 增长。
- 正确的不变量是功率平衡而不是指数。判据：由 `T` 反解诱导速度 `v_i`，验证 `P_ideal = 2*rho*A*v_i*(V+v_i)^2`，并检查隐含 `FM = T*(V+v_i)/P_shaft` 恒为 `P.prop.power.figure_of_merit`。前飞时推力对功率的指数会从 2/3 趋近 1，所以不许写跨速度的固定指数断言。

### 2.2 L1 配平与线性化层

- `trim = trim_cruise(P, op)`，`op` 取 `P.op` 的一个字段。
- `[lin, info] = linearize_at(P, trim)`。
- 子矩阵索引只在 `linearize_at` 内定义并打印，其他脚本不得重新推导。
- 产物：`results/plant_baseline.mat` 存 `P`、`trim`、`lin`、`info` 与判据量；两份文本报告存人可读的数值。

### 2.3 L2 控制律层

- `gains = design_lqr(P, lin, opts)`：纵向与横航向各一套，含积分增广。
- `gains = design_pid(P, lin)`：级联结构，内环角速率，外环姿态，最外层速度与高度。
- `[da, de, dr, dth_l, dth_r] = allocate(P, M_cmd, T_cmd)`：加权伪逆加饱和处理。
- 参考点自洽性：`x_ref` 与 `u_ff` 必须满足 `A*x_ref + B*u_ff = 0`，容差 1e-9。

### 2.4 L3 闭环仿真层

- 模型程序化生成，生成脚本结尾必须 `save_system`。
- 主求解器定步长 `ode4`，步长 `P.sim.Ts_ctrl`；`ode45` 只用于步长无关性交叉验证。
- 模型里不放字面量数字，全部读 base 工作区的 `P` 字段。
- 产物：轨迹落盘后再画图，图件不参与判据。

### 2.5 L4 验收层

- 每条判据一个可执行断言，断言里写明阈值与失败消息。
- 未安装工具箱对应的判据打印 SKIP 与原因，判据状态记为 SKIP，不计入通过。

## 3. 状态与输入约定

以下顺序与符号来自 `docs/architecture.md` 第 4 节，测试断言直接引用，不重新定义。

- 状态 12 维：`[u v w | p q r | phi theta psi | pn pe pd]`。
- 输入 6 维：`[da de dr dth_l dth_r dT]`。
- 座舱正方向：`de` 后缘下偏产生低头力矩，`da` 右副翼下偏产生右滚，`dr` 后缘左偏产生左偏航。
- `theta` 越过正负 90 度必须在验收里报出，不能静默通过。

## 4. 场景矩阵

| 编号 | 场景 | 所在层 | 判据摘要 | 主证据文件 |
|---|---|---|---|---|
| V1 | 参数自检 | L0 | 断言全通过，质量预算求和相等 | `results/job_*.log` |
| V2 | 配平 | L1 | 归一化残差小于 1e-6，巡航 `alpha` 在 -3 到 +8 度，慢速点 `alpha` 在 2 到 10 度，`de` 按百分比行程判（巡航不超过 10%，任意点不超过 80%） | `results/plant_baseline.mat`、`results/plant_trim_report.txt` |
| V3 | 线性化 | L1 | 12 特征值无正实部，子矩阵按固定索引抽取，模态与解析法差 5% 内 | `results/plant_baseline.mat`、`results/plant_modes_report.txt` |
| V4 | 模态 | L1 | 先判模态类型：过阻尼短周期判实根时间常数小于 0.1 s；振荡型判 `wn` 2 到 20 rad/s 且阻尼大于 0.3；长周期与荷兰滚阻尼大于 0.05 另行报告；滚转时间常数小于 1 s（不敏感，仅报值） | `results/plant_modes_report.txt` |
| V5 | LQR | L2 | 闭环极点全在左半平面，增益与 `lqr` 直接解差 1e-6 内，积分增广消除稳态误差 | `results/gains_baseline.mat`、`results/control_lqr_report.txt` |
| V6 | PID | L2 | 每环上升时间与带宽目标同量级，超调小于 20%，无持续振荡 | `results/control_pid_report.txt` |
| V7 | 控制分配 | L2 | 不饱和区重构误差小于 1e-9，饱和时输出落在 `P.act` 与 `P.prop` 限幅内 | `results/control_allocation_report.txt` |
| V8 | 差动推力 | L2 | 偏航力矩与 `dT*y_motor` 差 1% 内，给出相对方向舵权限的比值；该比值当前有 2 倍分歧，见 `docs/acceptance_report.md` D20 | `results/control_allocation_report.txt` |
| V9 | 风扰 | L3 | 5 m/s 侧风下定直平飞，航迹偏差有界不发散，滚转收敛；前置条件是 PID 增益可用且风向项已修 | `results/sim_*.mat`、场景日志 |
| V10 | 步长无关性 | L3 | `ode45` 与 `ode4` 在同指令下状态轨迹最大差小于 1%，先插值到公共时间轴。**注意**：rev 11 实测本模型在 0.0025 s 离散步长下的真实敏感性约 1.3%，且历史那次 1.888e-8 的 PASS 是退化的（ode45 被离散采样点逼成准定步长，与 ode4 同网格）。阈值与前置条件待重述，见 `docs/handover.md` 4.3 | `results/sim_*.mat`、场景日志 |
| V11 | 模型完整性 | L3 | 每个 `.slx` 从磁盘加载后块数与生成脚本记录一致 | `results/simulink_build_report.txt` |
| V12 | 交叉验证 | L4 | 与 Aerospace Blockset 的 `6DOF (Euler Angles)` 轨迹重合；已用产品目录探测判定为已安装并通过，最差状态相对差 9.12e-17。rev 11 起另加陈旧性前置条件：交叉验证数据不得早于它所评判的参考模型与所依赖的标定 | `results/` 下的交叉验证证据 |
| V13 | 模型与报告一致性 | L4 | 四条：模型块参数必须**按名字**引用 base 变量（不许写死数字）；推送值等于设计字段；线性设计仿真**实际施加**的限幅等于同一字段；同一指令下两个实现的峰值速率指令与稳态量一致。第三条与第四条不可省，因为前两条靠"两边都读同一字段"就能满足 | `results/model_report_consistency.txt`、`results/int_limit_probe.txt` |

## 5. 逐条判据的测法

### V1 参数自检

- 怎么测：调用 `init_uacs_twin('quiet', true)`，捕获断言错误；额外对 `P.mass.items` 求和与 `P.mass.m` 做差。
- 拿哪个文件：`params/init_uacs_twin.m`、`params/uacs_twin_params.m`，运行日志 `results/job_*.log`。
- 判据阈值：无断言错误；质量求和差小于 1e-9。
- 失败时看哪里：断言消息里点名的那个量；惯量三角不等式失败先看 `mass.items` 的分布项与 `dist_scale`。

### V2 配平

- 怎么测：`trim = trim_cruise(P, P.op.cruise)`，取 `trim.residual` 按力与力矩的量纲归一化后取最大绝对值。
- 归一化口径：力残差除以 `P.mass.m * P.env.g`，力矩残差除以 `qbar * S * cbar`。
- 拿哪个文件：`results/plant_trim_report.txt`，结构体在 `results/plant_baseline.mat` 的 `trim`。
- 判据阈值：归一化残差最大值小于 1e-6；`trim.exitflag > 0`。
- 舵偏按百分比行程判，不按固定度数。阈值：巡航点 `abs(trim.de_deg)/act.elevator.max_deg <= 0.10`；全包线任意点不超过 `geom.elevator_trim_frac`（当前 0.80）。
- 迎角窗口按设计点重定。rev 10 巡航所需 `CL` 只有 0.1884，配平迎角是 **-0.3033 度（负值）**，因为 CG 前移到 -0.1576 m 后需要负迎角配平。
  - 巡航点：`alpha_deg` 落在 **-3 到 +8 度**，允许负值，理由写在 `docs/architecture.md` 第 5.0.4 节第 5 条。
  - 慢速点 `P.op.slow`：`alpha_deg` 落在 2 到 10 度，只考核迎角不考核舵偏，因为低速配平本来就要用掉大部分行程。
  - 契约表的 2 到 10 度不适用于本布局。同一套代码在 09:22 修订上算出 0.4937 度、09:26 修订上算出 0.1554 度、rev 10 算出 -0.3033 度，引用迎角必须带修订号。
- 失败时看哪里：`trim.residual` 的哪个分量最大；`alpha` 是否贴住失速边界；油门是否被压到 0 或 1；`theta` 是否被限幅。
- 口径来源见 `docs/acceptance_report.md` 的 D6 与 D16。

### V3 线性化

- 怎么测：`[lin, info] = linearize_at(P, trim)`，检查 `eig(lin.A)` 实部最大值；把 `lin.A_long`、`lin.A_lat` 与 `lin.A` 对应行列逐元素相减；把 `lin.modes_long/lat` 的频率与阻尼同解析式比较。
- 拿哪个文件：`results/plant_modes_report.txt`，矩阵在 `results/plant_baseline.mat` 的 `lin`。
- 判据阈值：`max(real(eig(lin.A))) < 0`；子矩阵抽取误差为 0（同一批元素，浮点完全相同）；解析法模态频率与阻尼差在 5% 内。
- 四个运动学积分状态：`psi`、`pn`、`pe`、`pd` 对应四个零特征值，实部为 0 不算正实部，判据通过，但报告必须把这四个零单独列出来，不许当成三个正常模态。
- 失败时看哪里：`linearize_at` 打印的索引表；`info.step`（步长过大导致雅可比失真）；是否有人另起炉灶对 Simulink 模型重新线性化。

### V4 模态

- 怎么测：读 `lin.modes_long` 与 `lin.modes_lat` 的结构体字段 `freq_rad_s` 与 `damping`；滚转沉降取一阶近似时间常数。
- 拿哪个文件：`results/plant_modes_report.txt`。
- 判据阈值按 rev 10 重定，分两条写，先判模态类型再套窗口：
  - **先判类型**：纵向最快的两个极点若都是实根，短周期是**过阻尼**的，此时不套频率窗口与阻尼比，改判两个实根的时间常数上限（rev 10 实测 0.018695 s 与 0.041045 s，阈值取 0.1 s）。若是一对复根，才按下一档判。
  - **振荡型短周期**：固有频率 `wn` 落在 **2 到 20 rad/s**，阻尼大于 0.3。上沿从 8 放宽的理由：上沿应按俯仰速率环的目标穿越频率 `P.ctrl.bw.rate_q_rad_s` 定，不按运输机惯例。
  - **长周期单独报**：wn 与 zeta 只记录不进 V4 判定。rev 10 实测 wn 0.431531 rad/s、zeta 0.220892、0.0687 Hz。
  - 荷兰滚阻尼大于 0.05。rev 10 实测 0.827599，通过。
  - 滚转沉降时间常数小于 1 s，但这一条**对当前惯量估算不敏感，只报值不作有效判据**。rev 10 实测 0.010575 s。
- **命名规则（已修，必须保持）**：`linearize_at` 必须**按解析估算比对命名**，不许按“频率最高的振荡模态”命名。做法：算出短周期估算 `wn_sp = sqrt(qbar*S*cbar*|Cm_alpha|/Iyy)` 与长周期估算 `wn_ph = sqrt(2)*g/V`，每个模态取与哪个估算更近就命名为哪个；剩余模态按字符命名，过阻尼带时间常数、振荡带频率，这样过阻尼对不会被误读成缺一个模态。
- rev 10 改后实测命名（已落盘，`results/job_plant_baseline.log` 15:50:40，V4 overall PASS）：`short_period` lam -24.3635（tau 0.041045）、`phugoid` lam -0.0953 ± 0.4209i（wn 0.4315、zeta 0.2209）、`long_real_tau_0.0187` lam -53.4905。**判据：实测短周期与单导数估算 `wn_sp` 的偏差必须落在单导数式应有的量级**（当前 -9.1%）；旧规则给 -98.4%，那个量级本身就说明量的不是短周期。判 V4 前先看模态类型，不要直接读标签，详见 `docs/acceptance_report.md` D25。
- 包线扫描留一个格点提示：`lowest speed inside 80 percent travel 16.0000 m/s` 与二分值 `V_min_mps 15.4665` 并存，差 0.53 m/s 等于扫描的 0.5 m/s 步长。引用最低可配平速度用二分值。
- 失败时看哪里：`P.mass.Iyy_cg`、`P.mass.Izz_cg`、`P.mass.Ixx_cg` 的来源标记；`P.aero.Cm_alpha`、`Cn_beta`、`Cl_p` 是否为 ESTIMATE。
- 口径提醒：V4 只判定数值是否落在区间，不判定真机是否如此。三个主惯量标记为 SIZED，但输入是 ESTIMATE，判据通过不等于气动数据可信。
- 历史值提醒：同一套代码在 09:26 修订上给短周期 wn 10.5071、解析估算 7.71、另一份独立实现 18.05 rad/s。三个数分散说明这一条对建模口径极敏感，报告必须写清读法与修订号。

### V5 LQR

- 怎么测：取 `gains.long.K`、`gains.lat.K`，构造闭环 `A - B*K` 求特征值；用同阶 `Q`、`R` 单独调用 `lqr` 复算增益并逐元素比较；检查 `x_ref` 是否满足 `A*x_ref + B*u_ff = 0`；分别用无积分增广与有积分增广做阶跃，比较稳态误差。
- 拿哪个文件：`results/control_lqr_report.txt`，数据在 `results/gains_baseline.mat`。
- 判据阈值：闭环极点实部全为负；增益差小于 1e-6；参考点残差小于 1e-9；无增广稳态误差大于 0，有增广稳态误差绝对值小于指令的 1%。
- 失败时看哪里：`P.lqr` 的 Bryson 起手值是否被改（改过必须有改前值、改后值与理由）；增广矩阵的列顺序是否与状态顺序一致；`x_ref` 是否误用了绝对状态而不是相对配平点的偏差。

### V6 PID

- 怎么测：对每一环单独闭合做阶跃，取上升时间、超调、末段振荡幅值。
- 拿哪个文件：`results/control_pid_report.txt`，仿真函数 `control/pid_cascade_sim.m`，指标函数 `control/step_metrics.m`。
- 判据阈值：上升时间与 `1/P.ctrl.bw.*` 同量级（比值在 0.2 到 5 之间）；超调小于 20%；最后 10% 仿真时长内峰峰值小于稳态值的 2%。
- 失败时看哪里：各环 `Kp, Ki, Kd, Tf, Kb` 与带宽目标的对应关系；速率限幅与积分限幅是否触发；外环带宽是否高于内环。

### V7 控制分配

- 怎么测：在 `M_cmd` 与 `T_cmd` 的网格上调用 `allocate`，检查输出限幅与力矩重构误差。饱和判定用 `info.sat`。
- 拿哪个文件：`results/control_allocation_report.txt`；实现 `control/allocate.m`。
- 判据阈值：不饱和样本的 `norm(M_ach - M_cmd) / max(norm(M_cmd), eps)` 小于 1e-9；所有样本满足 `abs(da) <= deg2rad(P.act.aileron.max_deg)`，`de`、`dr` 同理；`dth_l` 与 `dth_r` 落在 `P.prop.esc.min_throttle` 到 `max_throttle` 之间；油门差不超过 `2 * info.limits.d_eff`。
- 失败时看哪里：`info.sat` 的各个旗标；`info.limits` 里记录的实际限幅值；是否只限了输入没限输出。

### V8 差动推力

- 怎么测：给纯偏航力矩指令，读 `info.dT_ach` 与力臂 `y_arm = P.prop.y_motor - P.mass.y_cg`，与解析式 `abs(dT_ach) * y_arm` 比较；同时算该力矩与满舵方向舵力矩的比值。分两档给指令：小偏差 `dT` 对应局部斜率，大偏差对应端点。
- 拿哪个文件：`results/control_allocation_report.txt`。
- 判据阈值：数值解与解析式差小于 1%。比值必须分档报出：**局部斜率档（推荐引用）**与端点档（会把低油门那台压到零，含义是“一台停车”，不是差动推力）。
- 参考量：巡航点 `d = ±0.1` 时推力差 6.66 N、力矩约 6.0 N·m，等于方向舵权限的 12%。
- 禁止事项：不许把 `P.prop.differential_limit` 当可用权限，也不许引用冻结表里已作废的 40.60 N·m（81.0%），见 D20。
- 失败时看哪里：`allocate.m` 里 `d_eff` 的取值；`y_motor` 与 `y_cg`；反向旋转假设是否被破坏（左右推力不等价）。

### V9 风扰

- 怎么测：在 `P.atmo.wind_ned` 上叠加 5 m/s 侧风分量，定直平飞指令跑 `P.sim.t_end_s`，记录 `pn, pe, pd` 与 `phi`。
- 拿哪个文件：场景脚本与 `results/sim_*.mat`，日志在 `results/job_*.log`。
- 判据阈值（本文件定的执行口径）：末 10 s 内 `|pe|` 增量小于 0.5 m；`|pe|` 最大值不超过 30 m；末 10 s `|phi|` 峰峰值小于 2 度；全程 `|phi|` 不超过 `P.limits.bank_max_deg`。
- 失败时看哪里：风是否只加在平动方程而没有进入气动角；航迹偏差是稳态偏置还是持续增长；滚转是否等幅振荡（说明荷兰滚阻尼不足或滚转环相位不对）。

### V10 步长无关性

- 怎么测：同一初值、同一指令、同一控制器，分别用变步长 `ode45` 与定步长 `ode4` 跑同一条轨迹，先把两条轨迹插值到公共时间轴，再逐状态比较。
- 拿哪个文件：`results/sim_*.mat`、场景日志。
- 判据阈值：`max_k max_i |x_i(t_k) - x_ref_i(t_k)| / scale_i` 小于 1%。`scale_i` 取该状态在参考轨迹上的峰值与 1 之间的较大者，避免除零。
- 失败时看哪里：两条轨迹的点数是否相同（点数不同直接比较会误报等长错误）；控制器是否用了零阶保持而积分器用了连续时间；定步长轨迹是否因步长过大而数值不稳定。
- 实现注意：R2024a 里 `ode4` 不是可调用的 MATLAB 函数，定步长只能走 Simulink 的定步长求解器或自写 RK4。脚本里直接写 `ode4(...)` 必然报未定义函数。

### V11 模型完整性

- 怎么测：先 `close_system('all')`，再对每个 `.slx` 执行 `load_system`，用 `find_system(mdl, 'SearchDepth', inf, 'Type', 'Block')` 计数，与生成脚本记录的清单逐块名比对；同时比较 `.slx` 的修改时间与生成脚本运行时刻。
- 拿哪个文件：`results/simulink_build_report.txt`，模型在 `model/`。
- 判据阈值：块数与记录完全相等；顶层块名集合相等；`.slx` 修改时间不早于生成脚本运行时刻。
- 失败时看哪里：生成脚本结尾是否真的执行了 `save_system`；保存是否在缩略图生成阶段失败；内存里的模型是否比盘上的新。

### V12 交叉验证

- 状态：SKIP。Aerospace Blockset 未安装。
- 探测判据：产品是否出现在 `ver` 列表，以及 `load_system('aeroblks')` 是否成功。不允许用 `license('test','Aerospace_Blockset')` 判定，该调用在本机返回 1，属于假阳性。
- 装好后的测法：用 `6DOF (Euler Angles)` 与 `Atmosphere (ISA)` 搭对照模型，施加与自写 6DOF 相同的舵面与推力时程，两条轨迹插值到公共时间轴后比较，最大差小于 1%。
- 失败时看哪里：两边的坐标系定义、欧拉角顺序、重力方向、力矩参考点是否一致。

### V13 模型与报告一致性（rev 11 新增）

- 状态：PASS，72 项全过。`verify/v_sim_v13_consistency.m`，已挂进 `v_sim_all`。
- 为什么需要它：控制器交付两次——`control/pid_cascade_sim.m` 是 V6 全部线性读数的来源，
  `model/uacs_manual_pid.slx` 是实际飞的那个。**两个实现会静默漂开**，rev 11 之前就在三处漂开了：
  theta 积分状态界差 2.5 倍、`V` 差 0.8 倍（方向还相反）、`q_c` 指令限幅差 68 倍。
  第三处最严重：线性仿真夹在 80 rad/s（永不动作），模型夹在 1.1678，
  **于是 V6 验的是一个没有该限幅的控制器**，而那个限幅正是当初救回滚转与偏航环的那一个。
- 四条腿，逐条加强：
  1. `names`：模型块参数（`Kp` 增益、`Ki` 的 `gainval` 与 `UpperSaturationLimit`、`pol`、
     三个 `*_c_lim` 的 `UpperLimit`）**按名字**引用 base 变量。防的是模型里写死数字。
  2. `pushed`：`sim_push_design_vars` 推给那些变量的值等于设计字段。
  3. `applied`：`pid_cascade_sim` 本次运行**实际施加**的限幅等于同一字段（读它自己记录的 `meta`）。
  4. `behaviour`：同一滚转指令过两个实现，两侧的**峰值速率指令**都必须精确等于限幅，
     且稳态坡度相差在 2 度内。
- **为什么第三条与第四条不可省**：前两条靠"两边都读同一字段"就能满足，
  抓不住"其中一边压根没读"。第四条是唯一能抓住"一边有限幅一边没有"的腿：
  rev 11 修之前，线性侧峰值是 1.399 rad/s，模型侧是 0.4776，一眼就分开。
- 拿哪个文件：`results/model_report_consistency.txt`（逐项对照表）、
  `results/int_limit_probe.txt`（模型积分器状态直读）。
- 失败时看哪里：`applied` 行失败说明 `pid_cascade_sim` 与 `P.ctrl` 脱节；
  `behaviour` 行失败说明两个实现的限幅位置或取值不同。

## 6. 判据表

| 编号 | 阈值 | 量纲与口径 |
|---|---|---|
| V1 | 断言零失败，质量求和差小于 1e-9 | kg |
| V2 | 归一化残差小于 1e-6，巡航 `alpha` -3 到 +8 度，慢速点 `alpha` 2 到 10 度，`de` 按行程百分比（巡航不超过 10%，任意点不超过 80%） | 力除以 `m*g`，力矩除以 `qbar*S*cbar`，舵偏除以 `act.elevator.max_deg` |
| V3 | `max(real(eig(A))) < 0`，模态差小于 5% | rad/s 与无量纲阻尼 |
| V4 | 先判类型：过阻尼短周期判实根时间常数小于 0.1 s；振荡型 `wn` 2 到 20 rad/s 且阻尼大于 0.3；荷兰滚阻尼大于 0.05；长周期只报不判；滚转时间常数小于 1 s（不敏感，仅报值） | rad/s，无量纲，s |
| V5 | 闭环极点实部为负，增益差小于 1e-6，参考点残差小于 1e-9 | 无量纲 |
| V6 | 上升时间比在 0.2 到 5，超调小于 20%，尾振小于 2%；设计作业自检另加**级联分离度**每项比值小于 0.5、抗饱和开/关两臂必须可分辨、积分限幅咬合率低于 10% | s，百分比 |
| V7 | 重构误差小于 1e-9，输出全部在限幅内 | N*m，rad，无量纲油门 |
| V8 | 力矩差小于 1%，给出与方向舵权限的比值 | N*m |
| V9 | 末段航迹增量小于 0.5 m，航迹偏差小于 30 m，末段滚转峰峰值小于 2 度 | m，度 |
| V10 | 归一化最大状态差小于 1%。**阈值待重述**：实测本模型真实敏感性约 1.3%，且需加"两侧采样点数必须独立"的前置条件 | 无量纲 |
| V11 | 块数与块名集合完全一致 | 个 |
| V12 | 最大相对差小于 1e-4（实测 2.22e-14），且交叉验证数据不得早于参考模型与标定 | 无量纲 |
| V13 | 72 项全部相等：命名 31 项字符串精确、推送 31 项相对差小于 1e-12、施加 7 项相对差小于 1e-12、行为 3 项（两侧峰值速率指令都等于限幅，容差 1%；稳态坡度差小于 2 度） | rad/s，度，无量纲 |

## 7. 已知边界

环境类边界见 `docs/architecture.md` 第 8 节与第 9 节，本文件只列与判据直接相关的部分。

- Aerospace Blockset 已确认安装，V12 的判定改用产品目录探测（不用许可证探测），当前结论是 PASS，最差状态相对差 9.12e-17。UAV Toolbox 仍未安装，坐标变换与日志分析用自写函数。
- MATLAB 批处理启动在本机失败（`docs/architecture.md` 第 8.1 节），MATLAB 侧工作一律通过 MCP 通道在共享会话内执行。
- 只有一个 MATLAB 会话，必须串行（`docs/architecture.md` 第 8.2 节）。会话已四次卡死，撞上超时不要重试，写文件通报（第 8.3 节）。
- `ode4` 不是 R2024a 的 MATLAB 函数，定步长对比只能走 Simulink 或自写积分器。
- 气动导数与大部分几何量是 ESTIMATE，判据通过只说明内部自洽。
- **`V_mc` 与速度包线是海平面值。** `P.atmo.model = 'const'`（默认）取海平面密度。150 m ISA 下 `V_mc` 从 24.452667 升到 24.570526（物理一致）或 24.629668（按现状实现，推力映射取海平面），对巡航 25.0 的裕度从 2.189% 缩到 1.718% 或 1.481%。凡引用 `V_mc` 或“单发可控”必须标注高度；只写数值不写高度按缺陷处理，见 `docs/acceptance_report.md` D23。
- **高度修正不是通用值**：上一条的三组数都是 150 m 的修正量，其他高度按 `rho^(1/3)` 律重算。按现状实现的口径更保守，引用时优先用它。
- **已知偏差 D22：巡航阻力与油门有两个口径，`trim_cruise` 是权威。** `sizing_summary` 用解析迎角算 `CD = 0.032896`、油门 0.423410；配平点实测 `CD = 0.033219`、油门 0.430065。**凡引用巡航阻力、油门、电功率，一律用配平点实测值并注明口径**，`sizing_summary` 的值只能当估算。
- **D25 已修复：模态命名改为按解析估算比对。** 旧规则按“频率最高的振荡模态”命名，短周期过阻尼时必然挂错，V4 因此会误报 FAIL。命名规则已写在 V4 一节，改这一条前先读它，见 `docs/acceptance_report.md` D25。
- **已知偏差 D26：推力映射硬编码海平面密度。** `P.atmo.model = 'isa'` 时气动力随密度下降而推力不下降，包线偏乐观。引用 `V_mc` 必须写明是海平面值且推力映射取海平面。
- **V9 的初值规则**：侧风场景必须以**蟹行配平** `wind_trim(trim, wind_ned, track_deg)` 为初值，不许用静风配平再加风。用静风配平会让开局带 `beta = -asin(W_E/V)`（5 m/s 风、25 m/s 时 -11.54 度）的侧滑，测到的是初值不在平衡点，不是风扰响应。实测：5 m/s 东风航迹沿北时蟹行 -11.53696 度、地速 24.49490 m/s、5 s 东向漂移 4.441e-15 m。证据 `results/atmo_wind_report.txt` G 段。
- **ISA 块的边界行为**：Aerospace 的 ISA 块把高度入口饱和在 0 m，`plant/atmo_isa.m` 不饱和、按递减率延伸。h >= 0 时两者最大相对差 7.420e-16；h < 0（-50 到 -2.5 m）时 4.809e-03，因为块在 20 个点上全部返回 1.225。V12 五配置全在 h >= 0，结论不受影响；把场景飞到参考高度以下时要先选口径。
- 判据配置故意用 `P.atmo.model = 'const'`，为的是让两个模型处在同一种流体里、把积分器与运动学差异隔离出来。这是设计选择，不是遗漏。
- `Cm_alphadot` 按 `alpha_dot = q` 取值，有风时不再精确，但它是小量。
- 伺服分辨率、间隙、螺旋桨扭矩反作用、桨叶挥舞在本版不建模。
- 惯量由 `mass.items` 点质量加统一分布因子派生，展向质量分布没有分开建模，V4 的滚转判据因此缺区分度，短周期频率也受影响，见 `docs/acceptance_report.md` 缺陷记录。
- 气动控制导数已改为由几何派生：`CL_de = tau_elevator*CL_alpha*Sh/S`，`CY_dr = k_body*tau_rudder*CL_alpha*Sv/S`。此前 `CL_de` 与自己的公式差 6.6 % 的缺陷已修，见 `docs/acceptance_report.md` 的 D10。
- 单发失效的判据要分两层写，不要合并成一条。
  - 可控性边界由 `V_mc = sqrt(2*T_static*y_motor/(rho*S*b*abs(Cn_dr)*dr_max))` 决定，`Cn_beta` 不进入该式。rev 10 实测：单发偏航力矩 **47.9443 N·m**，方向舵 **50.1146 N·m**，比值 **0.9567**，`V_mc = 24.452667 m/s`（海平面值，见上一条的高度纪律）。
  - `abs(Cn_dr)/Cn_beta` 是侧滑品质指标，含义是满舵时平衡所需的侧滑角等于该比值乘 `dr_max`，不是可控性边界。rev 10 实测 **0.341912**；公开 Aerosonde 参考模型的对应值是 0.9452，但那个值是由 `Cn_dr` 与 `Cn_beta` 反推的等效值，源文件没有写方向舵弦长比。
  - 机理：按当前派生链，该比值等于 `k_body*tau_rudder*CL_alpha*S_rudder/(0.75*S)`，只取决于方向舵面积 `S_rudder`，与竖直尾翼面积本身无关。尾翼加大而舵面不同步加大，比值一位不变；`Cn_r` 不出现在 `V_mc` 里。三组固定舵面积的算例见 `docs/acceptance_report.md` 第 3.9 节。
  - 失效处置必须写成收油门下降，见 `docs/architecture.md` 第 5.0.3 节。
- **已知偏差 D19：`Cm_de` 重复计入尾容量。** 现行文件式比标准式小一个 `Vh` 因子（rev 10 下 2.18 倍），升降舵权限与 `V_min_trim` 都偏保守：权限 167.07 N·m 应为 364.16，最低可配平速度 15.4665 m/s 应更低。方向是偏保守，不构成安全风险；修法排在 rev 11。**判据里引用升降舵权限必须注明是“现行文件式口径”。**
- **已知偏差 D20：差动推力字段含义有误。** 现报的是“把低油门那台压到零”的端点值，且用线性律，不是差动推力的有效权限。巡航点有效权限应取局部斜率：`d = ±0.1` 时约 6.0 N·m，等于方向舵的 12%。**结论是巡航点差动推力不能作为常规偏航手段**，只在低速油门余量大时有用。V8 在此之前只报 `dT_ach*y_motor` 与方向舵权限的比值，不报“可用权限”。
- 巡航点需用推力约 29.1 N，油门约 0.43，配平迎角接近零。任何按大迎角假设写的判据都要先按设计点核对。

## 8. 运行方式

规定见 `docs/architecture.md` 第 8.2 节，执行要点如下。

- 跑 MATLAB 前先取锁，锁由一个作业持有，不按单次调用持有。
- 作业包在 `run_job` 里，日志落 `results/job_<name>.log`。
- 判定成功只看日志里的 `=== JOB DONE`，不看退出码，也不看是否出现 `!! JOB FAILED`。
- 作业脚本命名 `uacs_job_<name>.m`，`<name>` 用小写字母数字下划线，且脚本自己 `addpath`、自己算项目根、不依赖调用者当前目录。
- 持锁超过 2 小时判为僵死并被强制打破，日志留痕。
- 用 `run_matlab_file` 跑脚本时，脚本名必须以字母开头且只含字母数字下划线，跑完当前目录停在脚本目录，收尾要切回。
- 结果先落盘再画图，图件不参与判据。

## 9. 新外形与新马达参数灌入流程

外形与动力确定后，按下述顺序操作。每一步都给出改哪个文件、改哪些字段、跑什么、看什么。

### 第 1 步 改几何与质量

改 `params/uacs_twin_params.m` 的 `geom` 与 `mass.items`。

- `geom.b` 是输入，硬上限 `geom.span_limit_m = 3.50 m`，超限在 init 直接报错。
- `geom.AR` 是自由外形参数，`geom.S = b^2/AR` 是派生量，不要手写面积。
- `geom.taper`、`geom.dihedral_deg`、`geom.sweep_deg`、`geom.iwing_deg`。
- `geom.Sh`、`geom.lh`、`geom.ih_deg`、`geom.Sv`、`geom.lv`，以及 `geom.aileron`、`geom.elevator`、`geom.rudder` 的面积比。竖直尾翼是按单发操纵需求定尺寸的，改小之前先看 `V_mc`。
- `mass.items` 每行的质量与三个坐标。改完必须保证质量列之和等于 `mass.m`，否则 init 直接报错。载荷 12 kg 与总重 20 kg 都体现在这张表里。
- `cbar`、`Vh`、`Vv`、`S` 由公式派生，不要手写。
- `mass.Ixx_cg` 等四个惯量由 `mass.items` 派生，改完自动跟着动。翼质量目前全放在 y = 0，换真实质量分布前不要相信滚转相关判据。

### 第 2 步 换马达与螺旋桨

改同一文件的 `prop` 段。

- `prop.D`、`prop.pitch_in`、`prop.rpm_static`。
- `prop.power.peak_per_motor_W`（电功率峰值）、`prop.power.figure_of_merit`、三个效率 `eta_prop_cruise`、`eta_motor`、`eta_esc`。
- `prop.T_static_max`、`prop.P_shaft_peak`、`prop.z_thrust_offset`、`prop.speed_table_T` 全是派生量，不要手填。
- `prop.throttle_exponent` 描述油门到轴功率的幂次，当前 2/3。
- `prop.y_motor`、`prop.x_motor`、`prop.z_motor` 按实际安装位置改。
- 拿到实测静推力后，把它作为校核值反推 `figure_of_merit`，不要直接把 `T_static_max` 写成实测值，否则两处输入重新变成独立量。
- 换完跑 `run/check_rescale.m` 一类的功率平衡检查：十个速度点上的 `P_ideal` 必须相同，隐含品质因数必须恒等于设定值且不超过 1。

### 第 3 步 改气动系数

改 `aero` 段。所有导数按弧度，`CL_alpha`、`CD0`、`Cm_alpha`、`Cm_de` 优先用风洞或 CFD 值替换。

- 改完把来源标记从 ESTIMATE 改成 MEASURED，并同步 `meta.estimated_fields` 与 `meta.sized_fields`。
- 符号约定不得改：`Cm_de < 0`、`Cl_da > 0`、`Cn_dr < 0`。

### 第 4 步 改工作点与限幅

- `op.cruise.V_mps`、`op.cruise.h_m` 是主线性化点。
- `act.*.max_deg` 与 `rate_deg_s` 按实际舵机改。
- `limits.V_stall_mps` 由质量、机翼面积与 `CL_max` 派生，`limits.V_min_mps` 再由失速裕度因子派生，两个都不要手写。
- `limits.V_max_mps` 按结构限制改，注意它现在低于 `V_mc`。
- 续航目标 `prop.endurance_target_s` 与电池假设在 `sizing/sizing_summary.m` 里，电池质量按 200 Wh/kg 与 80% 放电深度算，这两个数是假设不是实测。

### 第 5 步 按序重跑

1. 参数自检与配平线性化：跑 L0 自检后跑 plant 作业，产出 `results/plant_baseline.mat`、`results/plant_trim_report.txt`、`results/plant_modes_report.txt`。
2. 控制律：跑 control 作业，产出 `results/gains_baseline.mat` 与三份 control 报告。
3. 仿真：跑 sim 作业，产出 `results/sim_*.mat` 与 `results/simulink_build_report.txt`。
4. 验收：跑验收用例，把结果写进 `docs/acceptance_report.md`。

### 第 6 步 看哪些判据

- 必看：V1、V2、V3、V4、V5、V7、V11。
- 换动力后必看：V8 的权限比值与 `d_eff`，以及发动机失效偏航力矩。
- 换几何后必看：V4 的三个模态是否还在区间内，V9 的风扰航迹偏差是否仍有界。
- 任何一条从通过变成失败，都在 `docs/acceptance_report.md` 里记下改前值、改后值与诊断。

## 10. 证据文件清单

| 文件 | 内容 | 生成者 |
|---|---|---|
| `results/plant_baseline.mat` | `P`、`trim`、`lin`、`info` 与判据量 | L1 |
| `results/plant_trim_report.txt` | 配平残差、`alpha`、`de`、油门 | L1 |
| `results/plant_modes_report.txt` | 12 特征值、模态频率与阻尼、子矩阵索引 | L1 |
| `results/gains_baseline.mat` | 纵向与横航向增益、参考点、闭环极点 | L2 |
| `results/control_lqr_report.txt` | LQR 判据、与直接解的比较、稳态误差 | L2 |
| `results/control_pid_report.txt` | 各环阶跃指标 | L2 |
| `results/control_allocation_report.txt` | 重构误差、限幅检查、权限比值 | L2 |
| `results/sim_*.mat` | 闭环轨迹与场景记录 | L3 |
| `results/simulink_build_report.txt` | 每个模型的块清单 | L3 |
| `results/job_*.log` | 每次运行的开始与结束标记 | 全部 |
| `results/doc_link_check.txt` | 引用验活记录 | L4 |
| `results/doc_recompute.txt` | 独立复算：质量、惯量、巡航、推进表、操纵权限 | L4 |
| `results/doc_accept_probe.txt` | 参数读回与判据探针输出 | L4 |
| `docs/acceptance_report.md` | 逐条判据的实测值与结论 | L4 |

## 11. 引用

以下地址在 2026-10-08 逐条发过请求并返回 200。

- MathWorks，`lqr` 参考页：https://www.mathworks.com/help/control/ref/lti.lqr.html
- MathWorks，`lqi` 参考页：https://www.mathworks.com/help/control/ref/ss.lqi.html
- MathWorks，PID Controller 块参考页：https://www.mathworks.com/help/simulink/slref/pidcontroller.html
- MathWorks，Choose a Solver：https://www.mathworks.com/help/simulink/ug/choose-a-solver.html
- MathWorks，6DOF (Euler Angles)：https://www.mathworks.com/help/aeroblks/6dofeulerangles.html
- MathWorks，Equations of Motion：https://www.mathworks.com/help/aeroblks/equations-of-motion-2.html
- MathWorks，Coordinate Systems：https://www.mathworks.com/help/aeroblks/coordinate-systems.html
- MathWorks，Dryden Wind Turbulence Model (Continuous)：https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html
- MathWorks，Discrete-Time Integrator：https://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html
- MathWorks，`linearize`：https://www.mathworks.com/help/slcontrol/ug/linearize.html
- MathWorks，`ode45`：https://www.mathworks.com/help/matlab/ref/ode45.html

契约原文见 `docs/architecture.md`，参数权威源见 `params/uacs_twin_params.m`。
