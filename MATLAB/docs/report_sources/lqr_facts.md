# LQR 取证笔记

只记录从源码读到的内容。每条事实后括号里是来源文件与行号。需要推算的数值标注「推导值，未实测」。

## 0. 本次实际读过的文件

- `MATLAB/control/design_lqr.m`（256 行）
- `MATLAB/control/lqr_reference.m`（100 行）
- `MATLAB/control/lin_contract.m`（77 行）
- `MATLAB/control/allocate.m`（282 行）
- `MATLAB/plant/linearize_at.m`（251 行）
- `MATLAB/plant/trim_cruise.m`（200 行）
- `MATLAB/sim/sim_push_design_vars.m`（162 行）
- `MATLAB/sim/sim_gains_ph.m`（70 行）
- `MATLAB/sim/sim_lin_ph.m`（98 行）
- `MATLAB/sim/sim_lqr_refmap.m`（84 行）
- `MATLAB/sim/sim_alloc_sfun.m`（49 行）
- `MATLAB/sim/sim_alloc_wrap.m`（30 行）
- `MATLAB/sim/sim_alloc_act.m`（37 行）
- 另有取证时查证的旁证文件：`MATLAB/params/uacs_twin_params.m`、`MATLAB/control/uacs_job_control_design.m`、`MATLAB/sim/sim_bench_mix.m`、`MATLAB/sim/sim_dth_for_thrust.m`、`MATLAB/docs/architecture.md`、`MATLAB/docs/design_notes_sim.md`。

## 1. 状态空间怎么得到的

### 1.1 全量状态向量

12 个分量，次序固定（来源：plant/linearize_at.m:73）。

| 序号 | 符号 | 含义 | 单位 |
|---|---|---|---|
| 1 | `u` | 机体系前向速度 | m/s |
| 2 | `v` | 机体系右向速度 | m/s |
| 3 | `w` | 机体系下向速度 | m/s |
| 4 | `p` | 滚转角速率 | rad/s |
| 5 | `q` | 俯仰角速率 | rad/s |
| 6 | `r` | 偏航角速率 | rad/s |
| 7 | `phi` | 滚转角 | rad |
| 8 | `theta` | 俯仰角 | rad |
| 9 | `psi` | 偏航角 | rad |
| 10 | `pn` | 北向位置 | m |
| 11 | `pe` | 东向位置 | m |
| 12 | `pd` | 地向位置，向下为正 | m |

单位依据见参数文件头部的约定段：角度 rad、位置 m、姿态 rad、角速率 rad/s、速度 m/s，机体轴 x 前 y 右 z 下（来源：params/uacs_twin_params.m:13-15）。

### 1.2 输入向量

6 个分量，次序固定（来源：plant/linearize_at.m:74）。

1. `da` 副翼偏度，rad
2. `de` 升降舵偏度，rad
3. `dr` 方向舵偏度，rad
4. `dth_l` 左油门，0..1
5. `dth_r` 右油门，0..1
6. `dT` 总推力增量，N

第 6 路与两路油门在物理上是同一路执行机构，不可同时当独立自由度（来源：docs/architecture.md:60）。

### 1.3 输出向量

`lin.C = eye(12)`，`lin.D = zeros(12, 6)`（来源：plant/linearize_at.m:69-70）。输出就是全部 12 个状态，无前通项。项目里没有另外定义输出选择矩阵。

### 1.4 线性化方法

数值中心差分，不是解析雅可比（来源：plant/linearize_at.m:51、plant/linearize_at.m:57）。

`A(:, i) = (eom_6dof(0, x+h_i*e_i, u0, P) - eom_6dof(0, x-h_i*e_i, u0, P)) / (2*h_i)`

`B(:, j) = (eom_6dof(0, x0, u+h_j*e_j, P) - eom_6dof(0, x0, u-h_j*e_j, P)) / (2*h_j)`

`eom_6dof` 的第一个参数是时间，这里恒传 0，即取定常点（来源：plant/linearize_at.m:51）。

### 1.5 扰动步长

默认 `opts.step = 1e-6`（来源：plant/linearize_at.m:28-30）。头注释里的示例写的是 1e-5，与实际默认值不一致（来源：plant/linearize_at.m:5）。

步长规则按分量缩放（来源：plant/linearize_at.m:41-42）：

- 状态：`hx(i) = step * max(1, abs(x0(i)))`
- 输入：`hu(j) = step * max(1, abs(u0(j)))`

所以在配平点上，凡是模小于 1 的分量实际扰动就是 1e-6 本身，速度分量按自身量级放大。注释给出的理由是角度与角速率都拿到 step 的绝对量（来源：plant/linearize_at.m:21-22）。

全部调用点都显式传 1e-6，没有用默认值的：控制设计作业传 `struct('step', 1e-6, 'verbose', true)`（来源：control/uacs_job_control_design.m:30），仿真设计缓存与配置脚本同样传 1e-6（来源：params/uacs_apply_config.m:147），results 下的诊断脚本一律传 1e-6 且 `verbose` 关掉。

雅可比求值次数 `info.n_eval = 2*(n+m) = 36`（来源：plant/linearize_at.m:46-59 的循环，计数字段在 plant/linearize_at.m:95）。

### 1.6 纵向与横航向怎么切

抽取索引写死在 `linearize_at.m` 里并且打印出来，不允许别处重推（来源：plant/linearize_at.m:61-64，docs/architecture.md:359）。

- 纵向状态 `idx_long_x = [1 3 5 8]`，即 `u w q theta`（来源：plant/linearize_at.m:61）。
- 纵向输入 `idx_long_u = [2 6]`，即 `de dT`（来源：plant/linearize_at.m:62）。
- 横航向状态 `idx_lat_x = [2 4 6 7]`，即 `v p r phi`（来源：plant/linearize_at.m:63）。
- 横航向输入 `idx_lat_u = [1 3]`，即 `da dr`（来源：plant/linearize_at.m:64）。

子矩阵是直接从 `A`、`B` 里取元素，不再重新线性化（来源：plant/linearize_at.m:79-83，注释在 plant/linearize_at.m:23）。

切分依据是配平点上纵横解耦。旁证：`eig(A)` 与 `eig(A_long)`、`eig(A_lat)` 加四个零的并集最大失配 7.161e-15，记录在 plant 设计说明里（来源：docs/design_notes_plant.md:122）。该数值属于文档记录，不是本次从源码读出的。

### 1.7 契约坐标变换

`lin_contract` 把子模型搬到 `architecture.md 5.2` 冻结的次序：纵向 `[dV dalpha dq dtheta]` 配 `[dde dT]`，横航向 `[dbeta dp dr dphi]` 配 `[dda ddr]`（来源：control/lin_contract.m:5-7）。

原因是 `linearize_at` 直接抽出来的是机体分量 `u`、`w`、`v`，而 Bryson 权重是按空速与迎角定的，两者差一个 `V0` 因子（来源：control/lin_contract.m:8-12）。

变换式（来源：control/lin_contract.m:14-16，实现在 control/lin_contract.m:39-52）：

```
dV     =  cos(a0)*du + sin(a0)*dw
dalpha = -sin(a0)*du/V0 + cos(a0)*dw/V0
dbeta  =  dv/V0
```

其中 `V0 = norm(x_star(1:3))`，`a0 = atan2(x_star(3), x_star(1))`（来源：control/lin_contract.m:29-30）。

矩阵形式：`T2 = [cos(a0) sin(a0); -sin(a0)/V0 cos(a0)/V0]`（来源：control/lin_contract.m:39），`T_long = blkdiag(T2, eye(2))`，`T_lat = diag([1/V0, 1, 1, 1])`（来源：control/lin_contract.m:41、control/lin_contract.m:50）。

应用方式为相似变换加输入变换：`lc.A_long = T_long * lin.A_long / T_long`，`lc.B_long = T_long * lin.B_long`，横航向同理（来源：control/lin_contract.m:42-43、control/lin_contract.m:51-52）。

是否施加由子模型的状态名决定：状态名里出现 `w` 或 `u` 就转纵向，出现 `v` 就转横航向（来源：control/lin_contract.m:36-37）。已是空速状态时跳过，`map.mode` 记成 `identity`（来源：control/lin_contract.m:25、control/lin_contract.m:22）。

`map` 汇报 `mode`、`T_long`、`T_lat`、`V0`、`alpha0`、`reason`（来源：control/lin_contract.m:25-26，docs/architecture.md 未列此结构）。

配平点上的空速与迎角也从变换里还原出来，供参考轨迹使用（来源：plant/linearize_at.m:142-145）。

### 1.8 线性化点怎么取

配平结果直接充当线性化点：`x0 = trim.x_star(:)`，`u0 = trim.u_star(:)`（来源：plant/linearize_at.m:35-36）。所以 A、B 是以配平点为原点的雅可比，原点即平衡点，偏差坐标下的前馈为零（来源：control/design_lqr.m:23-25）。

配平点本身：巡航 `h = 150 m`、`V = 25 m/s`、`gamma = 0`、`bank = 0`（来源：params/uacs_twin_params.m:101-104）。其余备选工作点为爬升 23 m/s 与 6 度、慢速 18 m/s、盘旋 23 m/s 与 30 度（来源：params/uacs_twin_params.m:108-122）。

### 1.9 模态是从特征值算的

`lin.modes_long = local_modes(eig(lin.A_long), ...)`，横航向同理（来源：plant/linearize_at.m:88-89）。

每个模态记 `lambda`、`real_part`、`imag_part`、`freq_rad_s`、`freq_nat_rad_s`、`freq_damped_rad_s`、`freq_hz`、`damping`、`tau_s`、`is_oscillatory`、`stable`（来源：plant/linearize_at.m:137-140）。

频率口径：`freq_rad_s` 与 `freq_nat_rad_s` 都取极点模 `|lambda|`，`freq_damped_rad_s` 取 `|Im|`（来源：plant/linearize_at.m:236-237）。注释说明取模与 Control System Toolbox 的 `damp()` 口径一致（来源：plant/linearize_at.m:15-16、plant/linearize_at.m:234-235）。

阻尼比 `damping = -real(l)/abs(l)`，实根固定为 1 且虚部清零（来源：plant/linearize_at.m:240-245）。时间常数 `tau_s = -1/real(l)`，非负实部时为 `Inf`（来源：plant/linearize_at.m:246-250）。

纵向模态命名用两个解析估计式，而不是「取最快的振荡模态」：短周期 `sqrt(qbar0*S*cbar*|Cm_alpha|/Iyy_cg)`，长周期 `sqrt(2)*g/V0`（来源：plant/linearize_at.m:182-183）。命名按与估计值最接近的固有频率贴标签（来源：plant/linearize_at.m:184-189）。改这个规则的原因是旧规则在短周期过阻尼时把短周期名字贴到了长周期上（来源：plant/linearize_at.m:171-180）。

横航向命名：振荡模态一律叫 `dutch_roll`（来源：plant/linearize_at.m:202-205）；实根按 `|real|` 降序，单个实根时大于 1 叫 `roll_subsidence` 否则叫 `spiral`（来源：plant/linearize_at.m:206-214）。

## 2. LQR 的 Q 与 R

### 2.1 数值与来源

权重直接读 `P.lqr`，设计脚本不修改（来源：control/design_lqr.m:17-19、control/design_lqr.m:50-51）。定义处（来源：params/uacs_twin_params.m:532-535）：

```
lqr.long.q_diag = [1/2.0^2, 1/deg2rad(5)^2, 1/deg2rad(15)^2, 1/deg2rad(10)^2]
lqr.long.r_diag = [1/deg2rad(20)^2, 1/0.30^2]
lqr.lat.q_diag  = [1/deg2rad(5)^2, 1/deg2rad(60)^2, 1/deg2rad(30)^2, 1/deg2rad(20)^2]
lqr.lat.r_diag  = [1/deg2rad(20)^2, 1/deg2rad(20)^2]
```

换算成数值（推导值，未实测）：

| 通道 | 项 | 对应物理量 | 分母含义 | 数值 |
|---|---|---|---|---|
| 纵向 Q | 1 | `dV` 空速偏差 | 2.0 m/s | 0.25 |
| 纵向 Q | 2 | `dalpha` 迎角偏差 | 5 度 | 131.45 |
| 纵向 Q | 3 | `dq` 俯仰角速率 | 15 度/s | 14.59 |
| 纵向 Q | 4 | `dtheta` 俯仰角 | 10 度 | 32.83 |
| 纵向 R | 1 | `dde` 升降舵增量 | 20 度 | 8.213 |
| 纵向 R | 2 | `dT` 推力增量 | 0.30 N | 11.11 |
| 横航向 Q | 1 | `dbeta` 侧滑角 | 5 度 | 131.45 |
| 横航向 Q | 2 | `dp` 滚转角速率 | 60 度/s | 0.912 |
| 横航向 Q | 3 | `dr` 偏航角速率 | 30 度/s | 3.648 |
| 横航向 Q | 4 | `dphi` 滚转角 | 20 度 | 8.213 |
| 横航向 R | 1 | `dda` 副翼增量 | 20 度 | 8.213 |
| 横航向 R | 2 | `ddr` 方向舵增量 | 20 度 | 8.213 |

`deg2rad(1) = 0.0174533`。写法是 Bryson 规则「1 除以该项可接受最大偏差的平方」（来源：params/uacs_twin_params.m:530-531）。

注意 `dT` 的容差写成绝对量 0.30 N，不是角度（来源：params/uacs_twin_params.m:533）。`R` 里两个舵面都按 20 度取，横航向副翼与方向舵同权（来源：params/uacs_twin_params.m:535）。

权重可被覆盖：`lqr = uacs_apply_override('lqr', lqr)`（来源：params/uacs_twin_params.m:536），覆盖接口把外部配置写回这四个字段（来源：params/uacs_apply_config.m:104-107）。

### 2.2 维数与构造

`R = diag(r_diag)`，纵向 2x2、横航向 2x2（来源：control/design_lqr.m:60、control/design_lqr.m:122）。

增广前 `Q = diag(q_diag)`，4x4（来源：control/design_lqr.m:59）。增广后 `Q_aug = blkdiag(diag(q_diag), q_int)`，5x5（来源：control/design_lqr.m:59）。

### 2.3 积分增广

增广方式（来源：control/design_lqr.m:57-58，横航向 control/design_lqr.m:119-120）：

```
A_aug = [A, zeros(4,1); C_int, 0]     5x5
B_aug = [B; zeros(1,2)]               5x2
```

`C_int` 是 1x4 的选择行，被调输出的那一列为 1，其余为 0（来源：control/design_lqr.m:54-55，横航向 control/design_lqr.m:116-117）。

积分状态的权 `q_int` 取「它调的那个状态自己的 Bryson 容差」：`q_int = q_diag(idx)`（来源：control/design_lqr.m:52-53、control/design_lqr.m:18-20、横航向 control/design_lqr.m:114-115）。

被调输出可由选项切换，纵向默认 `'V'`（下标 1），可换 `'theta'`（下标 4）（来源：control/design_lqr.m:33-35、control/design_lqr.m:9、control/design_lqr.m:52）。横航向默认 `'phi'`（下标 4），可换 `'beta'`（下标 1）（来源：control/design_lqr.m:36-38、control/design_lqr.m:11、control/design_lqr.m:114）。名字非法时断言失败（来源：control/design_lqr.m:175）。

作业里跑了两套对照：默认一套加 `long_int_out='theta'`、`lat_int_out='beta'` 各一套（来源：control/uacs_job_control_design.m:34-36）。

### 2.4 两套增益都留存

同时解增广与未增广两套：`K` 为 2x5，`K_unaug = lqr(A, B, diag(q_diag), R)` 为 2x4（来源：control/design_lqr.m:62-63，横航向 control/design_lqr.m:124-125）。头注释说明理由：无积分时阶跃有稳态误差，有积分时没有，判据 V5 要这一对证据（来源：control/design_lqr.m:13-15）。

### 2.5 积分状态的限幅

限幅不在 `design_lqr.m` 里，在参数文件与推入脚本里。

`P.ctrl.int_limit` 是误差域界：`phi = deg2rad(15)`、`theta = deg2rad(10)`、`V = 5.0`、`h = 20.0`（来源：params/uacs_twin_params.m:479-480）。

推给 LQR 积分状态的两个界（来源：sim/sim_push_design_vars.m:137-140）：

- 纵向 `sim_lqr_zlim_long = P.ctrl.int_limit.V = 5.0`，负向 `-5.0`
- 横航向 `sim_lqr_zlim_lat = P.ctrl.int_limit.phi = deg2rad(15) = 0.261799`，负向取负

模型里这两个界接到积分器的 `UpperSaturationLimit` 与 `LowerSaturationLimit`（来源：sim/sim_build_lqr.m:84-85，同一接线在 sim/sim_bench_lqr.m:92-93）。

另有一套状态域界 `P.ctrl.int_state_limit`，按 `margin * int_limit / bw` 算，`margin = 8.0`（来源：params/uacs_twin_params.m:499-504）。这一套是 PID 侧用的，推入脚本给 PID 用它（来源：sim/sim_push_design_vars.m:91-99），LQR 的 z 限幅没有用它。

## 3. 控制律的完整形式

律的字符串直接存在增益结构体里（来源：control/design_lqr.m:105-106，横航向 control/design_lqr.m:163-164）：

```
u_dev = u_ff + K*([x_ref; 0] - [x_dev; z])
x_dev = x_abs - x_op
z     = integral(C_int*x_dev - r)
u_abs(u_idx_full) = u_star_full(u_idx_full) + u_dev
```

逐项（来源：control/design_lqr.m:99-106）：

- `x_op` 是子模型的绝对工作点，纵向 `[V0; a0; x_star(5); x_star(8)]`，横航向 `[0; x_star(4); x_star(6); x_star(7)]`（来源：control/lin_contract.m:64-65）。
- `u_idx_full` 纵向 `[2 6]`（`de`、`dT`），横航向 `[1 3]`（`da`、`dr`），指的是 6 维被控对象输入里的位置（来源：control/design_lqr.m:101、control/design_lqr.m:160）。
- `u_star_full` 是 6x1 的绝对配平输入（来源：control/design_lqr.m:102，来源字段 control/lin_contract.m:67）。
- 被控对象第 6 路输入 `dT` 在模型里恒零，推力实际走两路油门（来源：sim/sim_alloc_sfun.m:10-12，接线证据 sim/sim_bench_mix.m:80-82）。

### 3.1 前馈 Uff 怎么来的

`u_ff` 与 `x_ref` 一起由同一个方程解出，方程是 `A*x_ref + B*u_ff = 0`，即参考点必须是被设计模型自身的平衡点（来源：control/lqr_reference.m:11-13）。

纵向把 `[dalpha; dq; dde; dT]` 当未知量，`dV` 与 `dtheta` 当已知指令（来源：control/lqr_reference.m:33-38）：

```
Ml = [A(:,2), A(:,3), B(:,1), B(:,2)]       4x4
rl = -(A(:,1)*dV + A(:,4)*dth)
yl = Ml \ rl
x_ref_l = [dV; yl(1); yl(2); dth]
u_ff_l  = [yl(3); yl(4)]
```

横航向把 `[dp; dr; dda; ddr]` 当未知量，`dbeta` 与 `dphi` 当已知指令（来源：control/lqr_reference.m:46-50）。

残差 `norm(A*x_ref + B*u_ff)` 作为可达性判据，阈值 `1e-9`（来源：control/lqr_reference.m:20-22、control/lqr_reference.m:84-87）。

设计点自身代入结果是 `x_ref = 0`、`u_ff = 0`，因为雅可比取在配平点上，偏差坐标下前馈按构造为零；唯一非零的前馈是绝对执行机构量 `u_star`（来源：control/lqr_reference.m:15-18）。

命令默认值全为 0，字段名 `dV`、`dtheta`、`dphi`、`dbeta`（来源：control/lqr_reference.m:6-9、control/lqr_reference.m:93-99）。

绝对执行机构输出由配平值加偏差前馈得到：`u_abs = u_star_full`，再把 `u_idx_full` 那两路换成配平值加前馈（来源：control/lqr_reference.m:58-60，横航向 control/lqr_reference.m:67-69）。

工程上验证的可达性条件数也一并返回：`cond(Ml)` 与 `cond(Mt)`（来源：control/lqr_reference.m:88-89）。

### 3.2 参考映射 Tref 解的是什么方程

`Tref` 不是另一个方程。它把 `A*x_ref + B*u_ff = 0` 对指令的线性关系预先解成矩阵，避免模型里放求解器（来源：sim/sim_lqr_refmap.m:11-16）。

做法：在四个单位指令上各解一次，拼成两张矩阵（来源：sim/sim_lqr_refmap.m:20-33）。

- `R.long.Tref` 4x2，输入 `[dV_c; dtheta_c]`，输出 `x_ref_long`
- `R.long.Uff` 2x2，输入同上，输出 `u_ff_long` 即 `[dde; dT]`
- `R.lat.Tref` 4x2，输入 `[dbeta_c; dphi_c]`，输出 `x_ref_lat`
- `R.lat.Uff` 2x2，输入同上，输出 `u_ff_lat` 即 `[dda; ddr]`

线性度自检：在非单位指令 `dV = 0.37`、`dtheta = -0.21`、`dphi = 0.13`、`dbeta = -0.07` 上复算，与矩阵映射的结果逐项比，差值存 `R.linearity`（来源：sim/sim_lqr_refmap.m:35-48）。

`lqr_reference` 不在路径上时，这里用同一组方程自己解一遍（来源：sim/sim_lqr_refmap.m:65-83）。

两张矩阵推入 base 工作区的名字是 `sim_lqr_Tref_long`、`sim_lqr_Uff_long`、`sim_lqr_Tref_lat`、`sim_lqr_Uff_lat`（来源：sim/sim_push_design_vars.m:142-146），模型里用 `Matrix(K*u)` 的 Gain 还原（来源：sim/sim_build_lqr.m:54-63）。

线性模型的参考由 `sim_lqr_ref_ts` 时变表给（来源：sim/sim_build_lqr.m:33），由场景脚本推入（来源：sim/sim_push_scenario.m:84）。

### 3.3 增益矩阵的维数

`gains.long.K` 为 2x5，`gains.lat.K` 为 2x5（来源：docs/architecture.md:366-367，实现 control/design_lqr.m:62、control/design_lqr.m:124）。

### 3.4 自检项

- 与 `care` 解对比：`K_care = R \ (B_aug' * X)`，差值存 `dK_care`（来源：control/design_lqr.m:66-67、control/design_lqr.m:86；横航向 control/design_lqr.m:127-128、control/design_lqr.m:147）。
- Riccati 残差：`A_aug'*X + X*A_aug - X*B_aug*(R\(B_aug'*X)) + Q_aug` 的 Frobenius 范数（来源：control/design_lqr.m:68、control/design_lqr.m:87）。
- 闭环极点 `eig(A_aug - B_aug*K)`，是否全稳定的布尔量 `stable`（来源：control/design_lqr.m:88-90）。
- 逐回路裕度见第 5 节。

### 3.5 稳态误差对照怎么算的

未增广闭环停在 `x_ss = -(A - B*K_unaug) \ (B*K_unaug*x_ref)`，输出取 `e'*x_ss`（来源：control/design_lqr.m:232-235）。

增广闭环解一个线性方程组，把积分状态一起解出（来源：control/design_lqr.m:237-243）：

```
M = [A - B*Kx, -B*Kz; e', 0]
sol = M \ [-B*Kx*x_ref; r]
```

测试量：纵向 `test_l = 2.0` m/s 的阶跃，横航向 `test_t = deg2rad(10)` 的滚转阶跃（来源：control/design_lqr.m:96、control/design_lqr.m:156）。

判据阈值：未增广稳态误差绝对值大于 1e-3，增广后小于 1e-9（来源：control/uacs_job_control_design.m:64-65）。

口径差异：`test_architecture.md` 把增广那条写成「小于指令的 1%」（来源：docs/test_architecture.md:151），代码里用的是绝对 1e-9，两者不是同一个量。

## 4. 控制分配的数学

### 4.1 效力矩阵 G_surf

这是舵面到气动力矩的 3x3 矩阵，用来把舵面偏度合成力矩指令（来源：sim/sim_push_design_vars.m:118-120）：

```
G_surf = [ Sb*Cl_da,  0,          Sb*Cl_dr ;
           0,         Sc*Cm_de,   0        ;
           Sb*Cn_da,  0,          Sb*Cn_dr ]
```

其中 `Sb = qbar_a * S * b`，`Sc = qbar_a * S * cbar`（来源：sim/sim_push_design_vars.m:116-117）。

`qbar_a = 0.5 * P.env.rho_sl * V_a^2`，`V_a = P.op.cruise.V_mps = 25.0` m/s（来源：sim/sim_push_design_vars.m:114-115）。注释把这一项写成 `qbar*S*diag([b cbar b])` 乘以三个舵效导数（来源：sim/sim_push_design_vars.m:23）。

用到的气动导数（来源：params/uacs_twin_params.m:306-329）：

| 导数 | 值 | 含义 |
|---|---|---|
| `Cl_da` | 0.0412 | 副翼滚转操纵效能，1/rad，SIZED |
| `Cl_dr` | 0.0100 | 方向舵引起的滚转，1/rad |
| `Cm_de` | 由 `-CL_de*Vh*(lh/cbar)` 得到 | 升降舵俯仰操纵效能，1/rad，SIZED |
| `Cn_da` | -0.0060 | 副翼不利偏航，1/rad |
| `Cn_dr` | 由 `-Vv*CY_dr*rudder.chord_frac` 得到 | 方向舵偏航操纵效能，1/rad，SIZED |

`G_surf` 对角元符号是 `da +1`、`de -1`、`dr -1`，记录在交接文档里（来源：docs/handover.md:203）。

### 4.2 决策量

分配器的决策量 `v = [da; de; dr; dTdiff]`，`dTdiff = T_r - T_l`，与 `plant/thrust_moment.m` 的定义一致（来源：control/allocate.m:50-51）。设计说明里把它记成 `v = [da; de; dr; dT_rl]`（来源：docs/design_notes_control.md:173）。

### 4.3 含差动推力的 3x4 矩阵 G

```
G = [ Sb*Cl_da, 0,           Sb*Cl_dr, 0       ;   行 1 滚转
      0,        Sc*Cm_de,    0,        0       ;   行 2 俯仰
      Sb*Cn_da, 0,           Sb*Cn_dr, y_arm ]    行 3 偏航
```

来源：control/allocate.m:52-54。行序在注释里写明是「行 1 滚转、行 2 俯仰、行 3 偏航」（来源：control/allocate.m:51）。

几何量（来源：control/allocate.m:47-48）：

- `y_arm = P.prop.y_motor - P.mass.y_cg`，差动推力作用力臂，m
- `z_arm = P.prop.z_thrust_offset`，推力线偏置，m，用于俯仰，不参与 G

`P.prop.y_motor = 0.90` m（来源：params/uacs_twin_params.m:380）。`P.prop.differential_limit = 0.60`（来源：params/uacs_twin_params.m:402）。

`Sb = qbar_a*S*b`、`Sc = qbar_a*S*cbar`，用同一个巡航动压（来源：control/allocate.m:43-46），动压线性化速度为 `P.op.cruise.V_mps`（来源：control/allocate.m:42、control/allocate.m:24-25）。

### 4.4 五个作动器的分配关系与优先级

三段，按注释里的顺序执行（来源：control/allocate.m:14-19）：

1. 升降舵单独解，先于其它：`de_raw = M_cmd(2) / G(2,2)`，随即按 `±de_lim` 夹紧（来源：control/allocate.m:80-81）。俯仰残差不回灌给别的作动器（来源：control/allocate.m:15-16）。
2. 副翼与方向舵为主：滚转与偏航的剩余指令 `M_ry = M_cmd([1 3]) - G([1 3],2)*de`（来源：control/allocate.m:84）。
3. 差动推力做补充，按它相对方向舵的力矩权限加权，只在最后才用（来源：control/allocate.m:17-19）。

### 4.5 求解方法

加权最小范数伪逆，不是普通伪逆（来源：control/allocate.m:103）：

```
e = Wi * An' * pinv(An * Wi * An') * b
```

- `An = G_ry(:, free) * diag(v_lim(free))`，归一化后的效力矩阵（来源：control/allocate.m:101）。
- `Wi = diag(1 ./ p_pri(free).^2)`，优先级的平方倒数做权重（来源：control/allocate.m:102）。
- `b = M_ry - G_ry(:, ~free) * v(~free)`，扣掉已冻结分量的剩余指令（来源：control/allocate.m:100）。
- 解出后乘回限幅：`v_new(free) = v_lim(free) .* e`（来源：control/allocate.m:104-105）。

迭代最多 3 次，命限幅的通道从自由集里摘掉并把值钉在限幅上，再解一次（来源：control/allocate.m:96-114）。

### 4.6 优先级权重的算法

```
n_rud  = Sb * abs(Cn_dr) * dr_lim          方向舵的偏航力矩权限，N*m
n_dif  = abs(y_arm * dT_lim)               差动推力的偏航力矩权限，N*m
p_supp = min(max(n_rud / max(n_dif, eps), 1.0), 1e6)
p_pri  = [1.0; 1.0; p_supp]
```

来源：control/allocate.m:87-90。权重取 1 到 1e6，差动推力越大越被压后（来源：control/allocate.m:86-87、control/allocate.m:89）。

### 4.7 限幅从哪里来

- `da_lim = deg2rad(P.act.aileron.max_deg)`，`de_lim`、`dr_lim` 同理，三处都是 25 度（来源：control/allocate.m:35-37，参数在 params/uacs_twin_params.m:128-141）。
- `tc_min = P.prop.esc.min_throttle = 0.0`，`tc_max = P.prop.esc.max_throttle = 1.0`（来源：control/allocate.m:38-39，参数在 params/uacs_twin_params.m:405-406）。
- `diff_lim = P.prop.differential_limit = 0.60`（来源：control/allocate.m:40）。

### 4.8 油门侧怎么算

总推力命令拆成每台一半：`T_want = 0.5 * T_cmd`（来源：control/allocate.m:69）。

公共油门由权威推进模型反解，反解用二分法 40 次；注释说明 40 次半分裂已经到双精度极限（来源：control/allocate.m:220-244，注释在 control/allocate.m:222-224）。

差动可用量取结构上限与油门余量的较小者（来源：control/allocate.m:210-211）：

```
d_headroom = min(tc - tc_min, tc_max - tc)
d_eff      = min(d_cap, d_headroom)
dT_lim     = T(tc + d_eff) - T(tc - d_eff)
```

差动拆分用带保护的试位法解 `T(tc+dh) - T(tc-dh) = dT`，最多 40 次，收敛判据 `|f| < 1e-11`，整条指令为零时直接返回 0（来源：control/allocate.m:246-282，零值短路在 control/allocate.m:251-254）。

最后合成两路油门并各自夹紧（来源：control/allocate.m:120-124）：

```
dth_l = tc - dth_half
dth_r = tc + dth_half
```

相同（速度，推力）的重复请求走缓存，避免每次都解一次诱导速度（来源：control/allocate.m:182-218）。

### 4.9 饱和了怎么办

- 升降舵与三个解出的量都做硬夹紧（来源：control/allocate.m:81、control/allocate.m:115-117）。
- 每路都有饱和标志 `info.sat`，判据是解出的值达到限幅的 `1+1e-12` 倍；油门另加「求出的油门被夹过」或「差动拆分超过 d_eff」（来源：control/allocate.m:151-155）。
- 汇总标志 `info.sat_any`（来源：control/allocate.m:156）。
- 一旦饱和，残余力矩不重新分配，而是如实报出来：`info.M_err`、`info.M_err_norm`、`info.M_err_rel`、`info.err_pitch_Nm`（来源：control/allocate.m:141-145）。
- 实际达到的力矩用真实油门再算一遍：`T_l`、`T_r` 走权威推力图，`dT_ach = T_r - T_l`，`M_ach = G(:,1:3)*[da;de;dr] + [0;0;y_arm*dT_ach]`（来源：control/allocate.m:130-134）。
- 请求值与原样解出的值分开记：`info.v_cmd = [v(1); de_raw; v(2); v(3)]`，`info.v_ach = [da; de; dr; dT_ach]`（来源：control/allocate.m:146-147）。
- 限幅实际用了哪些值也一并报告，含 `da/de/dr/dT/tc_min/tc_max/d_cap/d_headroom/d_eff`（来源：control/allocate.m:148-150）。
- 俯仰还单独报一个不属于 `M_cmd` 的量：`info.M_thrust_pitch_Nm = z_arm * T_cmd`（来源：control/allocate.m:170）。
- 速率限幅只报告不施加：`info.rate_limit_deg_s` 与 `info.throttle_rate_per_s` 只是从参数搬过来（来源：control/allocate.m:171-173）。分配器本身没有速率积分环节。

### 4.10 仿真侧怎么接

S-Function 两个入端口：`M_cmd` 3x1（N*m）与 `T_cmd` 1x1（N），一个出端口 5x1 `[da de dr dth_l dth_r]`（来源：sim/sim_alloc_sfun.m:5-7、sim/sim_alloc_sfun.m:29-32、sim/sim_alloc_sfun.m:48）。

采样时间取 base 工作区的 `sim_ts_alloc`，即 `P.sim.Ts_alloc = 0.010` s（来源：sim/sim_alloc_sfun.m:37，参数在 params/uacs_twin_params.m:460）。

`sim_alloc_act` 在 `allocate` 在路径上时直接调用它，否则退回到只对三个舵面做普通伪逆的兜底实现（来源：sim/sim_alloc_act.m:12-15，兜底在 sim/sim_alloc_act.m:17-36）。

`sim_alloc_wrap` 返回同一个五元指令并附上 `allocate` 的 `M_ach`、`T_ach`、`M_err_norm`、`sat_any`、`limits`，并用 `info.src` 标明走的是 `allocate` 还是 `placeholder`（来源：sim/sim_alloc_wrap.m:19-29）。

模型里力矩指令的来路：舵面增量加配平舵偏，经 `G_surf` 合成 `M_cmd`，推力指令是 `sim_T_trim` 加 `dT` 增量（来源：sim/sim_bench_mix.m:12-15、sim/sim_bench_mix.m:72-78）。

## 5. 传递函数在项目里出现在哪里

### 5.1 LQR 这条线：没有解析传递函数

结论：`design_lqr.m`、`lqr_reference.m`、`lin_contract.m`、`allocate.m`、`linearize_at.m`、`trim_cruise.m`、`sim_push_design_vars.m`、`sim_lqr_refmap.m`、`sim_alloc_*.m` 这些文件里，没有出现 `tf(`、`zpk(`、`bode`。

`design_lqr.m` 里唯一用到 Control System Toolbox 的地方是逐回路裕度（来源：control/design_lqr.m:201），它把回路做成状态空间再交给 `allmargin`：

```
S = allmargin(ss(A_k, B(:, j), K(j, :), 0))
```

这里 `ss(...)` 只是把 `(A, B, C, D)` 四个矩阵包成一个 LTI 对象，`C` 直接取 `K` 的那一行，`D = 0`（来源：control/design_lqr.m:201）。它不产生解析传递函数。

其余能算作「最接近传递函数」的东西：

- 闭环极点：`eig(A_aug - B_aug*K)` 与 `eig(A - B*K_unaug)`（来源：control/design_lqr.m:88-89、control/design_lqr.m:149-150）。开环极点同样在回路裕度里取一次：`poles = eig(A - B*K)`（来源：control/design_lqr.m:193）。
- 模态表：特征值加频率、阻尼、时间常数（来源：plant/linearize_at.m:88-89、plant/linearize_at.m:132-164）。
- 逐回路裕度：见 5.2。
- 稳态终值：见 3.5。

### 5.2 LQR 的裕度报告

逐回路做法：一次只断开一个输入，另一个回路保持闭环（来源：control/design_lqr.m:186-189、control/design_lqr.m:194-196）：

```
k   = setdiff(1:n_in, j)
A_k = A - B(:, k) * K(k, :)
L_j = ss(A_k, B(:, j), K(j, :), 0)
```

字段有 `loop`、`gm`、`gm_db`、`pm_deg`、`dm_s`、`wcg_rad_s`、`wcp_rad_s`、`loop_stable`、`closed_loop_poles`（来源：control/design_lqr.m:191-192）。

数值取 `allmargin` 返回的 `GainMargin` 最小值、`PhaseMargin` 最小值、`DelayMargin` 最小值，`gm_db = 20*log10(gm)`（来源：control/design_lqr.m:202-213）。默认全部填 `Inf`，交叉频率填 `NaN`（来源：control/design_lqr.m:197-199）。

标签纵向是 `{'de','dT'}`，横航向是 `{'da','dr'}`（来源：control/design_lqr.m:92-93、control/design_lqr.m:153-154）。

增广与未增广各报一套（来源：control/design_lqr.m:92-93）。

（旁注，非源码事实：PID 侧刻意不用 `allmargin`，理由写在 `design_pid.m` 的注释里，来源：control/design_pid.m:996-1000。）

### 5.3 其余全项目范围内的搜索结果

对 `MATLAB` 目录下所有 `.m` 搜 `tf(`、`ss(`、`zpk(`、`bode`、`margin`、`allmargin`、`stepinfo`、`damp`、`pole`、`eig(`：

- `zpk(` 与 `bode(`：整个 `MATLAB` 目录下 0 命中。
- `allmargin`：只有一处，就是 `control/design_lqr.m:201`。
- `margin(` 作为函数调用：0 命中；命中的都是变量名或注释（例如 `int_state_margin`）。
- `damp(` 作为函数调用：0 命中；只在注释里被引用为口径（来源：plant/linearize_at.m:15、plant/linearize_at.m:235、plant/uacs_job_plant_baseline.m:433）。
- `tf(` 与 `ss(`：集中在 PID 这条线，以及 `results/` 下的一次性诊断脚本。清单如下。
  - `control/design_pid.m`：`tf` 用于构造执行机构一阶滞后与控制器多项式（来源：control/design_pid.m:86、control/design_pid.m:238、control/design_pid.m:265、control/design_pid.m:630、control/design_pid.m:937-940），`ss` 用于把对象做成 LTI 对象并把回路串起来（来源：control/design_pid.m:104-106、control/design_pid.m:143、control/design_pid.m:921、control/design_pid.m:926），`pole` 用于闭环极点（来源：control/design_pid.m:631、control/design_pid.m:954、control/design_pid.m:958），`eig` 用于模态频率（来源：control/design_pid.m:851、control/design_pid.m:853、control/design_pid.m:1114）。
  - `results/` 下一次性诊断脚本：`diag_pid_loops.m`、`diag_pid_ss.m`、`diag_pid_full.m`、`diag_pid_math.m`、`diag_pid_regions.m`、`diag_pid_scan.m`、`diag_pid_scan2.m`、`diag_pid_rhp.m`、`diag_pid_resid.m`、`diag_pid_outer.m`、`diag_pid_zero.m`、`diag_q*.m`、`diag_v*.m`、`diag_rate_*.m`、`control/_diag_pid_loops.m` 等，都在用 `tf`/`ss`/`pole`/`freqresp`/`dcgain`/`stepinfo`。
  - `sim/sim_gains_ph.m` 不用 `tf`，只用 `lqr` 直接解，并写死 `C_int` 为 `[1 0 0 0]` 与 `[0 0 0 1]`（来源：sim/sim_gains_ph.m:60-68）。
- `stepinfo`：只在 PID 这条线的诊断脚本里，共 7 处（`control/design_pid.m:1035`、`results/diag_v_loop.m:115`、`results/diag_rate_sweep.m:31`、`results/diag_rate_sweep.m:55`、`results/diag_vh.m:45`、`results/diag_q2.m:33`、`results/diag_q.m:36`、`results/diag_pid_vmax.m:27`）。LQR 这条线一处都没有。
- `eig(`：见 5.1，另有 `plant/uacs_job_plant_baseline.m:122-123`、`sim/sim_lin_ph.m:81`。

对 `MATLAB/docs` 目录下所有 `.md` 搜同一批符号：

- `tf(`、`ss(`、`zpk(`、`bode`、`stepinfo`、`damp(`、`pole(`：0 命中。
- `eig(`：命中 5 处，都在讲线性化验证：`docs/acceptance_report.md:23`、`docs/acceptance_report.md:167`、`docs/acceptance_report.md:334`、`docs/design_notes_plant.md:122`、`docs/test_architecture.md:124`、`docs/test_architecture.md:126`、`docs/test_architecture.md:235`。
- `allmargin`：命中 1 处，就是讲 LQR 逐回路裕度（来源：docs/design_notes_control.md:89）。
- `margin`：另有多处，但都是 `int_state_margin` 这类字段名（来源：docs/handover.md:474、docs/handover.md:810、docs/handover.md:1097、docs/test_architecture.md:211）。
- 注意 `MATLAB/docs/_scratch/pages/` 下有若干 MathWorks 帮助页的网页快照文本，里面出现 `bode`、`Bode`、`Nyquist`（来源：docs/_scratch/pages/www.mathworks.com_help_control_ref_lqr.html.txt:103、docs/_scratch/pages/www.mathworks.com_help_slcontrol_ug_linearize-simulink-model.html.txt:164）。这些是第三方资料快照，不是项目代码。

结论：项目里没有逐通道的解析传递函数。纵向与横航向只有数值线性化得到的 A、B、C、D 四个矩阵（C 是单位阵、D 是零阵），加上由特征值导出的模态表与由 `allmargin` 导出的逐回路裕度。

### 5.4 兜底线性化同样是数值的

`sim_lin_ph.m` 是 `linearize_at` 不可用时的替身，做法相同：中心差分，`hx = 1e-6`、`hu = 1e-6`，步长同样按 `max(1, |x0|)` 缩放（来源：sim/sim_lin_ph.m:11-12、sim/sim_lin_ph.m:22、sim/sim_lin_ph.m:30），并自己做同一套坐标变换（来源：sim/sim_lin_ph.m:36-53）。

它额外算了 `f0 = RHS(x0, u0)` 的模作为配平残差旁证（来源：sim/sim_lin_ph.m:17、sim/sim_lin_ph.m:76-77）。模态表只有频率、阻尼、时间常数，没有名字（来源：sim/sim_lin_ph.m:80-97）。

## 6. 配平是怎么解的

### 6.1 未知量与约束

未知量三个：`z = [alpha; de; throttle]`，两台电机共用一个油门（来源：plant/trim_cruise.m:8、plant/trim_cruise.m:59）。

约束三个：`Fx = 0`、`Fz = 0`、`My = 0`（来源：plant/trim_cruise.m:10）。

固定条件：`beta = da = dr = 0`、`phi = 0`、`theta = alpha + gamma`，平飞时 `theta = alpha`（来源：plant/trim_cruise.m:9-10）。

残差取自 `eom_6dof`，力除以 `m*g`、力矩除以 `qbar*S*cbar` 做归一化，`fsolve` 只看 `[Fx Fz My]` 这三行（来源：plant/trim_cruise.m:11-13、plant/trim_cruise.m:154-158）。

带坡度配平未实现，遇到非零坡度直接断言失败（来源：plant/trim_cruise.m:30）。

### 6.2 初值与求解器

解析初值（来源：plant/trim_cruise.m:39-49）：

```
CL_guess    = m*g*cos(gamma) / (qbar0*S)
alpha_guess = (CL_guess - CL0) / CL_alpha
de_guess    = -(Cm0 + Cm_alpha*alpha_guess) / Cm_de
CD_guess    = CD0 + k_induced*CL_guess^2 + CD_alpha*(alpha_guess - alpha_zl)^2 + CD_de*|de_guess|
T_guess     = qbar0*S*CD_guess + m*g*sin(gamma)
```

油门初值由权威推进模型的二分反解给出（来源：plant/trim_cruise.m:46-49，反解实现 plant/trim_cruise.m:178-200）。

求解器设置（来源：plant/trim_cruise.m:54-57）：

- `fsolve`
- `Algorithm = 'levenberg-marquardt'`
- `FunctionTolerance = 1e-14`
- `StepTolerance = 1e-14`
- `MaxIterations = 200`
- `MaxFunctionEvaluations = 4000`
- `Display = 'off'`

`exitflag <= 0` 时从迎角 5 度、油门夹在 0.05..0.9 重启一次（来源：plant/trim_cruise.m:61-65）。

### 6.3 收敛判据

- 归一化残差最大值 `max(abs(res_norm)) < 1e-6`（来源：plant/trim_cruise.m:139）。
- `exitflag > 0`（来源：plant/trim_cruise.m:143）。
- 迎角落在 2..10 度（冻结判据）或 0..8 度（重标定后的窗口），两套都算并都保留（来源：plant/trim_cruise.m:137-141）。
- 升降舵偏度 `|de_deg| <= 15`（来源：plant/trim_cruise.m:142）。
- 综合判据 `check_all` 与 `check_all_revised`（来源：plant/trim_cruise.m:143-145）。

### 6.4 配平结果的形态

```
x_star = [V*cos(alpha); 0; V*sin(alpha); 0; 0; 0; 0; theta; 0; 0; 0; -h_m]
u_star = [0; de; 0; th; th; 0]
```

来源：plant/trim_cruise.m:72-76。

注意 `x_star(10:12)` 是 `[0; 0; -h_m]`，即高度用向下为正的 `pd` 表示（来源：plant/trim_cruise.m:75）。

`u_star(4) = u_star(5) = th`，第 6 路 `dT` 恒零（来源：plant/trim_cruise.m:76）。

配平结果里还存了密度与动压，用的是被控对象实际用的那套（来源：plant/trim_cruise.m:119-120），并记录 `CL`、`CD`、阻力、升阻比、每台推力、总推力（来源：plant/trim_cruise.m:123-130）。

## 7. 符号约定与单位

- 机体系：x 前、y 右、z 下（来源：params/uacs_twin_params.m:13）。
- 角度 rad、位置 m、姿态 rad、角速率 rad/s、速度 m/s（来源：params/uacs_twin_params.m:14-15）。
- 原点取在机翼气动中心、机身中线上，`x_cg` 为负表示重心在原点之前（来源：params/uacs_twin_params.m:16-17）。
- 气动导数一律按弧度，名字以 `_deg` 结尾的除外（来源：params/uacs_twin_params.m:223）。
- `Cm` 绕机体 y 轴、`Cn` 绕机体 z 轴、`Cl` 绕机体 x 轴（来源：params/uacs_twin_params.m:224-225）。
- 操纵面正负号约定（来源：params/uacs_twin_params.m:227-233）：
  - 升降舵正 = 后缘向下 = 低头力矩
  - 副翼正 = 右副翼向下 = 右翼下沉的滚转力矩
  - 方向舵正 = 后缘向左 = 机头向左的偏航力矩
- 该符号约定由 `init_uacs_twin` 里的断言强制（来源：params/uacs_twin_params.m:232-233）。
- 配平与线性化的状态量是绝对量；LQR 增益工作在偏差坐标，用前必须减去 `x_op`（来源：control/lin_contract.m:62-63、control/design_lqr.m:105-106）。
- 高度用 `pd`（向下为正），所以外部高度是 `-x(12)`（来源：plant/trim_cruise.m:75）。
- 油门是峰值轴功率的比例而不是推力的比例，推力按 `throttle^(4/3)` 走（来源：params/uacs_twin_params.m:368-371）。
- `dT` 是加性对称推力增量，左右各加一半（来源：docs/design_notes_plant.md:67）。
- 差动推力定义为 `T_r - T_l`（来源：control/allocate.m:50-51）。

## 8. 顺带记录的几个约束与旁证（都是源码事实）

- 权重改动必须记录在 `docs/design_notes_control.md`，写清改前值、改后值与理由（来源：control/design_lqr.m:20-21）。
- `weights_unchanged` 与 `weights_source` 两个字段随增益一起返回，标明权重来自 `P.lqr`（来源：control/design_lqr.m:168-169）。
- 增益结构体里存了创建时间、契约映射、线性化步长（来源：control/design_lqr.m:43-45）。
- 横航向增益里没有自己的 `u_star_full` 字段，`u_abs` 是从 `gains.long.u_star_full` 取的（来源：control/lqr_reference.m:67）。推入脚本用的也是 `gl.long.u_star_full`（来源：sim/sim_push_design_vars.m:136）。
- 兜底增益（`control/` 不在路径上时）用 `lqr` 直接解，积分输出写死为纵向 `dV`、横航向 `phi`，并标 `placeholder = true`（来源：sim/sim_gains_ph.m:16、sim/sim_gains_ph.m:60-69）。它只返回 `K`、`A`、`B`、`C_int`、`x_op`、`u_star_full`、`u_idx_full`，没有 `ss_test`、`margins`、`poles` 这些字段（来源：sim/sim_gains_ph.m:64-68）。
- 推入脚本把 `gains`（PID 的那份）也写到 base 工作区（来源：sim/sim_push_design_vars.m:49）。
- 推入脚本是幂等的，并返回写过的变量名清单 `push.written`（来源：sim/sim_push_design_vars.m:27-28、sim/sim_push_design_vars.m:148-151）。
- 兜底来源会被标记：走兜底时 `push.gains_source = 'placeholder'`（来源：sim/sim_push_design_vars.m:152-155）。
- LQR 通道在 Simulink 里没有高度环（来源：docs/handover.md:120）。
- 闭环极点实测值：纵向最右 -0.6108，横航向最右 -1.0601（来源：docs/design_report.md:120，docs/status_verified.md:243）。这是文档记录的实测值，不是本次从源码读出的。
- V5 的验收线：`care` 差小于 1e-6，参考残差小于 1e-9，无增广有稳态误差、增广后消除（来源：control/uacs_job_control_design.m:56-57、control/uacs_job_control_design.m:64-65、control/uacs_job_control_design.m:72-73）。
- V7 的验收线：分配器重构误差相对值小于 1e-9（来源：control/uacs_job_control_design.m:268-269）。
- V8 的验收线：差动推力扫掠相对误差小于 0.01（来源：control/uacs_job_control_design.m:343-344）。
