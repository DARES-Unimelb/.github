# 串级 PID 设计方法取证笔记

范围：`MATLAB\control\design_pid.m`、`MATLAB\control\step_metrics.m`、`MATLAB\sim\sim_add_pid.m`、`MATLAB\sim\sim_pid_coeffs.m`、`MATLAB\control\_diag_pid_loops.m`、`MATLAB\params\uacs_twin_params.m` 的 `ctrl` 段与 `bw` 目标。

写法约定：源码直接读到的数写原值。由源码公式算出的数标「推导值，未实测」。行号是本会话实际读到的行。源码路径一律简写为 `control/…`、`sim/…`、`params/…`。

取证不覆盖：LQR 设计、控制分配内部权重、验证脚本 V6 与 V13 的判据实现、`.slx` 内已保存的块参数快照。这些不在本次阅读范围内。

---

## 0. 术语与文件职责

1. 七个回路固定为 `p`、`q`、`r`、`phi`、`theta`、`V`、`h`，顺序在这一处定义（来源：control/design_pid.m:76）。
2. 结构分层写死：内环 `p`、`q`、`r` 纯比例，外环 `phi`、`theta` 是带滤波微分的 PID 并输出速率指令，最外层 `V` 输出总推力、`h` 输出俯仰角指令（来源：control/design_pid.m:12-15）。
3. 设计入口签名是 `gains = design_pid(P, lin, opts)`，第三参数可省（来源：control/design_pid.m:1）。作业脚本按两参数调用（来源：control/uacs_job_control_design.m:37）。
4. `control/step_metrics.m` 只做阶跃响应度量，不参与增益求解。它给 10 到 90 上升时间、相对稳态值的超调、最后一次离开 2% 带的时刻、对指令的稳态误差（来源：control/step_metrics.m:10-12）。
5. `control/step_metrics.m` 的稳态值取末段 5% 样本的均值（来源：control/step_metrics.m:22-24）。
6. `control/step_metrics.m` 的超调以稳态值为基准、以 `y_ss - y0` 的绝对值为分母（来源：control/step_metrics.m:44、47）。它不被 `design_pid.m` 调用，源码里全仓库只有定义没有调用点。
7. `sim/sim_add_pid.m` 把单个并联 PID 装配成子系统：比例支路一个 `Gain`、积分支路一个 `Discrete-Time Integrator`、微分支路一个 `Discrete Transfer Fcn`，三路进一个三输入求和块（来源：sim/sim_add_pid.m:44-54）。
8. `sim/sim_pid_coeffs.m` 只算离散系数，连续律就是 `design_pid.m` 设计的那一条（来源：sim/sim_pid_coeffs.m:7-8）。
9. `control/_diag_pid_loops.m` 是临时诊断脚本，打印三个速率环与两个姿态环的对象零点与闭环极点结构，不产出增益（来源：control/_diag_pid_loops.m:1-2）。

---

## 1. 七条控制律的原文与逐项增益来源

`gains.law` 里存的七条律原文如下（来源：control/design_pid.m:200-206）：

```
p      da = Kp*(p_c - p) + Kd*Df(p_c - p)
q      de = Kp*(q_c - q) + Kd*Df(q_c - q)
r      dr = Kp*(r_c - r) + Kd*Df(r_c - r)
phi    p_c = Kp*(phi_c - phi) + Ki*i_phi
theta  q_c = Kp*(theta_c - theta) + Ki*i_theta
V      T_cmd = Kp*(V_c - V) + Ki*i_V, throttle through allocate.m
h      theta_c = clamp(Kp*(h_c - h) + Ki*i_h, limits.pitch_max_deg)
```

10. 统一误差口径 `err = cmd - meas`，七环一致（来源：control/design_pid.m:208-209）。
11. 微分项定义 `Df(e) = Kd*s/(1 + Tf*s) e`，即一阶滤波微分（来源：control/design_pid.m:207）。
12. 积分状态的钳位口径原文：钳在 `P.ctrl.int_state_limit.(loop)`，原文说该值是 `P.ctrl.int_limit.(loop)/bw` 再乘 `P.ctrl.int_state_margin`；三个速率环无积分，直接钳在舵面行程（来源：control/design_pid.m:210-212）。注：参数文件里的实际算式是 `margin*int_limit/bw`（来源：params/uacs_twin_params.m:501-504），与本条原文的语序不同但数值一致。
13. 交付字段 `gains.rules.law` 复述同一条律：`u = Kp*e + Ki*int(e) + Kd*Df(e)`（来源：control/design_pid.m:95-96）。
14. `Kp` 的名义值规则是「回路幅值在目标穿越频率处等于 1」，实现为 `Kp_nom = 1/|G(j*wc)|`（来源：control/design_pid.m:864-866）。交付的 `Kp` 不直接等于它，见第 4 节。
15. `Ki` 由「积分零点频率」参数化，网格变量 `frac` 与 `Ki`、`Kp`、`wc` 的关系是 `Ki = Kp*frac*wc`（来源：control/design_pid.m:401）。
16. `Kd` 由「以 Kp/wc 为单位」参数化，关系是 `Kd = Kp*kd_grid/wc`（来源：control/design_pid.m:402、410）。
17. 姿态环的 `Kd` 是给姿态环补阻尼用的，理由是内环是纯比例，姿态环看到的是机体自身阻尼、没有额外阻尼（来源：control/design_pid.m:369-373）。
18. `Kd` 与滤波比例 `kf` 的搜索网格是 `kd_grid = [0, 0.02, 0.05, 0.1, 0.2, 0.4, 0.8]`、`kf_grid = [1, 3]`（来源：control/design_pid.m:387-388）。
19. `Kp`、`Ki`、`Kd`、`kf` 四个量是「网格加一轮细化」，细化档位是 `kp` 九档、`ki` 六档、`kd` 六档、`kf` 两档（来源：control/design_pid.m:418-427）。
20. 一条重要的实现事实：`use_d` 为假时只把 `kd_grid` 置 0、`kf_grid` 置 1，同时把 `Kp` 网格下探到 `1e-4*Kp_nom`、上探到 `1e4*Kp_nom`（来源：control/design_pid.m:379-392）。因此「V 与 h 不开放微分」这句话在源码里对应的是开关，不是把 D 从搜索里去掉。

各环增益取哪个函数（来源：control/design_pid.m:114-155）：

| 回路 | 设计函数 | 对象 | 调用行 |
|---|---|---|---|
| p | `local_rate_loop` | `Gp`，副翼到滚转角速率，串副翼一阶延迟 | 115-116 |
| q | `local_rate_loop` | `Gq`，升降舵到俯仰角速率，串升降舵一阶延迟 | 119-120 |
| r | `local_rate_loop` | `Gr`，方向舵到偏航角速率，串方向舵一阶延迟 | 123-124 |
| phi | `local_pi_loop` | 闭合的 p 环再串一个 1/s 运动学 | 132-133 |
| theta | `local_pi_loop` | 闭合的 q 环再串一个 1/s 运动学 | 137-138 |
| V | `local_pi_loop` | 纵向模型前向速度通道，串 ESC 一阶延迟 | 143-144 |
| h | `local_pi_loop` | 闭合的 theta 环、乘 `V0`、再串一个 1/s | 151-154 |

21. 姿态环的被控对象就是「闭合速率环乘 1/s」，写法与 `model/uacs_manual_pid.slx` 的接法一致（来源：control/design_pid.m:128-130）。
22. 速率环对象的状态空间取法与诊断脚本一致，输出矩阵分别取滚转、俯仰、偏航角速率（来源：control/design_pid.m:104-106，control/_diag_pid_loops.m:15-17）。
23. 高度环对象里 `h_dot = V0*(theta - alpha)` 的 `alpha` 项被有意省略，由作业里的闭环纵向检查覆盖（来源：control/design_pid.m:147-149）。
24. 高度环对象里的 `V0` 取配平纵向状态第一个分量，即配平空速，再乘在闭合 theta 环之后（来源：control/design_pid.m:147-154）。
25. 高度环不带微分，调用时 `use_d` 传 `false`；姿态环与空速环传 `true`（来源：control/design_pid.m:133、138、144、154）。

---

## 2. 输出限幅与权限

26. 速率环输出的限幅值就是舵面行程，单位弧度：`p` 用 `deg2rad(P.act.aileron.max_deg)`，`q` 用 `deg2rad(P.act.elevator.max_deg)`，`r` 用 `deg2rad(P.act.rudder.max_deg)`（来源：control/design_pid.m:115-124）。
27. 三个舵面最大偏度都是 25 度（来源：params/uacs_twin_params.m:128、133、138）。折合 0.436332 rad（推导值，未实测）。
28. 速率环对象里 `gains.p.int_limit` 等字段存的就是上面这个舵面行程值，字段名是 `int_limit`（来源：control/design_pid.m:247）。
29. `h` 环的输出限幅是 `P.limits.pitch_max_deg`，值为 30 度（来源：control/design_pid.m:206，params/uacs_twin_params.m:563）。折合 0.523599 rad（推导值，未实测）。
30. `h` 环的限幅在模型里由 `Saturation` 块实现，上下限变量是 `sim_pid_h_lim` 与 `sim_pid_h_lim_neg`（来源：sim/sim_build_manual_pid.m:174-175）。
31. `V` 环输出总推力指令，不在这两个文件里限幅，由分配环节处理（来源：control/design_pid.m:205）。
32. 姿态环输出速率指令的限幅不在 `design_pid.m` 里，由参数文件的 `P.ctrl.rate_cmd_limit` 给出，见第 5 节。
33. 线性时域校验里的执行机构限幅口径：升降舵、副翼、方向舵各自对 `P.act.*.max_deg`，总推力对两台 ESC 窗口 `0..2*T_static_max`（来源：control/pid_cascade_sim.m:17-19），四个限幅值分别由 `deg2rad(P.act.*.max_deg)` 与 `2*prop_thrust_from_throttle(P, throttle, V0)` 算出（来源：control/pid_cascade_sim.m:49-54）。
34. 线性校验里舵面指令是「配平值加增量」后再限幅，`de_cmd = de_trim + Kq*e`（来源：control/pid_cascade_sim.m:147-148）。
35. 速率环的限幅只出现在 `p_c`、`q_c`、`r_c` 三条指令上，姿态环内部没有别的限幅（来源：control/pid_cascade_sim.m:145、157）。

---

## 3. 抗积分饱和与积分状态限幅

36. 反算增益 `Kb` 直接读 `P.ctrl.anti_windup_Kb`，值为 1.0（来源：control/design_pid.m:84，params/uacs_twin_params.m:478）。
37. `Kb` 同时写进交付结构体的 `gains.rules.anti_windup_Kb`、`gains.anti_windup_Kb` 和每个回路的 `Kb` 字段（来源：control/design_pid.m:95、99、240、267）。
38. 线性校验里的反算实现是：积分器增量等于 `Ts` 乘「本环误差加 `Kb` 乘下游折算系数乘执行机构饱和误差」（来源：control/pid_cascade_sim.m:190-193、195）。
39. 下游折算系数四个：`Kaw_th = 1/(Kq*Ktheta)`、`Kaw_h = 1/(Kq*Kh)`、`Kaw_phi = 1/(Kp_gain*Kphi)`、`Kaw_V = 1/KV`（来源：control/pid_cascade_sim.m:95-98）。
40. 四条积分器更新式（来源：control/pid_cascade_sim.m:195、198、202、206）：
```
i_th  = i_th  + Ts*(e_th  + Kb*Kaw_th *(de_sat - de_cmd))
i_V   = i_V   + Ts*(e_V   + Kb*Kaw_V  *(T_cmd  - T_cmd_raw))
i_h   = i_h   + Ts*(e_h   + Kb*Kaw_h  *(theta_c - theta_c_raw))
i_phi = i_phi + Ts*(e_phi + Kb*Kaw_phi*(da_sat - da_cmd))
```
41. 四式的修正方向由下游回路增益的符号自带，注释写明所以修正永远与造成饱和的误差相反（来源：control/pid_cascade_sim.m:192-193）。
42. 参数文件里的状态限幅算式：`int_state_limit.phi = int_state_margin*int_limit.phi/bw.att_phi_rad_s`，`theta`、`V`、`h` 同式（来源：params/uacs_twin_params.m:501-504）。
43. 状态限幅的四个数值（推导值，未实测，代入 params/uacs_twin_params.m:479-480、499 与 470-476 的数）：`phi` 0.698132、`theta` 0.558505、`V` 50.0、`h` 400.0。
44. 误差域限幅 `P.ctrl.int_limit` 的四个值是 `phi` 15 度、`theta` 10 度、`V` 5.0 m/s、`h` 20.0 m（来源：params/uacs_twin_params.m:479-480）。前两个折合 0.261799 rad 与 0.174533 rad（推导值，未实测）。
45. 状态域与误差域的换算关系原文：两者由回路时间常数 `1/bw` 联系，`int_state_margin` 是状态在钳位生效前可以持有多少个时间常数的极限误差（来源：params/uacs_twin_params.m:483-487）。
46. `int_state_margin` 的值为 8.0（来源：params/uacs_twin_params.m:499）。
47. 参数文件把钳位定位成安全网，反算才是抗饱和机制；margin 取 1 时钳位变成了全部抗饱和，实测代价是 30 度俯仰指令下可达俯仰从 28.47 度降到 26.79 度、调节时间从 4.768 s 升到 5.583 s，且抗饱和开与关的对照打印逐位相同（来源：params/uacs_twin_params.m:489-495）。
48. 同一个 manoeuvre 下反算稳定在无 margin 界的 3.9 倍，取 8 留约 2 倍余量（来源：params/uacs_twin_params.m:496-497）。
49. 线性校验用的状态钳位就是上面四个派生值，可通过 `opts.int_limit_scale` 整体缩放，默认 1.0（来源：control/pid_cascade_sim.m:90-93）。
50. 模型侧不实现反算。PID 子系统里积分支路是 `Discrete-Time Integrator` 加输出限幅（`LimitOutput` 为 `on`），上限变量 `sim_pid_<loop>_Isat`、下限 `sim_pid_<loop>_Isat_neg`（来源：sim/sim_add_pid.m:46-50）。
51. `sim_add_pid.m` 明确写出这个差别：设计里的 `gains.<loop>.Kb` 是反算增益，模型走输出限幅，两者只在饱和期间积分器回退速度上不同（来源：sim/sim_add_pid.m:29-32）。
52. 模型积分钳位取值优先级：先查 `P.ctrl.int_state_limit.(loop)`，没有则回退到速率环的舵面行程，再没有则用 `P.ctrl.int_state_limit.V`（来源：sim/sim_push_design_vars.m:91-97）。
53. 该值要过 `max(abs(ilim), 1e-12)` 的地板，因为离散积分器要求增益严格为正、上限也为正（来源：sim/sim_push_design_vars.m:73-79、98）。
54. `sim_pid_<loop>_Ki_abs` 也被抬到 `max(abs(Ki), 1e-12)`，`Ki = 0` 是合法设计结果（来源：sim/sim_push_design_vars.m:78-79）。

---

## 4. 「稳定性受限的增益回退」的判据

55. 交付结构体里逐环存三个字段：`Kp_nominal`（规则给出的名义值）、`Kp_mag`（实际用的幅值）、`kp_scale`（两者之比）、`gain_reduced`（`Kp_mag` 是否小于 `Kp_nominal`）（来源：control/design_pid.m:240-241）。
56. `kp_nominal` 的定义是 `1/|G(j*wc)|`，即「让回路幅值在目标穿越频率处等于 1」的那个增益（来源：control/design_pid.m:305、864-866）。
57. 速率环触发回退的形式不是逐档下调，而是在几何网格上搜最小代价点。网格范围 `1e-2*Kp_nom` 到 `1e3*Kp_nom`、161 点，加一次 11 点细化（来源：control/design_pid.m:310、332-333）。
58. 速率环的极性也一起搜，`sgn` 取 `[1, -1]` 两值（来源：control/design_pid.m:314）。
59. 速率环的代价函数含一项控制努力惩罚 `1.0*max(0, log(Kp/Kp_nom))^2`，作用是让交付增益落在「满足阻尼与裕度的最小增益」上（来源：control/design_pid.m:321-326）。
60. 速率环的硬门槛是矢量裕度 `m.sm >= 0.60`，达不到直接给代价 1e3（来源：control/design_pid.m:517-520）。
61. 矢量裕度的定义是 `min|1 + C*G(jw)|`，在共用频栅上取（来源：control/design_pid.m:909-915）。
62. 速率环的代价函数全文（来源：control/design_pid.m:537-541）：
```
c = 6.0*e_damp^2 + 4.0*e_sm^2 + 0.20*e_slow_wc^2 + 1.50*e_fast_wc^2
    + 0.05*sum(m.slow_modes)*log10(wc_target/0.1)
e_damp = max(0, 0.70 - m.min_zeta)
e_sm   = max(0, log(0.50 / m.sm))
若 max(real(m.slowest)) > -0.05 则 c 再加 0.5
```
63. 超越目标频率的惩罚不对称：低于目标轻（系数 0.20），高于目标重（系数 1.50），理由是满足阻尼后多余的穿越频率只花执行机构带宽（来源：control/design_pid.m:523-536）。
64. 稳定性判据本身是「闭环极点严格落在左半平面」：`ok = max(real(p)) < -opts.tol_axis`，`tol_axis` 默认 1e-7（来源：control/design_pid.m:635，control/design_pid.m:67-68）。
65. 闭环极点用 `pole(feedback(C*G,1))`，并且先做 `minreal(..., 1e-9)`（来源：control/design_pid.m:630-631）。
66. 容差为什么必须是 1e-9：1e-7 会把 q 环在原点上的实闭环极点消掉，判据于是报稳定，而不做 `minreal` 的 `feedback()` 报 +4.06（来源：control/design_pid.m:620-625）。
67. 落在虚轴上或右半平面的极点一律拒绝，没有豁免；留数占比只作为证据打印，不作为放行理由（来源：control/design_pid.m:49-52、637-645）。
68. 留数占比由部分分式展开计算，逐极点取绝对留数除以总和（来源：control/design_pid.m:732-775）。
69. 若某通道不存在可用的稳定比例增益，`local_search_rate` 返回 `kp = NaN`、`score = Inf`，不抛异常（来源：control/design_pid.m:345-354）。
70. 此时 `gains.<loop>` 按「开口环」交付：`Kp = 0`、`designed = false`、`sign = 1`（来源：control/design_pid.m:233-237）。
71. 开口环的物理原因写在注释里：`Gr` 有右半平面零点 `+0.819±0.463i`，`|Gr|` 在 `w -> 0` 时峰值 2.31，环路两次穿过单位圆；凡是大于 0.433 的增益都不稳定，小于它则环路永不过 1（来源：control/design_pid.m:223-231）。
72. 反过来说姿态环与空速环的 `gain_reduced` 被硬写成 `false`（来源：control/design_pid.m:268）。`kp_scale` 与 `int_reduced` 仍然照实计算（来源：control/design_pid.m:268、270）。
73. 姿态环与空速环的积分回退判据是：`Ki < Kp*frac_nominal*wc`，`frac_nominal` 固定 0.5（来源：control/design_pid.m:270，control/design_pid.m:449）。
74. 姿态环与空速环的稳定性门是断言，找不到稳定解会直接抛 `design_pid:noStableTuning`（来源：control/design_pid.m:441-442）。
75. 两个名义积分零点比例常量：有原点极点的对象取 0.5，没有的取 0.2（来源：control/design_pid.m:97）。
76. 报告里「搜索在哪里让了步」那张表打印 `kp_nominal`、`kp_used`、`scale`、`zf_nom`、`zf_used`、`gain_red`、`int_red` 七列（来源：control/uacs_job_control_design.m:490-503）。
77. 该表的注释解释：scale 小于 1 意味着滚转姿态增益为了让螺旋极点留在左半平面而降低，原因是这一环抵消了螺旋模态所依赖的上反效应（来源：control/uacs_job_control_design.m:504-507）。

---

## 5. 速率指令限幅 `rate_cmd_frac`

78. `ctrl.rate_cmd_frac` 的值是 0.70（来源：params/uacs_twin_params.m:517）。
79. 滚转限幅公式：`rate_cmd_frac * (|Cl_da|*deg2rad(aileron.max_deg)/|Cl_p|) * 2*V/b`（来源：params/uacs_twin_params.m:519-521）。
80. 俯仰限幅公式：`rate_cmd_frac * (|Cm_de|*deg2rad(elevator.max_deg)/|Cm_q|) * 2*V/cbar`（来源：params/uacs_twin_params.m:522-524）。
81. 两个公式的物理含义原文：各自取该通道「满舵稳态角速率」的 70%，取 70% 是为了让回路还有余量去修正（来源：params/uacs_twin_params.m:512-516）。
82. 偏航限幅不用上两式，先置 `NaN`（来源：params/uacs_twin_params.m:525-527），再在限幅段之后填成 `min(g*tan(bank_max)/V_cruise, bw.rate_r_rad_s)`（来源：params/uacs_twin_params.m:573-574）。
83. 偏航那个式子和驾驶杆已用的协调转弯限幅同式，所以限幅不会比杆本身能给的更紧（来源：params/uacs_twin_params.m:570-572）。
84. 偏航限幅的后填是必要的，因为它依赖在限幅段才定义的 `bank_max_deg`（来源：params/uacs_twin_params.m:526、570-571）。
85. 三处限幅的数值（推导值，未实测）：
```
p_lim = 0.70 * 0.682235 = 0.477564 rad/s
q_lim = 0.70 * 1.668329 = 1.167831 rad/s
r_lim = min(9.80665*tan(45deg)/25, 5.0) = 0.392266 rad/s
```
86. 上式的输入值：`Cl_da = 0.0412`（来源：params/uacs_twin_params.m:308），`Cl_p = -CL_alpha/12`（来源：params/uacs_twin_params.m:306），`CL_alpha = 4.65`（来源：params/uacs_twin_params.m:236），`Cm_q = -2*CL_alpha*Vh*(lh/cbar)`（来源：params/uacs_twin_params.m:282），`Cm_de = -CL_de*Vh*(lh/cbar)`（来源：params/uacs_twin_params.m:283），`CL_de = tau_elevator*CL_alpha*Sh/S`（来源：params/uacs_twin_params.m:246）。
87. 几何输入：`b = 3.40`（来源：params/uacs_twin_params.m:152），`AR = 5.00`（来源：params/uacs_twin_params.m:153），`S = b^2/AR`（来源：params/uacs_twin_params.m:155），`cbar = S/b`（来源：params/uacs_twin_params.m:156），`Sh/S = 0.16`（来源：params/uacs_twin_params.m:187），`lh = 1.95`（来源：params/uacs_twin_params.m:188）。由此 `S = 2.312`、`cbar = 0.68`、`Vh = 0.458824`（推导值，未实测）。
88. 工作点输入：巡航 `V = 25.0` m/s、`h = 150` m（来源：params/uacs_twin_params.m:101-102），`g = 9.80665`（来源：params/uacs_twin_params.m:30），`bank_max_deg = 45.0`（来源：params/uacs_twin_params.m:562）。代入第 85 到 87 条的公式，`Cm_q = -12.236419`、`Cm_de = -0.636294`、`Cl_p = -0.3875`，满舵稳态速率 `p_ss = 0.682235`、`q_ss = 1.668329` rad/s（推导值，未实测）。这三个数与仓库里 `results/control_pid_report.txt` 的运行期记录逐位一致。
89. 限幅在模型里的落点是三个 `Saturation` 块 `p_c_lim`、`q_c_lim`、`r_c_lim`，块参数是 `sim_rate_cmd_lim_p`、`sim_rate_cmd_lim_q`、`sim_rate_cmd_lim_r`（来源：sim/sim_build_manual_pid.m:200-205）。
90. 姿态环接到速率环必须穿过限幅器，这样大的姿态误差只能要求机体真能给出的速率，且上报的速率指令是限幅后的值（来源：sim/sim_build_manual_pid.m:257-265）。
91. 三个限幅值由推参器从 `P.ctrl.rate_cmd_limit` 推到基础工作区，模型限幅与参数文件不会各说各话（来源：sim/sim_push_design_vars.m:105-110）。
92. 线性校验读同一字段，理由写在注释里：以前这里是环带宽的十倍、从不生效，于是设计校验验的是一个不含该限幅的控制器；20 度滚转指令会要求 2.07 rad/s 而对 0.478 rad/s 的限幅（来源：control/pid_cascade_sim.m:56-65）。
93. 参数文件的修改理由记录：侧杆到底时滚转环一次要求 1.40 rad/s、持续误差下冲到 17.73 rad/s，而机体只能维持 1.23 rad/s；随后副翼饱和、侧滑到 9 度、飞机翻了（来源：params/uacs_twin_params.m:506-510）。
94. 偏航通道没有外环，它的速率指令被直接限幅而不是经姿态律（来源：control/pid_cascade_sim.m:132-135）。

---

## 6. 微分项的实现细节

95. 连续形式是一阶滤波微分 `Kd*s/(1+Tf*s)`（来源：control/design_pid.m:207），不是纯微分。
96. 基准滤波时间常数 `Tf = P.ctrl.rate_filter_tau_s = 0.010` s（来源：control/design_pid.m:83，params/uacs_twin_params.m:477）。
97. 外环搜索把滤波时间按 `kf` 倍缩放，`kf` 取 1 或 3；交付字段是 `d_filter_tau_s = kf*Tf`（来源：control/design_pid.m:272，control/design_pid.m:388）。
98. `kf` 的下限约束是外环采样时间 `P.sim.Ts_outer`，注释写明 kf 必须不低于它微分才在采样回路上可实现（来源：control/design_pid.m:375-377）。源码里没有对这个下限做断言或截断。
99. 交付字段 `Tf` 存的是缩放后的值（来源：control/design_pid.m:267）。
100. 模型侧离散化方法写明是 Tustin 代换 `s = (2/Ts)*(1 - z^-1)/(1 + z^-1)`（来源：sim/sim_pid_coeffs.m:11）。
101. 离散系数闭式（来源：sim/sim_pid_coeffs.m:12、31-34）：
```
D(z) = [2*Kd/Ts, -2*Kd/Ts] / [1 + 2*Tf/Ts, -1 + 2*Tf/Ts]
两个向量都按分母首项归一化
```
102. 速率环 `Kd = 0` 时 `Dnum` 退化成标量 0、`Dden` 为 1，注释说明这样块就是一个纯零点而不是两抽头相消（来源：sim/sim_pid_coeffs.m:16-17、26-30）。
103. 离散化用的采样时间由调用方传入；`sim_pid_coeffs` 在缺参时从基础工作区取 `sim_ts_ctrl`（来源：sim/sim_pid_coeffs.m:19-21）。
104. 模型里 P 支路是 `Gain`，I 支路是 `Discrete-Time Integrator`，D 支路是一个 `Discrete Transfer Fcn`，三条支路进一个 `+++` 求和块（来源：sim/sim_add_pid.m:44-54）。
105. 三条支路的采样时间都取 `sim_pid_<loop>_Ts`（来源：sim/sim_add_pid.m:47、52）。
106. 极性与幅值分离：三个增益块只装幅值（`Kp_abs`、`Ki_abs`、`Dnum_abs`），极性由求和块之后的 `pol` 增益块一次性施加（来源：sim/sim_add_pid.m:18-27、44-56）。
107. 为什么不能靠给 `gainval` 取负来实现极性：那样只覆盖比例与积分支路，微分支路的分子只由 `Kd` 构造，会漏掉符号（来源：sim/sim_add_pid.m:40-43）。
108. `pol` 的取值是 `sign(gains.<loop>.Kp)`，为 0 时取 1（来源：sim/sim_push_design_vars.m:66-70）。
109. 线性校验里微分滤波的离散化是后向欧拉：状态更新为 `xf = xf + (Ts/Tf)*(e - xf)`，微分支路贡献 `Kd*(e - xf)/Tf`（来源：control/pid_cascade_sim.m:184-187、147、159、162）。模型里同一环节走的是 Tustin（来源：sim/sim_pid_coeffs.m:11），两者不是同一种离散化。
110. 线性校验的控制器周期取 `P.sim.Ts_ctrl`，对象以 `Ts_ctrl/4` 积分，注释称这是 Simulink 模型将使用的仿真方式（来源：control/pid_cascade_sim.m:13-15、38-40）。
111. `step_metrics.m` 没有参与微分项或增益的任何计算，它只被时域校验调用做度量（来源：control/step_metrics.m:1-12）。

---

## 7. 采样时间

112. `sim.Ts_ctrl = 0.0025` s，400 Hz 内环，注释说明与典型陀螺速率匹配（来源：params/uacs_twin_params.m:458）。
113. `sim.Ts_outer = 0.010` s，100 Hz 姿态与能量环（来源：params/uacs_twin_params.m:459）。
114. `sim.Ts_alloc = 0.010` s（来源：params/uacs_twin_params.m:460）。
115. `sim.Ts_sensor = 0.0025` s（来源：params/uacs_twin_params.m:461）。
116. 采样时间分派规则：`p`、`q`、`r` 用 `Ts_ctrl`，`phi`、`theta`、`V`、`h` 用 `Ts_outer`（来源：sim/sim_push_design_vars.m:31-33）。
117. `sim_add_pid.m` 的注释复述同一条分派（来源：sim/sim_add_pid.m:12-14）。
118. 推参器把 `sim_ts_ctrl`、`sim_ts_outer`、`sim_ts_alloc`、`sim_ts_sensor`、`sim_t_end` 一并推到基础工作区（来源：sim/sim_push_design_vars.m:39-43）。
119. 求解器是定步长 `ode4`，另有 `ode45` 作步长无关性交叉校验（来源：params/uacs_twin_params.m:462-463）。

---

## 8. 带宽目标 `P.ctrl.bw.*` 的数值、单位与来源

120. `ctrl.bw.rate_p_rad_s = 12.0`，单位 rad/s，注释为滚转速率环目标穿越频率（来源：params/uacs_twin_params.m:470）。
121. `ctrl.bw.rate_q_rad_s = 8.0`（来源：params/uacs_twin_params.m:471）。
122. `ctrl.bw.rate_r_rad_s = 5.0`（来源：params/uacs_twin_params.m:472）。
123. `ctrl.bw.att_phi_rad_s = 3.0`，注释为滚转姿态环，约为速率环的四分之一（来源：params/uacs_twin_params.m:473）。
124. `ctrl.bw.att_theta_rad_s = 2.5`（来源：params/uacs_twin_params.m:474）。
125. `ctrl.bw.speed_rad_s = 0.8`，注释为空速环最慢（来源：params/uacs_twin_params.m:475）。
126. `ctrl.bw.height_rad_s = 0.4`（来源：params/uacs_twin_params.m:476）。
127. 这七个数是文件里直接给出的常量，源码没有写出它们的推导过程。该段只有 `%% ---------------- controller bandwidth targets ----------------` 一行标题加七行赋值，逐行只有功能注释（来源：params/uacs_twin_params.m:469-476）。
128. 三个速率环目标在 `design_pid.m` 里只作报告对照，不作设计目标。设计目标是另一个量 `wi_inner`（来源：control/design_pid.m:110-112）。
129. 不用 `bw.rate_*` 作内环目标的原因写在文件头：内环目标取自机体结构模态，见 `local_inner_target`（来源：control/design_pid.m:37-39）。
130. `wi_inner` 的推导规则：取纵横两个状态矩阵特征值幅值中大于 1.0 的那些，取最小的一个除以 2，再夹在 10.0 到 20.0 rad/s 之间（来源：control/design_pid.m:851-861）。
131. 规则原文说明：内环要坐在最慢结构模态之下，因为速率环是去阻尼那个模态而不是跑赢它；结构模态指短周期与荷兰滚，长周期与螺旋被排除，因为它们慢于 1 rad/s（来源：control/design_pid.m:841-848）。
132. 二分之一因子与 10 到 20 rad/s 的夹紧带被说明为本机内环放置带；举例短周期 `wn = 24.36`、荷兰滚 `14.38`，较慢者的一半是 7.19，被下限抬到 10（来源：control/design_pid.m:848-850）。
133. 候选集为空时回退到 `wi = 12.0`（来源：control/design_pid.m:856-858）。
134. 其余四个环的设计目标直接取 `wc.att_phi_rad_s`、`wc.att_theta_rad_s`、`wc.speed_rad_s`、`wc.height_rad_s`（来源：control/design_pid.m:133、138、144、154）。
135. 七个目标在参数结构体里的字段路径是 `ctrl.bw.rate_p_rad_s`、`ctrl.bw.rate_q_rad_s`、`ctrl.bw.rate_r_rad_s`、`ctrl.bw.att_phi_rad_s`、`ctrl.bw.att_theta_rad_s`、`ctrl.bw.speed_rad_s`、`ctrl.bw.height_rad_s`（来源：params/uacs_twin_params.m:470-476）。
136. 配置入口 `uacs_go.m` 里没有 `bw` 的对应项，只有 `rate_filter_tau_s`、`anti_windup_Kb`、`int_state_margin`、`rate_cmd_frac` 四项被暴露成可改（来源：params/uacs_apply_config.m:97-100）。
137. 带宽分离判据：外环速度除以内环速度必须小于 0.5（来源：control/design_pid.m:184-187）。
138. 五个分离比的定义是 `phi_over_p`、`theta_over_q`、`V_over_q`、`h_over_theta`、`h_over_V`（来源：control/design_pid.m:169-173）。
139. 分离读数的取值规则：开口环为 `Inf`；有穿越无带宽读穿越；有带宽读带宽（来源：control/design_pid.m:827-839）。
140. 用七个目标算出的分离比目标值（推导值，未实测）：`phi/p = 3.0/10.0 = 0.300`、`theta/q = 0.250`、`V/q = 0.080`、`h/theta = 0.160`、`h/V = 0.500`（公式来源：control/design_pid.m:179-183）。

---

## 9. 符号约定与单位

141. 误差一律 `err = cmd - meas`（来源：control/design_pid.m:208）。
142. 姿态角与姿态指令单位是弧度。`phi_c`、`theta_c` 在参数与杆量映射里都由度经 `deg2rad` 得到（来源：control/pilot_to_command.m:13-14、38-39）。
143. 角速率单位是 rad/s。`cmd.r_c` 的单位注释直接写 rad/s（来源：control/pilot_to_command.m:15）。
144. 舵面行程单位是弧度，设计里由度经 `deg2rad` 转入（来源：control/design_pid.m:116、120、124）。
145. 空速单位是 m/s，高度单位是 m（来源：params/uacs_twin_params.m:479-480）。
146. 油门单位是 0 到 1，窗口由 `prop.esc.min_throttle = 0.0` 与 `prop.esc.max_throttle = 1.0` 给出（来源：params/uacs_twin_params.m:405-406）。
147. 差动油门的结构上限是 `prop.differential_limit = 0.60`（来源：params/uacs_twin_params.m:402）。
148. 推力单位是 N。总推力限幅窗口是 `0` 到 `2*T_static_max`，即两台电机之和（来源：control/pid_cascade_sim.m:52-54）。
149. 时间常数单位是 s：舵机 0.045（来源：params/uacs_twin_params.m:130、135、140），ESC 0.060（来源：params/uacs_twin_params.m:403），微分滤波 0.010（来源：params/uacs_twin_params.m:477）。
150. 积分状态限幅的单位随环不同：姿态环是 rad*s，空速环是 (m/s)*s，高度环是 m*s（来源：params/uacs_twin_params.m:483）。
151. 代价函数里的相位裕度、阻尼比、矢量裕度都是无量纲数，相位单位是度（来源：control/design_pid.m:591-594、696-700）。
152. 频率栅是 `logspace(-4, 4, 600)` rad/s，即每十倍频 75 点（来源：control/design_pid.m:806-818）。
153. `design_pid.m` 里姿态角以弧度进出，只有参数文件与杆量映射在边界上做度到弧度的换算（来源：params/uacs_twin_params.m:479-480、control/pilot_to_command.m:38-39）。

---

## 10. 求解与评判流程的事实

154. 候选控制器的闭环极点用 `pole(feedback(C*G,1))`，`minreal` 容差 1e-9（来源：control/design_pid.m:19、630）。
155. 回路响应在一条固定频栅上按多项式有理函数精确求值（来源：control/design_pid.m:20、653-654）。
156. 穿越频率取 `|C*G|` 首次下穿 1 的频率，用对数频率线性插值（来源：control/design_pid.m:21、697-699、873-884）。
157. 相位裕度取 180 度加该穿越频率处的相位（来源：control/design_pid.m:22、700）。
158. 矢量裕度取 `min|1 + C*G(jw)|`（来源：control/design_pid.m:23、707）。
159. 阻尼取闭环极点的最小阻尼比（来源：control/design_pid.m:24、711-718）。
160. 速率环的判据是阻尼缺口对 0.70、矢量裕度缺口对 0.50、再加控制努力；跟踪环的判据是稳态误差、闭环 -3 dB 带宽对 `P.ctrl.bw`、10 到 90 上升、超调对 8%、尾部纹波（来源：control/design_pid.m:26-29）。
161. 跟踪环代价函数全文（来源：control/design_pid.m:591-595）：
```
pm_work = max(pm_target, 60)
c = 40*e_ss^2 + 0.3*e_slow^2 + 0.02*e_fast^2 + 1.0*e_rise^2
    + 2*e_os + 2*e_rip + 4.0*e_pm^2
    + 0.05*sum(m.slow_modes)*log10(wc_target/0.1)
e_ss   = |1 - dcgain(T)|
e_rise = log(max(rise, realmin) / (2.2/wc_target))
e_pm   = max(0, (pm_work - pm)/pm_work)
超调超过 8% 起罚，超过 20% 起平方项
```
162. 相位裕度的合同下限是 `opts.pm_target_deg`，默认 45 度；工作值取 60 度，理由是设计到下限的回路对线性模型未含的非线性项没有余量（来源：control/design_pid.m:64-65、582-585）。
163. 上升时间目标取 `2.2/wc`，注释说明与 `verify/v_sim_v6_pid.m` 用的是同一个目标（来源：control/design_pid.m:479-481、565）。
164. 速率环的 -3 dB 带宽与阶跃响应被判为退化量，源码给出的实测读数：q 通道 `bandwidth()` 报 75295 rad/s 而 `|L|` 在 8 rad/s 只有 1.5；p 通道阶跃「超调」报 3141%（来源：control/design_pid.m:294-300）。
165. 前一代交付 `Kq = 0.600` 而该通道边界是 0.433，V6 因此测到 66% 俯仰超调（来源：control/design_pid.m:301-303）。
166. 速率环极性的实测依据：开环短周期过阻尼，`Kq = +0.05` 时阻尼比 0.90、`+0.20` 时 0.67，`Kq = -0.05` 时 0.955 且自然频率 44.2（来源：control/design_pid.m:502-506）。
167. 三个速率环的矢量裕度校准点写在注释里：p 为 0.9139，q 为 0.8237，r 为 0.5193；门槛取 0.60 落在 0.53 到 0.82 之间（来源：control/design_pid.m:511-516）。
168. 报告路径的裕度与阶跃指标不用设计自己的频栅，改用 MATLAB 的 LTI 对象；`allmargin()` 被明确弃用，因为它在近相消零点附近报数值噪声引起的穿越（来源：control/design_pid.m:929-933、996-1000）。
169. 相位必须先 `unwrap` 再插值，否则会报出 1.1e6 度与 1.2e154 度的相位裕度（来源：control/design_pid.m:1057-1065）。
170. `local_cl_eval` 是唯一的 PD 候选求值函数，但全仓库没有调用点，实际使用的只有 `local_cl_eval_p`、`local_cl_eval_pid` 与 `local_cl_core`（来源：control/design_pid.m:599-614）。
171. 被控对象多项式的分子系数按 `ss2tf` 原样使用，不去除原点附近的零点；注释说明早期用 `poly()` 重建分子会让 q 通道的双重原点零点造成向量长度不匹配、使每个 q 候选都被判不稳定（来源：control/design_pid.m:790-795）。
172. 被控对象多项式与频响在一次设计内缓存（来源：control/design_pid.m:777-804）。
173. 交付结构体顶层字段包括 `created`、`rules`、`rate_filter_tau_s`、`anti_windup_Kb`、`int_limit`、`x_op`、`polarity`、`law`、`verify`（来源：control/design_pid.m:88-101、189-213）。
174. 逐环极性由 `gains.polarity.(loop) = gains.(loop).sign` 汇总上报（来源：control/design_pid.m:191-197）。
175. 给定 `opts.which` 是七环的子集时，分离比一律置 `NaN`（来源：control/design_pid.m:164-177）。

---

## 11. 本笔记未覆盖与源码里没有找到的项

176. `params/uacs_twin_params.m` 的 `ctrl.bw` 七个数是被直接赋值的常量，源码里没有给出它们的推导过程或引用的设计依据（来源：params/uacs_twin_params.m:469-476）。
177. `design_pid.m` 内没有对 `kf*Tf >= P.sim.Ts_outer` 的断言或截断，只有注释陈述这条下限（来源：control/design_pid.m:375-377）。
178. `sim_add_pid.m` 内没有任何反算抗饱和的实现，该文件只有三条支路加求和加极性增益（来源：sim/sim_add_pid.m:44-61）。
179. `step_metrics.m` 里没有增益、限幅、抗饱和或采样时间的任何内容（来源：control/step_metrics.m:1-55）。
180. 本笔记没有核对 `docs/design_notes_control.md` 与当前源码的一致性；两份文件的若干描述与本笔记记录的行号内容并不相同，该差异属于另一项取证工作。
181. 速率指令信号的来源与去向只在本笔记第 5 节覆盖到「姿态环输出经限幅进速率环」。杆量到姿态与速率指令的映射在 `control/pilot_to_command.m`，不在本次指定阅读清单内，本笔记只引用了它的单位注释（来源：control/pilot_to_command.m:13-15、38-39）。
182. 本笔记没有运行任何 MATLAB 代码，没有读取 `.slx` 内已保存的块参数快照。所有数值来自源码文本，或由源码公式用 Python 复算并在第 85、87、88 条标明为推导值。
