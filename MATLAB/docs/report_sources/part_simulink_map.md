# 台架逐块对照：`uacs_flight_bench.slx`

这份文件只讲一件事：`MATLAB\model\uacs_flight_bench.slx` 这张图上的每一个块是什么、在算什么、参数从哪来。读者假定从没打开过 Simulink。

三个词先说清楚，后面反复用到。

- 块：图上的一个方块。它是 Simulink 里最小的计算单位，比如一个乘法、一个求和、一个常量。
- 子系统：把一堆块装进一个大方块里，外面只看得到它的输入口与输出口。台架有九个子系统，主线就是这九个大方块串起来的。
- 端口：方块边上的小尖角。一个小尖角是一路信号。方块之间的连线在 Simulink 里叫信号线，图上显示成带箭头的实线，本文一律用「接到」描述方向。

来源标注分四种，全文统一。

- 带行号的源码引用，形如（来源：`sim/sim_bench_mix.m:41`）。行号是实际读到的。
- 模型文件判读，形如（来源：`model/uacs_flight_bench.slx`，块 `驾驶员/k_thr`）。这是直接解开 `.slx` 读里面 XML 得到的，不是跑 MATLAB 得到的。
- 用户给定的实测事实，形如（实测）。
- 判不出来的写「未确认」，不猜。

关于块数：顶层 70 个块，全模型 271 个块，注释 3 条（实测）。本文另做了一次独立核对，解开盘上那个 `.slx` 逐文件求和，得到同样的 271 个块与 3 条注释（来源：`model/uacs_flight_bench.slx` 判读）。生成脚本 `sim/sim_build_bench.m` 建完会自查九个子系统、八个控件、三块状态读数、五条画布内示波器、一条兜底示波器是否都在（来源：`sim/sim_build_bench.m:191-208`）。

---

## 1. 一句话看懂整张图

一句话：画布最左边操纵台上的八个控件不接任何线，它们只是把数字写进模型里八个常量块；这些数字从那条主线一路走到飞机模型，飞机算出来的状态再走回来变成右边的表针、示波器和四路记录。

把这条链路按顺序讲一遍，全部用大白话。

1. 你在操纵台上拖三个旋钮、两个滑杆、拨三个开关。
2. 这三个旋钮和两个滑杆写的是一组「杆位」，取值范围 −1 到 +1 或 0 到 1。三个开关写的是三个档位数字。
3. 这八个数字落在八个常量块的字面值上。所谓字面值，就是块参数里写的是 `0.655968` 这样的数字，不是 `sim_stick_thr` 这样的变量名。
4. 驾驶员子系统把四个杆位变成五路指令：滚转指令、俯仰指令、偏航率指令、空速指令、高度指令。
5. 制导与模式子系统按「模式」这个档位决定俯仰指令用哪一个：姿态模式直接用杆位算出来的俯仰角指令，高度模式改用高度环算出来的俯仰角。
6. 加工好的五路指令同时送给 PID 控制器与 LQR 控制器。两个控制器每一步都在算，谁也不停。
7. 控制器选择子系统拿「控制器」这个档位从两路输出里挑一路。挑出来的是一路四元的「舵面与推力增量」。
8. 混合与分配子系统把这四元增量加上配平量，算成三轴力矩指令，再分给五个真实作动器：副翼、升降舵、方向舵、左油门、右油门。
9. 六自由度对象就是飞机本体。它吃进五个作动器位置和一股风，用十二个状态量的微分方程一步一步往前推。
10. 导航与测量子系统把十二个状态量拆成三样东西：给控制器用的九路反馈、给记录用的一份原始状态、给仪表用的七路带单位的读数。
11. 九路反馈回到第 6 步，闭环就合上了。七路读数送到右边十二块圆形仪表、一块失速灯与五条画布内示波器。
12. 每次仿真停下来，记录钩子把四路信号存成文件并打一张表。

文字框图如下，往右缩进表示「送到下一步」。

- 操纵台八个控件，不接一根线，只写数字
  - 俯仰杆、滚转杆、方向舵三个旋钮，写 −1 到 +1 的杆位
  - 速度指令滑杆，写 0 到 1 的杆位，它代表空速指令不是油门
  - 侧风滑杆，写 −10 到 +10 的东向风速
  - 模式、控制器、操纵源三个开关，各写一个 0 或 1 的档位
  - 落点：驾驶员里的 `k_roll`、`k_pitch`、`k_yaw`、`k_thr`、`js_on`，制导与模式里的 `mode_const`，控制器选择里的 `k_sel`，大气与风扰动里的 `wind_e`
- 驾驶员，四个杆位变成五路指令 `cmd`
  - 制导与模式，按模式决定俯仰指令取杆位还是取高度环输出，输出五路参考 `ref`
    - PID 控制器，七个回路串成两级，输出四元增量 `du`
    - LQR 控制器，纵向与横航两个通道各带积分增广，输出同样四元增量 `du`
      - 控制器选择，一个开关在两者之间瞬时切换
        - 混合与分配，加配平、算力矩指令、分给五个作动器，输出六元 `u6` 与五元 `act5`
          - 六自由度对象，十二状态非线性模型
            - 导航与测量，拆成九路反馈与七路仪表读数
              - 九路反馈回到驾驶员之后的第一个求和点，闭环成立
              - 七路读数送到圆形仪表、失速灯、状态读数与画布内示波器
              - 四路记录在仿真停止时落盘

---

## 2. 顶层信号流

### 2.1 主线一条

顶层只有一条主线，顺序是驾驶员、制导与模式、两个控制器、控制器选择、混合与分配、六自由度对象、导航与测量，最后回到驾驶员的下游（来源：`sim/sim_build_bench.m:63-74`）。大气与风扰动不在主线上，它单独给六自由度对象送第二路输入（来源：`sim/sim_build_bench.m:73`）。

### 2.2 每一根线是什么

下表按接线语句的顺序列出顶层每一段连接。维数是这一路信号里装了几个数，单位按分量给出。

| 从 | 到 | 信号与维数 | 单位 | 来源 |
|---|---|---|---|---|
| `驾驶员/1` | `制导与模式/1` | `cmd` 五元：滚转指令、俯仰指令、偏航率指令、空速指令、高度指令 | 依次为 rad、rad、rad/s、m/s、m | `sim/sim_build_bench.m:66`、`sim/sim_bench_pilot.m:4` |
| `驾驶员/1` | `cmd/1` | 同一路 `cmd`，接到顶层第 2 路输出口当观测点 | 同上 | `sim/sim_build_bench.m:65-66` |
| `驾驶员/2` | `操纵源读数/1` | `state` 一元，操纵源档位 | 无量纲，0 虚拟杆、1 手柄 | `sim/sim_bench_pilot.m:70-73`、`sim/sim_bench_cockpit.m:61` |
| `导航与测量/1` | `制导与模式/2` | `fb` 九元：滚转、俯仰、p、q、r、空速、高度、迎角、侧滑角 | 依次为 rad、rad、rad/s、rad/s、rad/s、m/s、m、rad、rad | `sim/sim_build_bench.m:67`、`sim/sim_bench_nav.m:4-7` |
| `导航与测量/1` | `PID控制器/2` | 同一路 `fb` | 同上 | `sim/sim_build_bench.m:68` |
| `导航与测量/1` | `LQR控制器/2` | 同一路 `fb` | 同上 | `sim/sim_build_bench.m:68` |
| `制导与模式/1` | `PID控制器/1` | `ref` 五元，分量次序与 `cmd` 相同 | 同 `cmd` | `sim/sim_build_bench.m:69` |
| `制导与模式/1` | `LQR控制器/1` | 同一路 `ref` | 同上 | `sim/sim_build_bench.m:69` |
| `PID控制器/1` | `控制器选择/1` | `du` 四元：副翼增量、升降舵增量、方向舵增量、推力增量 | 前三个 rad，第四个 N | `sim/sim_build_bench.m:70`、`sim/sim_bench_pid.m:4-6` |
| `LQR控制器/1` | `控制器选择/2` | 同一形状的 `du` 四元 | 同上 | `sim/sim_build_bench.m:70`、`sim/sim_bench_lqr.m:4-6` |
| `控制器选择/1` | `混合与分配/1` | 选中的 `du` 四元 | 同上 | `sim/sim_build_bench.m:71` |
| `混合与分配/1` | `六自由度对象/1` | `u6` 六元：三个舵面 rad、左右油门 0 到 1、第六路直接推力增量恒零 | 前三个 rad，第四第五无量纲，第六 N | `sim/sim_build_bench.m:72`、`sim/sim_bench_mix.m:4-9` |
| `大气与风扰动/1` | `六自由度对象/2` | `wind_ned` 三元：北向、东向、地向风速 | m/s | `sim/sim_build_bench.m:73`、`sim/sim_bench_wind.m:4` |
| `六自由度对象/1` | `导航与测量/1` | `x` 十二元状态 | 前三元 m/s，第四到第六 rad/s，第七到第九 rad，第十到第十二 m | `sim/sim_build_bench.m:74`、`plant/flow_state.m:28-36`、`plant/eom_6dof.m:28-30` |
| `六自由度对象/1` | `x/1` | 同一路十二元状态，顶层第 1 路输出口 | 同上 | `sim/sim_build_bench.m:74` |

除主线之外，还有几路只给仪表与记录用的分接，它们不进闭环。

| 从 | 到 | 信号与维数 | 单位 | 来源 |
|---|---|---|---|---|
| `PID控制器/2` | `rc_split` | `rate_cmd` 三元：p 指令、q 指令、r 指令 | rad/s | `sim/sim_bench_pid.m:66`、`sim/sim_bench_cockpit.m:72-74` |
| `导航与测量/3` | `inst_split` | `inst` 七元：空速、高度、迎角、滚转、俯仰、失速标志、偏航率 | m/s、m、deg、deg、deg、0 或 1、deg/s | `sim/sim_bench_nav.m:11-15`、`sim/sim_bench_cockpit.m:78-86` |
| `混合与分配/2` | `act_split2` | `act5` 五元：副翼、升降舵、方向舵、左油门、右油门 | 前三个 rad，后两个 0 到 1 | `sim/sim_bench_mix.m:53`、`sim/sim_bench_cockpit.m:139-141` |
| `混合与分配/3` | `dr_deg/1` | 方向舵偏度，顶层第 13 路输出口 | deg，右舵为正 | `sim/sim_bench_mix.m:65-67`、`sim/sim_bench_cockpit.m:156-157` |
| `混合与分配/4` | `da_deg/1` | 副翼偏度，顶层第 14 路输出口 | deg | `sim/sim_bench_mix.m:59-61`、`sim/sim_bench_cockpit.m:158-159` |
| `混合与分配/5` | `de_deg/1` | 升降舵偏度，顶层第 15 路输出口 | deg | `sim/sim_bench_mix.m:62-64`、`sim/sim_bench_cockpit.m:160-161` |

顶层一共 19 路输出口，编号 1 到 19 与块名一一对应（来源：`model/uacs_flight_bench.slx` 判读，顶层 `Outport` 块）。第 1 路是十二元状态 `x`，第 2 路是五元指令 `cmd`，第 3 到第 9 路是 `inst` 拆开的七路，第 10 到第 12 路是三条跟踪向量，第 13 到第 15 路是三个舵面偏度，第 16 到第 19 路是四条窄向量。第 16 到第 19 路的存在理由见第 5 节：只被仪表绑定、不被任何真正的汇使用的信号，会被 Simulink 的块化简连同上游一起删掉，仪表就收不到数据（来源：`sim/sim_bench_cockpit.m:166-167`）。

### 2.3 四个闭环落在哪几段

四个闭环全部收在子系统的框里，画布上看不到回线。它们在图上的位置如下。

| 闭环 | 落在哪个子系统 | 哪几个块 | 误差怎么算 | 输出去哪 |
|---|---|---|---|---|
| 姿态环 | `PID控制器` | `pid_phi` 与 `pid_theta` | `pid_phi` 吃 `ref` 第 1 路减 `fb` 第 1 路，`pid_theta` 吃 `ref` 第 2 路减 `fb` 第 2 路 | 滚转速率指令经 `p_c_lim`，俯仰速率指令经 `q_c_lim` |
| 速率环 | `PID控制器` | `pid_p`、`pid_q`、`pid_r` | 分别是限幅后的 p 指令减 `fb` 第 3 路、q 指令减第 4 路、r 指令先减第 5 路再限幅 | 分别给副翼、升降舵、方向舵增量 |
| 空速环 | `PID控制器` | `pid_V` | `ref` 第 4 路减 `fb` 第 6 路 | 推力增量 `dT` |
| 高度环 | `制导与模式` | `pid_h` | `cmd` 第 5 路减 `fb` 第 7 路 | 俯仰角指令，经 `theta_lim` 限幅后送到 `theta_switch` |

姿态环与速率环的两级串联来源：`sim/sim_bench_pid.m:10-15`、`sim/sim_bench_pid.m:72-85`。空速环来源：`sim/sim_bench_pid.m:15`、`sim/sim_bench_pid.m:79`。高度环来源：`sim/sim_bench_guidance.m:49-54`、`sim/sim_bench_guidance.m:8-9`。

高度环为什么不在 PID 控制器里：两个控制器都要用它。姿态模式下它只做透传，高度模式下它给出俯仰指令；放进任何一个控制器里，都会让另一个控制器反过来依赖那个块（来源：`sim/sim_bench_guidance.m:16-18`）。LQR 通道里没有高度环，所以高度模式对 LQR 无效（来源：`sim/sim_bench_lqr.m:25-26`）。

两级之间还有一道限幅。姿态环是纯比例律，任何姿态误差都会变成一个速率要求，误差持续就要求越来越大。速率指令限幅器把姿态环的输出卡在机体真能给出的角速率上。滚转杆到底曾经要求 17.73 rad/s，而机体只能维持 1.23 rad/s，副翼饱和、侧滑到 9 度、飞机翻了（来源：`sim/sim_bench_pid.m:17-19`、`params/uacs_twin_params.m:506-517`）。

### 2.4 为什么说两套控制器并联同跑，用一个开关选输出

三处证据。

- 顶层把同一路 `ref` 同时接到 `PID控制器/1` 与 `LQR控制器/1`，把同一路 `fb` 同时接到两者的第 2 路（来源：`sim/sim_build_bench.m:68-69`）。
- 两个控制器的输出各占 `控制器选择` 的一个输入端口，`du_lqr` 进第 1 路，`du_pid` 进第 3 路（来源：`sim/sim_build_bench.m:70`、`sim/sim_bench_select.m:28`）。
- `控制器选择` 里是一个 `Switch` 块，判据写的是 `u2 ~= 0`，选择量接在第 2 路（来源：`sim/sim_bench_select.m:22-23`）。`Switch` 是纯选择器，两路输入每一步都会被算出来，不是「使能子系统」那种会让另一支停摆的块。

这样做的两个后果写在源码注释里：切换是瞬时的，不触发重编译，切过去的那一个积分器已经跟上了误差；代价是两个控制器的积分状态都在走，切换前的饱和历史会被带过去（来源：`sim/sim_bench_select.m:6-8`）。

一个判读上的坑：`rate_log` 里的两组指令线只有 PID 有。LQR 直接给舵面增量，没有「速率指令」这个中间量，切到 LQR 之后那两条指令线不再是当前控制器的意思（来源：`sim/sim_build_bench.m:120-121`、`docs/handover.md:188-191`）。

---

## 3. 每个子系统一张表

九个子系统的表都在这一节。表里「块类型」用 Simulink 库里的原名，「参数来自」写的是这个参数读的是哪个基础工作区变量；写「字面量」的表示构建时就被替换成了数字。

### 3.1 驾驶员

位置 `[860 80 1010 220]`（来源：`sim/sim_build_bench.m:53` 与 `sim/sim_bench_pilot.m:32`）。内部 13 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `k_roll` | `Constant` 常量 | `Value` | 变量 `sim_stick_roll`，构建时与打开模型时都被落成字面量 | 滚转杆位，−1 到 +1 |
| `k_pitch` | `Constant` | `Value` | 变量 `sim_stick_pitch`，同上 | 俯仰杆位，−1 到 +1 |
| `k_yaw` | `Constant` | `Value` | 变量 `sim_stick_yaw`，同上 | 方向舵杆位，−1 到 +1 |
| `k_thr` | `Constant` | `Value` | 变量 `sim_stick_thr`，同上 | 速度指令杆位，0 到 1 |
| `stick_mux` | `Mux` 合路 | `Inputs` 为 4 | 无 | 把四个杆位拼成四元虚拟杆向量，并给这根线起名「0 虚拟操纵杆」 |
| `js` | `Reference`，库块 `aerolibanimutils/Pilot Joystick` | `JoystickID` 为 `None`，`OutputConfiguration` 为 `FourAxis`，`SampleTime` 为 `sim_ts_outer` | 手柄号来自 `uacs_go.m` 第 1.11 节的 `uacs_joystick.id`，采样时间来自 `sim_ts_outer` | 读真实游戏手柄的四路输出：滚转、俯仰、偏航、油门。默认不碰硬件、输出恒零 |
| `js_pitch_sign` | `Gain` 增益 | `Gain` | 变量 `sim_js_pitch_sign` | 给手柄俯仰通道乘一个 +1 或 −1，用来纠正拉杆方向 |
| `js_mux` | `Mux` | `Inputs` 为 4 | 无 | 把手柄四路拼成与虚拟杆同量纲的四元向量 |
| `js_on` | `Constant` | `Value` | 变量 `sim_js_on`，落成字面量 | 操纵源档位，0 虚拟杆、1 手柄 |
| `src_sw` | `Switch` 开关 | `Criteria` 为 `u2 ~= 0`，`Threshold` 为 `sim_c_zero` | 阈值变量 `sim_c_zero` 固定为 0 | 第 2 路非零就输出第 1 路（手柄），否则输出第 3 路（虚拟杆） |
| `pilot` | `Level-2 MATLAB S-Function` | `FunctionName` 为 `sim_pilot_sfun` | 采样时间 `sim_ts_outer` 在函数里读 | 杆位到指令的成形，含包线映射与速率限制 |
| `cmd` | `Out1` 输出口 | 端口 1 | 无 | 送出五元指令 |
| `state` | `Out1` | 端口 2 | 无 | 送出操纵源档位，给顶层那块状态读数 |

职责与契约：输入是四路杆位与一路档位，输出五元指令 `cmd`（滚转指令、俯仰指令、偏航率指令、空速指令、高度指令）与一元档位 `state`（来源：`sim/sim_bench_pilot.m:4`）。真正的成形算法不在模型里，在 `control/pilot_to_command.m`：杆位线性映射到包线，指令还有一道爬升率限制，高度指令是俯仰杆对时间的积分（来源：`control/pilot_to_command.m:38-54`、`control/pilot_to_command.m:68-70`）。控制器读到的永远是这几条指令，杆位的原始值不进控制律（来源：`control/pilot_to_command.m:19-22`）。

### 3.2 制导与模式

位置 `[1060 60 1220 210]`（来源：`sim/sim_build_bench.m:54` 与 `sim/sim_bench_guidance.m:24`）。内部 12 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `cmd` | `In1` 输入口 | 端口 1 | 无 | 收驾驶员送来的五元指令 |
| `fb` | `In1` | 端口 2 | 无 | 收导航与测量送来的九元反馈 |
| `cmd_split` | `Demux` 分路 | `Outputs` 为 5 | 无 | 把五元指令拆成五根单线 |
| `fb_split` | `Demux` | `Outputs` 为 9 | 无 | 把九元反馈拆成九根单线 |
| `mode_const` | `Constant` | `Value` | 变量 `sim_flight_mode`，落成字面量 | 飞行模式档位，0 姿态、1 高度 |
| `h_err` | `Sum` 求和 | `Inputs` 为 `+-` | 无 | 高度指令减实际高度，得到高度误差 |
| `pid_h` | `Subsystem` 子系统 | 内部七个块，参数前缀 `sim_pid_h_` | `params/uacs_twin_params.m` 的设计量，由 `sim/sim_push_design_vars.m` 发布 | 高度环，误差进、俯仰角指令出 |
| `theta_lim` | `Saturation` 饱和 | `UpperLimit` 与 `LowerLimit` | 变量 `sim_pid_h_lim` 与 `sim_pid_h_lim_neg`，等于俯仰包线 ±30 度 | 把高度环的输出卡在俯仰包线内 |
| `theta_switch` | `Switch` | `Criteria` 为 `u2 ~= 0`，`Threshold` 为 `sim_c_zero` | 阈值 `sim_c_zero` | 模式非零就输出高度环给的俯仰指令，否则输出杆位算出来的俯仰指令 |
| `ref_mux` | `Mux` | `Inputs` 为 5 | 无 | 把五路合成参考向量 `ref` |
| `ref` | `Out1` | 端口 1 | 无 | 送出五元参考 |
| `state` | `Out1` | 端口 2 | 无 | 送出模式档位，给顶层那块状态读数 |

职责与契约：输入是五元指令与九元反馈，输出五元参考 `ref`，分量次序与 `cmd` 相同（来源：`sim/sim_bench_guidance.m:6-7`）。这一个方块承担两件事：一是模式选择，二是高度环。姿态模式下 `ref` 就是 `cmd`；高度模式下第二个分量被换成 `pid_h` 的输出，也就是把俯仰杆当爬升率指令用，由高度环把高度慢慢拉到那个值（来源：`sim/sim_bench_guidance.m:8-9`、`sim/sim_build_bench.m:108-109`）。

### 3.3 PID控制器

位置 `[1280 40 1450 210]`（来源：`sim/sim_build_bench.m:55` 与 `sim/sim_bench_pid.m:26`）。顶层 23 个块，每个 `pid_*` 内部再各 7 块，本子系统合计 72 块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `ref` | `In1` | 端口 1 | 无 | 收五元参考 |
| `fb` | `In1` | 端口 2 | 无 | 收九元反馈 |
| `ref_split` | `Demux` | `Outputs` 为 5 | 无 | 拆参考 |
| `fb_split` | `Demux` | `Outputs` 为 9 | 无 | 拆反馈 |
| `phi_err` | `Sum` | `Inputs` 为 `+-` | 无 | 滚转指令减滚转 |
| `theta_err` | `Sum` | `Inputs` 为 `+-` | 无 | 俯仰指令减俯仰 |
| `r_err` | `Sum` | `Inputs` 为 `+-` | 无 | 偏航率指令减偏航率。限幅器串在这一级之后 |
| `V_err` | `Sum` | `Inputs` 为 `+-` | 无 | 空速指令减空速 |
| `pid_phi` | `Subsystem` | 内部参数前缀 `sim_pid_phi_` | 采样时间取 `Ts_outer` | 滚转姿态环 |
| `pid_theta` | `Subsystem` | 前缀 `sim_pid_theta_` | 采样时间取 `Ts_outer` | 俯仰姿态环 |
| `pid_V` | `Subsystem` | 前缀 `sim_pid_V_` | 采样时间取 `Ts_outer` | 空速环 |
| `p_c_lim` | `Saturation` | `UpperLimit` 为 `sim_rate_cmd_lim_p`，`LowerLimit` 取相反数 | 由 `params/uacs_twin_params.m` 从满舵稳态角速率算出来 | 滚转速率指令限幅 |
| `q_c_lim` | `Saturation` | `UpperLimit` 为 `sim_rate_cmd_lim_q` | 同上 | 俯仰速率指令限幅 |
| `r_c_lim` | `Saturation` | `UpperLimit` 为 `sim_rate_cmd_lim_r` | 同上，取协调转弯角速率与偏航带宽的较小值 | 偏航速率限幅 |
| `p_err` | `Sum` | `Inputs` 为 `+-` | 无 | 限幅后的 p 指令减实际 p |
| `q_err` | `Sum` | `Inputs` 为 `+-` | 无 | 限幅后的 q 指令减实际 q |
| `pid_p` | `Subsystem` | 前缀 `sim_pid_p_` | 采样时间取 `Ts_ctrl` | 滚转速率环，输出副翼增量 |
| `pid_q` | `Subsystem` | 前缀 `sim_pid_q_` | 采样时间取 `Ts_ctrl` | 俯仰速率环，输出升降舵增量 |
| `pid_r` | `Subsystem` | 前缀 `sim_pid_r_` | 采样时间取 `Ts_ctrl` | 偏航速率环，输出方向舵增量 |
| `rate_mux` | `Mux` | `Inputs` 为 3 | 无 | 拼速率指令向量 `rate_cmd` |
| `rate_cmd` | `Out1` | 端口 2 | 无 | 送出三元速率指令，供示波器与记录看 |
| `du_mux` | `Mux` | `Inputs` 为 4 | 无 | 拼四元增量 `du` |
| `du` | `Out1` | 端口 1 | 无 | 送出四元增量 |

每个 `pid_*` 内部结构完全相同，只是参数前缀不同。下表以一条回路为例，七条回路照抄（来源：`sim/sim_add_pid.m:34-61`）。

| 内部块 | 块类型 | 关键参数 | 参数来自 | 在算什么 |
|---|---|---|---|---|
| `e` | `In1` | 端口 1 | 无 | 收这一路的误差 |
| `Kp` | `Gain` | `Gain` | 变量 `sim_pid_<回路>_Kp_abs` | 比例项，取增益的绝对值 |
| `Ki` | `Discrete-Time Integrator` 离散积分器 | `gainval` 取 `sim_pid_<回路>_Ki_abs`，`SampleTime` 取 `sim_pid_<回路>_Ts`，`LimitOutput` 为 `on`，上下限取 `sim_pid_<回路>_Isat` 与 `_Isat_neg` | 由 `sim/sim_push_design_vars.m` 发布 | 积分项，输出被限幅，这就是抗积分饱和的做法 |
| `Kd` | `Discrete Transfer Fcn` 离散传递函数 | `Numerator` 取 `sim_pid_<回路>_Dnum_abs`，`Denominator` 取 `sim_pid_<回路>_Dden` | 同上，双线性变换算出来的系数 | 微分项，带一阶滤波 |
| `sum` | `Sum` | `Inputs` 为 `+++` | 无 | 三项相加 |
| `pol` | `Gain` | `Gain` 取 `sim_pid_<回路>_pol` | 同上，等于比例增益的符号 | 整条回路的极性在这里一次性乘上 |
| `u` | `Out1` | 端口 1 | 无 | 送出这一路的输出 |

职责与契约：输入五元参考与九元反馈，输出四元增量 `du` 与三元速率指令 `rate_cmd`（来源：`sim/sim_bench_pid.m:4-7`）。七条回路全部由同一个生成函数搭出来，增益从基础工作区读（来源：`sim/sim_bench_pid.m:9`）。接线与受验模型 `model/uacs_manual_pid.slx` 逐根一致，包括偏航那一路的特殊做法：限幅器串在 r 的误差之后而不是指令之后，所以 `rate_cmd` 的第 3 路取的是未限幅的 `r_c`（来源：`sim/sim_bench_pid.m:21-23`、`sim/sim_bench_pid.m:86-87`）。

极性为什么只用一级增益实现：三条支路的符号需求不一样，而离散积分器只接受正的 `gainval`，所以三个增益块携带幅值，极性统一放在求和之后（来源：`sim/sim_add_pid.m:18-27`、`sim/sim_push_design_vars.m:60-65`）。俯仰姿态环就是需要负极性的一条：它的对象直流增益为负，正升降舵给低头力矩（来源：`sim/sim_add_pid.m:22-24`）。

### 3.4 LQR控制器

位置 `[1280 250 1450 450]`（来源：`sim/sim_build_bench.m:56` 与 `sim/sim_bench_lqr.m:29`）。内部 45 个块。LQR 是线性二次型调节器，一句话解释：它拿一张事先算好的增益表，把状态偏差乘起来直接得到控制量，不用逐回路调比例积分微分。

纵向通道 19 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `long_abs` | `Mux` | 四元 | 无 | 按 `[空速 迎角 q 俯仰]` 次序拼实际状态 |
| `long_dev` | `Sum` | `Inputs` 为 `+-` | 无 | 实际状态减配平工作点，得到偏差 |
| `xop_long` | `Constant` | `Value` | 变量 `sim_lqr_xop_long` | 纵向配平工作点四元 |
| `xop_long_split` | `Demux` | 四路 | 无 | 拆工作点，第 1 路是配平空速，第 4 路是配平俯仰角 |
| `long_cmd` | `Mux` | 二元 | 无 | 拼指令侧的两元：空速指令与俯仰指令 |
| `long_cmd_op` | `Mux` | 二元 | 无 | 拼同一个工作点上的两元 |
| `long_ref` | `Sum` | `Inputs` 为 `+-` | 无 | 指令减工作点，把绝对指令换成偏差指令 |
| `long_ref_split` | `Demux` | 二路 | 无 | 拆出偏差空速指令与偏差俯仰指令 |
| `C_long` | `Gain` | `Gain` 为 `sim_lqr_C_long`，乘法方式 `Matrix(K*u)` | `sim/sim_push_design_vars.m:132` | 从偏差状态里挑出被调输出，纵向挑的是空速 |
| `long_int_err` | `Sum` | `Inputs` 为 `+-` | 无 | 被调输出减它的指令，得到积分器的输入 |
| `long_int` | `Discrete-Time Integrator` | `gainval` 为 `sim_c_one`，`SampleTime` 为 `sim_ts_ctrl`，`LimitOutput` 为 `on`，上下限为 `sim_lqr_zlim_long` 与 `_neg` | 采样时间与限幅都由 `sim/sim_push_design_vars.m` 发布 | 积分状态，用来消掉稳态误差 |
| `long_aug_dev` | `Mux` | 二元 | 无 | 拼 `[偏差状态; 积分状态]` |
| `long_aug_ref` | `Mux` | 二元 | 无 | 拼 `[参考偏差; 0]` |
| `long_err` | `Sum` | `Inputs` 为 `+-` | 无 | 参考减实际，得到增广误差 |
| `K_long` | `Gain` | `Gain` 为 `sim_lqr_K_long`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:130` | 增益矩阵乘增广误差 |
| `Tref_long` | `Gain` | `Gain` 为 `sim_lqr_Tref_long`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:143` | 参考前馈，从偏差指令算控制量 |
| `Uff_long` | `Gain` | `Gain` 为 `sim_lqr_Uff_long`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:144` | 配平前馈 |
| `long_uff_sum` | `Sum` | `Inputs` 为 `++` | 无 | 前馈与反馈相加 |
| `u_long_split` | `Demux` | 二路 | 无 | 拆成升降舵与推力两路 |

`C_long` 挑的是空速，`C_lat` 挑的是滚转，这两条来自设计函数的默认选项（来源：`control/design_lqr.m:33-37`、`control/design_lqr.m:52`、`control/design_lqr.m:114`）。

横航通道 19 个块，与纵向通道结构完全对称，只是状态次序换成 `[侧滑角 p r 滚转]`、被调输出换成滚转（来源：`sim/sim_bench_lqr.m:118-157`）。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `lat_abs` | `Mux` | 四元 | 无 | 按 `[侧滑角 p r 滚转]` 拼实际状态 |
| `lat_dev` | `Sum` | `Inputs` 为 `+-` | 无 | 减配平工作点 |
| `xop_lat` | `Constant` | `Value` | 变量 `sim_lqr_xop_lat` | 横航配平工作点四元 |
| `xop_lat_split` | `Demux` | 四路 | 无 | 拆工作点 |
| `lat_cmd` | `Mux` | 二元 | 无 | 拼指令侧两元：侧滑角指令固定为零，滚转指令来自参考 |
| `lat_cmd_op` | `Mux` | 二元 | 无 | 拼工作点侧两元 |
| `lat_ref` | `Sum` | `Inputs` 为 `+-` | 无 | 换成偏差指令 |
| `lat_ref_split` | `Demux` | 二路 | 无 | 拆出偏差侧滑角指令与偏差滚转指令 |
| `C_lat` | `Gain` | `Gain` 为 `sim_lqr_C_lat`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:133` | 挑出被调输出，横航挑的是滚转 |
| `lat_int_err` | `Sum` | `Inputs` 为 `+-` | 无 | 被调输出减指令 |
| `lat_int` | `Discrete-Time Integrator` | `gainval` 为 `sim_c_one`，`SampleTime` 为 `sim_ts_ctrl`，上下限为 `sim_lqr_zlim_lat` 与 `_neg` | `sim/sim_push_design_vars.m:139-140` | 横航积分状态 |
| `lat_aug_dev` | `Mux` | 二元 | 无 | 拼增广实际量 |
| `lat_aug_ref` | `Mux` | 二元 | 无 | 拼增广参考量 |
| `lat_err` | `Sum` | `Inputs` 为 `+-` | 无 | 增广误差 |
| `K_lat` | `Gain` | `Gain` 为 `sim_lqr_K_lat`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:131` | 增益矩阵乘增广误差 |
| `Tref_lat` | `Gain` | `Gain` 为 `sim_lqr_Tref_lat`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:145` | 参考前馈 |
| `Uff_lat` | `Gain` | `Gain` 为 `sim_lqr_Uff_lat`，`Matrix(K*u)` | `sim/sim_push_design_vars.m:146` | 配平前馈 |
| `lat_uff_sum` | `Sum` | `Inputs` 为 `++` | 无 | 前馈与反馈相加 |
| `u_lat_split` | `Demux` | 二路 | 无 | 拆成副翼与方向舵两路 |

公共块 7 个。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `ref` | `In1` | 端口 1 | 无 | 收五元参考 |
| `fb` | `In1` | 端口 2 | 无 | 收九元反馈 |
| `ref_split` | `Demux` | 五路 | 无 | 拆参考 |
| `fb_split` | `Demux` | 九路 | 无 | 拆反馈 |
| `const_zero` | `Constant` | `Value` | 变量 `sim_c_zero`，固定 0 | 供侧滑角指令与增广参考的第二分量使用 |
| `du_mux` | `Mux` | 四元 | 无 | 按与 PID 相同的次序拼 `[副翼 升降舵 方向舵 推力]` |
| `du` | `Out1` | 端口 1 | 无 | 送出四元增量 |

职责与契约：输入与 PID 完全相同，输出也是四元增量，所以控制器选择只需要一个 `Switch`（来源：`sim/sim_bench_lqr.m:4-6`）。参考量是从 `ref` 现算的：`ref` 是绝对指令，LQR 要的是偏差量，所以每一项各自减掉配平工作点（来源：`sim/sim_bench_lqr.m:13-20`）。工作点与增益来自同一个设计结果，减掉的量与增益的来源是同一个工作点（来源：`sim/sim_bench_lqr.m:19-20`、`sim/sim_push_design_vars.m:134-135`）。

一个需要复核的不对称：PID 的积分限幅取的是状态域参数 `P.ctrl.int_state_limit`，LQR 两个积分器的限幅取的是误差域参数 `P.ctrl.int_limit`（来源：`sim/sim_push_design_vars.m:91-99` 与 `sim/sim_push_design_vars.m:137-140`）。两个域的定义与换算见 `params/uacs_twin_params.m:499-504`。这处不对称是否有意为之，本文判不出来，列入需要复核的清单。

### 3.5 控制器选择

位置 `[1520 60 1670 180]`（来源：`sim/sim_build_bench.m:57` 与 `sim/sim_bench_select.m:15`）。内部 6 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `du_pid` | `In1` | 端口 1，线名标成「0 PID」 | 无 | 收 PID 的输出 |
| `du_lqr` | `In1` | 端口 2，线名标成「1 LQR」 | 无 | 收 LQR 的输出 |
| `k_sel` | `Constant` | `Value` | 变量 `sim_ctrl_sel`，落成字面量 | 控制器档位，0 PID、1 LQR |
| `sw` | `Switch` | `Criteria` 为 `u2 ~= 0`，`Threshold` 为 `sim_c_zero` | 阈值 `sim_c_zero` | 档位非零取第 1 路即 LQR，否则取第 3 路即 PID |
| `du` | `Out1` | 端口 1 | 无 | 送出选中的四元增量 |
| `state` | `Out1` | 端口 2 | 无 | 送出控制器档位，给顶层那块状态读数 |

职责与契约：两路四元增量进，一路四元增量出，外加一路档位。这一个方块是整个台架里最薄的子系统，作用只有一个：让 PID 与 LQR 的对比可以在一段仿真里来回切，而不必停下来改模型（来源：`sim/sim_bench_select.m:6-12`）。

### 3.6 混合与分配

位置 `[1720 40 1890 220]`（来源：`sim/sim_build_bench.m:58` 与 `sim/sim_bench_mix.m:20`）。内部 23 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `du` | `In1` | 端口 1 | 无 | 收四元增量 |
| `du_split` | `Demux` | 四路 | 无 | 拆成三个舵面增量与推力增量 |
| `const_utrim` | `Constant` | `Value` | 变量 `sim_u_trim_surf`，等于配平状态的前三个作动器量 | 配平舵面偏度，单位 rad |
| `utrim_split` | `Demux` | 三路 | 无 | 拆三个配平偏度 |
| `const_Ttrim` | `Constant` | `Value` | 变量 `sim_T_trim` | 配平总推力，单位 N |
| `const_zero` | `Constant` | `Value` | 变量 `sim_c_zero`，固定 0 | 给 `u6` 第六路填零 |
| `da_sum` | `Sum` | `Inputs` 为 `++` | 无 | 副翼增量加配平偏度 |
| `de_sum` | `Sum` | `Inputs` 为 `++` | 无 | 升降舵同理 |
| `dr_sum` | `Sum` | `Inputs` 为 `++` | 无 | 方向舵同理 |
| `surf_mux` | `Mux` | 三元 | 无 | 拼三元舵面向量 |
| `G_gain` | `Gain` | `Gain` 为 `sim_G_surf`，乘法方式 `Matrix(K*u)` | 由动压与舵效导数算出的 3×3 矩阵，见 `sim/sim_push_design_vars.m:113-120` | 舵面偏度换成三轴力矩指令 |
| `T_sum` | `Sum` | `Inputs` 为 `++` | 无 | 推力增量加配平推力 |
| `alloc` | `Level-2 MATLAB S-Function` | `FunctionName` 为 `sim_alloc_sfun` | 采样时间 `sim_ts_alloc` 在函数里读 | 分配器，解力矩与推力指令，返回五个作动器量 |
| `act_split` | `Demux` | 五路 | 无 | 拆五个作动器量 |
| `u6_mux` | `Mux` | 六元 | 无 | 拼六元作动向量 `u6` |
| `act5` | `Out1` | 端口 2 | 无 | 送出五元作动向量，供仪表与记录看 |
| `u6` | `Out1` | 端口 1 | 无 | 送出六元作动向量，直接喂飞机模型 |
| `da_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 副翼偏度换算成度 |
| `de_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 升降舵偏度换算成度 |
| `dr_deg` | `Gain` | `Gain` 为 `-180/pi` | 无 | 方向舵偏度换算成度并翻号，让右舵为正 |
| `da_deg_out` | `Out1` | 端口 4 | 无 | 送出副翼角度给顶层第 14 路输出口 |
| `de_deg_out` | `Out1` | 端口 5 | 无 | 送出升降舵角度给顶层第 15 路输出口 |
| `dr_deg_out` | `Out1` | 端口 3 | 无 | 送出方向舵角度给顶层第 13 路输出口 |

职责与契约：输入四元增量，输出六元作动、五元作动、三个舵面角度（来源：`sim/sim_bench_mix.m:4-5`）。三步走：三个舵面各自加配平偏度，用操纵效能矩阵把偏度换成力矩指令，再由分配器解出力矩指令与推力指令对应的五个作动器量（来源：`sim/sim_bench_mix.m:11-15`）。分配器本身是受验代码，这里不重写，直接调同一个 S-函数（来源：`sim/sim_bench_mix.m:17`）。`u6` 的第六路是飞机模型的直接推力增量入口，台架不用它、恒零，推力走的是两路油门的差动（来源：`sim/sim_bench_mix.m:8-9`、`sim/sim_alloc_sfun.m:11-12`）。

方向舵翻号的道理：气动约定里正舵是后缘向左、机头向左，而操纵台上那个方向舵旋钮向右是机头向右。不翻号，杆向右打而表针向左走（来源：`sim/sim_bench_mix.m:56-58`、`sim/sim_bench_mix.m:65-67`）。三个舵面角度为什么要另开三路输出口：分配器的输出是五元向量，按元素绑定做不到，所以在这里先换算成度再导出（来源：`sim/sim_bench_mix.m:55-56`）。

### 3.7 六自由度对象

位置 `[1960 40 2130 240]`（来源：`sim/sim_build_bench.m:59`）。内部 4 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `u_act` | `In1` | 端口 1 | 无 | 收六元作动向量 |
| `wind_ned` | `In1` | 端口 2 | 无 | 收三元风 |
| `plant_eom` | `Level-2 MATLAB S-Function` | `FunctionName` 为 `sim_plant_sfun`，连续状态 12 个，采样时间设为连续 | 所有数字从基础工作区结构体 `P` 读，初值读 `sim_x0` | 十二状态刚体运动方程 |
| `x_state` | `Out1` | 端口 1 | 无 | 送出十二元状态 |

职责与契约：输入六元作动与三元风，输出十二元状态。十二个状态依次是机体系速度三元、角速率三元、姿态角三元、位置三元，其中位置第三元是向下为正的高度（来源：`plant/flow_state.m:28-36`、`plant/eom_6dof.m:28-30`、`plant/flow_state.m:47-49`）。它是连续状态，所以同一块既能跑定步长 ode4 的主线，也能跑变步长 ode45 的对照（来源：`sim/sim_plant_sfun.m:9-14`）。初值优先取基础工作区变量 `sim_x0`，没有就取配平状态 `trim.x_star`，两者都没有就报错而不是静默从零起（来源：`sim/sim_plant_x0.m:9-16`）。台架里 `sim_x0` 由参数链发布成配平状态（来源：`params/uacs_apply_config.m:47-49`、`sim/sim_push_scenario.m:38`）。

### 3.8 导航与测量

位置 `[1960 300 2120 500]`（来源：`sim/sim_build_bench.m:60` 与 `sim/sim_bench_nav.m:22`）。内部 18 个块。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `x` | `In1` | 端口 1 | 无 | 收十二元状态 |
| `x_split` | `Demux` | 十二路 | 无 | 拆状态 |
| `meas` | `Level-2 MATLAB S-Function` | `FunctionName` 为 `sim_meas_sfun`，连续直通 | 无 | 从状态算大气数据：空速、迎角、侧滑角、动压 |
| `meas_split` | `Demux` | 四路 | 无 | 拆大气数据，第 1 路是空速 |
| `h_neg` | `Gain` | `Gain` 为 `sim_c_neg1`，即 −1 | 无 | 把向下为正的位置翻成向上为正的高度 |
| `fb_mux` | `Mux` | 九元 | 无 | 拼反馈向量 |
| `fb` | `Out1` | 端口 1 | 无 | 送出九元反馈 |
| `x_out` | `Out1` | 端口 2 | 无 | 送出十二元原始状态 |
| `alpha_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 迎角换算成度 |
| `phi_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 滚转换算成度 |
| `theta_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 俯仰换算成度 |
| `r_deg` | `Gain` | `Gain` 为 `180/pi` | 无 | 偏航率换算成度每秒 |
| `alpha_abs` | `Abs` 取绝对值 | 无 | 无 | 迎角取绝对值，失速判据不看正负 |
| `const_stall` | `Constant` | `Value` | 变量 `sim_alpha_stall_abs`，等于失速迎角的弧度绝对值 | 失速门限 |
| `stall_rel` | `Relational Operator` 关系运算 | `Operator` 为 `>`，输出布尔 | 无 | 迎角绝对值超过门限就给真 |
| `stall_dbl` | `Data Type Conversion` | `OutDataTypeStr` 为 `double` | 无 | 布尔转成双精度，否则进合路块会报数据类型不匹配 |
| `inst_mux` | `Mux` | 七元 | 无 | 拼仪表分接向量 |
| `inst` | `Out1` | 端口 3 | 无 | 送出七元仪表读数 |

职责与契约：输入十二元状态，输出九元反馈、十二元原始状态、七元仪表读数（来源：`sim/sim_bench_nav.m:4-5`）。九元反馈的次序在这里定死，下游三个子系统都按这个次序拆：滚转、俯仰、p、q、r、空速、高度、迎角、侧滑角（来源：`sim/sim_bench_nav.m:6-7`）。七元仪表读数的次序是空速、高度、迎角、滚转、俯仰、失速标志、偏航率，角度在这里就换成度，因为仪表刻度按度给（来源：`sim/sim_bench_nav.m:11-16`）。高度取负号的理由：机体系 z 向下，状态向量第 12 路是向下的位置，向上为正要用 −pd（来源：`sim/sim_bench_nav.m:9`）。失速比较用两输入形式而不是块自带的常数阈值，理由是那个阈值的参数名在不同版本里不一样（来源：`sim/sim_bench_nav.m:53-54`）。

### 3.9 大气与风扰动

位置 `[1520 300 1680 460]`（来源：`sim/sim_build_bench.m:61` 与 `sim/sim_bench_wind.m:17`）。内部 8 个块。这个子系统没有输入。

| 块名 | 块类型 | 关键参数 | 参数来自 | 这个块在算什么 |
|---|---|---|---|---|
| `wind_base` | `Constant` | `Value` | 变量 `sim_dist_wind_ned`，三元 | 常值风，地系三分量 |
| `wind_e` | `Constant` | `Value` | 变量 `sim_dist_wind_e`，落成字面量 | 画布上侧风滑杆给的那股东向风 |
| `e_zero` | `Constant` | `Value` | 变量 `sim_c_zero` | 给另外两个方向填零 |
| `e_mux` | `Mux` | 三元 | 无 | 拼成只有东向非零的三元向量 |
| `gust` | `Sine Wave` 正弦 | `Amplitude` 取 `sim_dist_gust_amp`，`Frequency` 取 `sim_dist_gust_freq`，`SampleTime` 为 0 即连续 | 幅度与频率由 `uacs_go.m` 第 1.10 节发布，幅度为零就是关掉 | 阵风标量 |
| `gust_gain` | `Gain` | `Gain` 取 `sim_dist_gust_dir`，乘法方式 `Matrix(K*u)` | 同上，方向是 NED 单位向量 | 标量正弦乘方向向量，得到三元阵风 |
| `wind_sum` | `Sum` | `Inputs` 为 `+++` | 无 | 常值风、侧风微调、阵风三项相加 |
| `wind_ned` | `Out1` | 端口 1 | 无 | 送出三元风 |

职责与契约：没有输入，输出三元风，单位 m/s（来源：`sim/sim_bench_wind.m:4`）。三项全部来自 `uacs_go.m` 第 1.10 节的 `uacs_dist`，再加上画布上那根侧风滑杆（来源：`sim/sim_bench_wind.m:6-11`）。阵风用向量增益而不是逐分量拼，这样方向由参数定，不用改模型（来源：`sim/sim_bench_wind.m:13-14`）。侧风滑杆给的是一股地系东向的常值风，不随飞机姿态转动，也就是说飞机航向一变，这股风相对机体的来向就跟着变（来源：`sim/sim_build_bench.m:155-157`）。

---

## 4. 八个可交互控件

八个控件全部没有端口，一根线都不接。它们是 Dashboard 控件，靠「绑定」工作：控件绑到某个块的某个参数上，拖动它等于改那个参数（来源：`sim/sim_bench_cockpit.m:6-7`）。八个都绑到各自常量块的 `Value` 参数上（来源：`sim/sim_hmi.m:131-134`）。

| 控件名 | 控件类型 | 绑哪个块的哪个参数 | 基础工作区变量名 | 取值范围 | 物理含义 |
|---|---|---|---|---|---|
| `俯仰杆` | 旋钮 `KnobBlock` | `驾驶员/k_pitch` 的 `Value` | `sim_stick_pitch` | −1 到 +1 | 姿态模式下是俯仰角指令，满杆 ±30 度；高度模式下是爬升率指令，满杆 ±6 m/s |
| `滚转杆` | 旋钮 | `驾驶员/k_roll` 的 `Value` | `sim_stick_roll` | −1 到 +1 | 滚转角指令，满杆 ±45 度 |
| `方向舵` | 旋钮 | `驾驶员/k_yaw` 的 `Value` | `sim_stick_yaw` | −1 到 +1 | 偏航率指令，满杆的指令值被夹在协调转弯角速率上，当前参数下约 22.5 度每秒 |
| `速度指令` | 滑杆 `SliderBlock` | `驾驶员/k_thr` 的 `Value` | `sim_stick_thr` | 0 到 1 | 空速指令，不是油门开度。指令空速等于 `V_min` 加杆位乘 `V_max` 减 `V_min` |
| `侧风 东向 mps` | 滑杆 | `大气与风扰动/wind_e` 的 `Value` | `sim_dist_wind_e` | −10 到 +10，单位 m/s | 给地系东向叠一股常值风，不随姿态转动 |
| `模式` | 拨动开关 `ToggleSwitchBlock` | `制导与模式/mode_const` 的 `Value` | `sim_flight_mode` | 姿态 0、高度 1 | 决定俯仰杆是俯仰角指令还是爬升率指令 |
| `控制器` | 旋转开关 `RotarySwitchBlock` | `控制器选择/k_sel` 的 `Value` | `sim_ctrl_sel` | PID 0、LQR 1 | 决定输出用哪套控制器 |
| `操纵源` | 拨动开关 | `驾驶员/js_on` 的 `Value` | `sim_js_on` | 虚拟杆 0、手柄 1 | 决定杆位来自画布还是来自游戏手柄 |

控件的位置、标号与量程来源：`sim/sim_bench_cockpit.m:44-55`。开关的档位文字来源：`sim/sim_hmi.m:71-79`；本机判读 `.slx` 得到的三组档位数据分别是「姿态 高度」「PID LQR」「虚拟杆 手柄」，与源码一致。量程写进块参数（来源：`sim/sim_hmi.m:65-68`），本机判读 `.slx` 得到三个旋钮都是 −1 到 1，侧风滑杆是 −10 到 10，速度指令滑杆是 0 到 1。

表里几个数的出处：俯仰满杆的包线角来自 `uacs_go.m` 第 1.8 节的 `uacs_limits.pitch_max_deg`，滚转满杆的包线角来自同节的 `uacs_limits.bank_max_deg`，爬升率上限来自同节的 `uacs_limits.climb_rate_max_mps`（来源：`uacs_go.m:127-130`）。方向舵满杆的指令角速率取协调转弯角速率与偏航带宽的较小值，当前参数下约 22.5 度每秒（来源：`control/pilot_to_command.m:44-45`、`params/uacs_twin_params.m:570-574`）。

八个控件的变量对应关系写在唯一一张表里，构建时与打开模型时共用这张表（来源：`sim/uacs_bench_seed_list.m:13-22`）。

### 4.1 速度指令滑杆给的是空速指令

这一点单独讲，因为最容易看错。滑杆的读数 0 到 1 不是油门开度，是空速指令的杆位。映射写死在 `control/pilot_to_command.m`：指令空速等于 `V_min` 加杆位乘 `V_max` 减 `V_min`（来源：`control/pilot_to_command.m:53`）。当前参数下 `V_min` 是 15.4665 m/s，`V_max` 是 30 m/s（来源：`docs/handover.md:93-96`）。

- 杆位 0：指令空速 15.4665 m/s。这不是收油门，是让飞机保持这个速度，速度环会一直喂推力把它按住。
- 杆位 0.655968：指令空速约 25.0 m/s，也就是设计配平速度。默认杆位就是这个数（来源：`docs/handover.md:87`）。
- 杆位 1：指令空速 30 m/s。

默认杆位的算法在参数链里：巡航速度减 `V_min` 再除以 `V_max` 减 `V_min`（来源：`params/uacs_apply_config.m:162-164`）。源码注释把结果写成 0.6560，精确值是 0.655968（来源：`params/uacs_apply_config.m:160-161`、`docs/handover.md:87`）。这里要强调：填配平油门 0.4301 是错的，那请求的是 21.7 m/s，飞机会一路减速离开配平（来源：`params/uacs_apply_config.m:158-161`）。

### 4.2 为什么控件必须绑字面量

结论：控件只能驱动「值是字面量」的块参数。写成变量表达式的参数绑不上。

- 绑基础工作区变量是假通路。那样绑能建起来，`isBound` 返回真，控件自己的 `setValue` 也真的改得动那个变量，但仿真运行中改那个变量，仿真完全不理：实测末态俯仰角 0.013 度，与不动一样。绑字面值参数则立刻生效：9.272 度，干预本身只要 0.04 秒（来源：`sim/sim_hmi.m:24-27`）。
- 生成控件的那一段代码带断言，参数是表达式时当场报错，报的是一句「操纵量要先落成字面量再绑定」（来源：`sim/sim_hmi.m:128-130`）。
- 落字面量在装配脚本里做，绑定在操纵台脚本里做，顺序不能反（来源：`sim/sim_build_bench.m:76-79`、`sim/sim_build_bench.m:95`）。
- 初值仍由 `uacs_go.m` 说了算，靠 `uacs_bench_seed` 在打开模型时推一次，两处用同一张对应表（来源：`uacs_bench_seed.m:6-9`、`uacs_bench_seed.m:22-34`）。
- 本机判读 `.slx` 的一个旁证：常量块的值等于该块类型的内置默认值时，Simulink 不把这个参数写进文件。盘上那份模型的 `驾驶员/k_thr` 就没有 `Value` 参数，而 `Value` 的默认值恰好是 1。这不是绑定丢了，同一份文件里速度指令滑杆的绑定数据末尾明确写着 `/k_thr`。

---

## 5. 仪表与示波器

### 5.1 十二块圆表与一块失速灯

圆形仪表绑到信号上，全部是工程单位，量程写进块参数（来源：`sim/sim_bench_cockpit.m:88-108`）。

| 表名 | 绑哪一路信号 | 量程 | 单位 | 来源 |
|---|---|---|---|---|
| `空速表` | `导航与测量/meas_split` 第 1 路 | 0 到 40 | m/s | `sim/sim_bench_cockpit.m:91` |
| `空速指令表` | `ref_split` 第 4 路 | 0 到 40 | m/s | `sim/sim_bench_cockpit.m:92` |
| `高度表` | `导航与测量/h_neg` 第 1 路 | 0 到 400 | m | `sim/sim_bench_cockpit.m:93` |
| `迎角表` | `导航与测量/alpha_deg` 第 1 路 | −15 到 20 | 度 | `sim/sim_bench_cockpit.m:94` |
| `滚转表` | `导航与测量/phi_deg` 第 1 路 | −60 到 60 | 度 | `sim/sim_bench_cockpit.m:95` |
| `滚转指令表` | `ref_split` 第 1 路 | −60 到 60 | 度 | `sim/sim_bench_cockpit.m:96` |
| `俯仰表` | `导航与测量/theta_deg` 第 1 路 | −30 到 30 | 度 | `sim/sim_bench_cockpit.m:97` |
| `俯仰指令表` | `ref_split` 第 2 路 | −30 到 30 | 度 | `sim/sim_bench_cockpit.m:98` |
| `偏航率表` | `导航与测量/r_deg` 第 1 路 | −60 到 60 | 度每秒 | `sim/sim_bench_cockpit.m:99` |
| `方向舵表 右舵正` | `混合与分配/dr_deg` 第 1 路 | −30 到 30 | 度，右舵为正 | `sim/sim_bench_cockpit.m:100` |
| `副翼表 原始偏度` | `混合与分配/da_deg` 第 1 路 | −30 到 30 | 度 | `sim/sim_bench_cockpit.m:101` |
| `升降舵表 原始偏度` | `混合与分配/de_deg` 第 1 路 | −30 到 30 | 度 | `sim/sim_bench_cockpit.m:102` |
| `失速灯` | `导航与测量/stall_dbl` 第 1 路 | 无 | 灯亮表示迎角超过失速迎角 | `sim/sim_bench_cockpit.m:108` |

三块指令表紧跟在对应的实况表右边，这是有意的：跟踪好坏不开窗口也看得出来（来源：`sim/sim_bench_cockpit.m:28-30`）。指令表绑的是 `ref` 的分量，所以在高度模式下俯仰指令表显示的是高度环算出来的那个有效俯仰指令，不是杆位（来源：`sim/sim_bench_cockpit.m:30`）。

注意两处容易误读的地方。副翼表与升降舵表给的是气动偏度原值，正负号与杆位不一定同号（来源：`sim/sim_build_bench.m:126-127`）。高度指令 `h_c` 没有对应的表，十二块表里没有它。

### 5.2 三块状态读数

拨动开关与旋转开关本身不显示当前档位，所以三个开关各配一块状态读数。三块都是经典 `Display` 块，各接一根实线（来源：`sim/sim_bench_cockpit.m:57-61`、`sim/sim_bench_cockpit.m:224-229`）。

| 读数块 | 接哪一路 | 显示什么 | 来源 |
|---|---|---|---|
| `模式读数` | `制导与模式/2` | 0 姿态，1 高度 | `sim/sim_bench_cockpit.m:59` |
| `控制器读数` | `控制器选择/2` | 0 PID，1 LQR | `sim/sim_bench_cockpit.m:60` |
| `操纵源读数` | `驾驶员/2` | 0 虚拟杆，1 手柄 | `sim/sim_bench_cockpit.m:61` |

上一版用的是三盏 Dashboard 灯，实测那三盏灯绑上信号之后运行中全部停在默认灰色，收不到数据，所以换成走端口、一定有数的读数块（来源：`sim/sim_bench_cockpit.m:15-17`）。

### 5.3 五条画布内示波器与一条兜底示波器

五条跟踪图用 `DashboardScope`，画布内出图，不弹窗（来源：`sim/sim_bench_cockpit.m:198-202`）。

| 图名 | 绑哪一路 | 路数 | 这一路是什么 | 来源 |
|---|---|---|---|---|
| `姿态跟踪` | `att_mux` 第 1 路 | 4 | 滚转指令、实际滚转、俯仰指令、实际俯仰 | `sim/sim_bench_cockpit.m:112-121`、`sim/sim_bench_cockpit.m:198` |
| `速率跟踪 pq` | `rate_mux_pq` 第 1 路 | 4 | p 指令、实际 p、q 指令、实际 q | `sim/sim_bench_cockpit.m:168-174`、`sim/sim_bench_cockpit.m:199` |
| `速率跟踪 r 方向舵` | `rate_mux_r` 第 1 路 | 2 | r 指令、实际 r | `sim/sim_bench_cockpit.m:176-181`、`sim/sim_bench_cockpit.m:200` |
| `三个舵面` | `surf_mux` 第 1 路 | 3 | 副翼、升降舵、方向舵 | `sim/sim_bench_cockpit.m:183-189`、`sim/sim_bench_cockpit.m:201` |
| `左右油门` | `thr_mux` 第 1 路 | 2 | 左油门、右油门 | `sim/sim_bench_cockpit.m:191-196`、`sim/sim_bench_cockpit.m:202` |

一条 `DashboardScope` 实测最多画四路：参数逐字相同的三块里，绑四路的出图，绑五路与六路的全空。所以按四路一块拆开（来源：`sim/sim_bench_cockpit.m:164-165`）。被拆窄之后，宽的那些 Mux 仍然留着，继续供给兜底示波器与信号记录（来源：`sim/sim_bench_cockpit.m:165-167`）。

兜底那条是经典 `Scope`，块名 `兜底示波器 三段`，三路输入依次是姿态、速率、舵面与油门三段，上下排成三行，设成不自动弹出，双击才弹（来源：`sim/sim_bench_cockpit.m:205-209`）。保留它的理由：`DashboardScope` 没有端口，数据到不到取决于绑定写对没有，而经典 `Scope` 有输入端口，一定有数据（来源：`docs/handover.md:1482`）。

示波器为什么放在画布内而不是弹窗：`DashboardScope` 是画布内控件，出图不占窗口，不必在几个窗口之间来回切（来源：`sim/sim_bench_cockpit.m:32-35`）。它的绑定按官方契约必须是元胞数组，元素是 `Simulink.HMI.SignalSpecification`，给单个对象会被拒（来源：`sim/sim_bench_cockpit.m:32-33`、`sim/sim_hmi.m:28-29`）。

---

## 6. 四路记录信号

四路信号在模型里开着记录并起了名字，名字不带空格与方括号，方便按名字取（来源：`sim/sim_bench_cockpit.m:211-217`、`sim/sim_bench_cockpit.m:231-239`）。跑完之后基础工作区里有 `logsout`（来源：`docs/handover.md:207-208`）。

| 记录名 | 谁送出来的 | 路数 | 通道定义 | 单位 | 画布上哪块表读的是它 |
|---|---|---|---|---|---|
| `inst_log` | `导航与测量` 第 3 路输出 | 7 | 空速、高度、迎角、滚转、俯仰、失速标志、偏航率 | m/s、m、度、度、度、0 或 1、度每秒 | `空速表`、`高度表`、`迎角表`、`滚转表`、`俯仰表`、`失速灯`、`偏航率表` |
| `att_log` | `att_mux` 第 1 路输出 | 4 | 滚转指令、实际滚转、俯仰指令、实际俯仰 | rad | `姿态跟踪` 那条图 |
| `rate_log` | `rate_mux` 第 1 路输出 | 6 | p 指令、实际 p、q 指令、实际 q、r 指令、实际 r | rad/s | `速率跟踪 pq` 与 `速率跟踪 r 方向舵` 两条图 |
| `act_log` | `act_mux` 第 1 路输出 | 5 | 副翼、升降舵、方向舵、左油门、右油门 | 前三个 rad，后两个 0 到 1 | `三个舵面` 与 `左右油门` 两条图 |

通道次序在源码里定死，改一处不会跟着改另一处（来源：`sim/sim_build_bench.m:136-142`）。报表按这四个名字取数并打表（来源：`verify/bench_report.m:39-42`）。

两处不要混：方向舵的原始偏度在 `act_log` 第三路，也就是舵面偏了多少；方向舵的跟踪是 `rate_log` 的 r 指令对 r，也就是角速度跟没跟上（来源：`sim/sim_build_bench.m:141-142`）。

### 6.1 采样率差异

实测：`act_log` 的采样间隔是 0.01 s，其余三路是 0.0025 s（实测）。机制在于记录点的采样率由该路上最快的源头决定。

- `act_log` 的唯一源头是 `act_mux`，而 `act_mux` 的五路全部来自 `混合与分配` 第 2 路输出，这一路是分配器 S-函数的输出，采样时间取 `Ts_alloc` 即 0.01 s（来源：`sim/sim_bench_cockpit.m:139-146`、`sim/sim_alloc_sfun.m:37`）。
- 另外三路都混进了连续或快节拍的分量：`inst_log` 来自状态向量，`att_log` 与 `rate_log` 各自把指令与实际并进同一个合路块，而实际值来自连续状态（来源：`sim/sim_bench_cockpit.m:112-135`、`sim/sim_bench_nav.m:39-41`）。

对外部脚本的影响有一处旁证：满杆矩阵作业把快信号按 0.005 s 重采样、把作动器信号按 0.01 s 重采样，正是按这两族来分的（来源：`run/uacs_job_bench_stick.m:33-34`、`run/uacs_job_bench_stick.m:157-173`）。报表的时间基准取的是 `inst_log` 的时间向量（来源：`verify/bench_report.m:51`）。四路各自的时间向量在文件里是分开保存的，不要假设它们等长。

仪表与示波器显示侧有没有做滑动平均一类的平滑，源码里没有写，本文未确认。可以确定的是记录侧不做平滑：`bench_report.m` 直接读原始数据。

---

## 7. 采样时间与求解器

### 7.1 四个采样时间

| 名字 | 基础工作区变量 | 数值 | 含义 | 来源 |
|---|---|---|---|---|
| `Ts_ctrl` | `sim_ts_ctrl` | 0.0025 s | 内环采样，同时是定步长求解器的步长 | `params/uacs_twin_params.m:458`、`uacs_go.m:145` |
| `Ts_outer` | `sim_ts_outer` | 0.010 s | 姿态环、空速环、高度环的采样 | `params/uacs_twin_params.m:459`、`uacs_go.m:146` |
| `Ts_alloc` | `sim_ts_alloc` | 0.010 s | 分配器与混合的采样 | `params/uacs_twin_params.m:460`、`uacs_go.m:147` |
| `Ts_sensor` | `sim_ts_sensor` | 0.0025 s | 传感器采样 | `params/uacs_twin_params.m:461`、`uacs_go.m:148` |

四个值都由 `uacs_go.m` 第 1.9 节声明，经参数链映射到参数结构体，再由设计发布器写进基础工作区（来源：`params/uacs_apply_config.m:118-122`、`sim/sim_push_design_vars.m:39-42`）。

### 7.2 哪些块用哪个采样时间

| 采样时间 | 哪些块 | 来源 |
|---|---|---|
| `sim_ts_ctrl` | 求解器步长；LQR 的两个离散积分器 | `sim/sim_new_model.m:31`、`model/uacs_flight_bench.slx` 判读，块 `LQR控制器/long_int` 与 `LQR控制器/lat_int` |
| `sim_ts_outer` | 驾驶员里的手柄块；驾驶员 S-函数；姿态环、空速环、高度环三个 PID 的积分项与微分项 | `model/uacs_flight_bench.slx` 判读，块 `驾驶员/js`；`sim/sim_pilot_sfun.m:33`；`sim/sim_push_design_vars.m:31-33`；七条回路的参数名见 `model/uacs_flight_bench.slx` 判读 |
| `sim_ts_alloc` | 分配器 S-函数 | `sim/sim_alloc_sfun.m:37` |
| `Ts_ctrl` 名义下但按回路细分 | 速率环三条回路的 PID 取 `Ts_ctrl`，姿态环与空速环取 `Ts_outer`，高度环取 `Ts_outer` | `sim/sim_push_design_vars.m:31-33` |
| 连续 | 飞机模型的十二个状态；大气数据 S-函数；阵风正弦 | `sim/sim_plant_sfun.m:44`、`sim/sim_meas_sfun.m:30`、`sim/sim_bench_wind.m:30` |
| `sim_ts_sensor` | 源码里只在发布器出现，本机判读 `.slx` 的全部 XML 里一次都没有出现 | `sim/sim_push_design_vars.m:42` |

`sim_ts_sensor` 与 `sim_ts_ctrl` 数值相同，都是 0.0025 s，所以即使某处改用传感器采样，当前参数下数值上不会看出差别。台架里没有块引用它。

一个必须显式设置采样时间的理由：手柄块的默认采样时间是 1/30 s，与定步长 0.0025 s 不成整数倍，不设的话更新模型时报采样时间不是定步长整数倍的错（来源：`sim/sim_bench_pilot.m:17-19`）。

### 7.3 为什么定步长 ode4 而不是变步长

模型的求解器设置：定步长，`ode4`，步长取 `sim_ts_ctrl`，停止时间取 `sim_bench_t_end`，起始时间 0，代数环报错，求解器参数检查关闭（来源：`model/uacs_flight_bench.slx` 判读，配置集；对应生成代码 `sim/sim_new_model.m:28-39`）。

三条理由。

- 架构约定就是定步长 `ode4`，变步长留给步长无关性的对照检查（来源：`sim/sim_new_model.m:9-11`）。
- 受验模型在定步长与变步长两种求解器下都要跑，而模型引用要求被引用模型与父模型用同一种求解器，所以飞机模型是内嵌的副本而不是模型引用（来源：`sim/sim_add_plant_subsystem.m:5-10`）。
- 台架是被拖着的：定步长让每一步的计算量恒定，拖动控件之后下一个采样点就生效。实测干预在运行中立刻生效（来源：`sim/sim_hmi.m:24-27`）。

速度代价要写在明处：本机实测模型比实时慢 13 到 18 倍（实测）。满杆矩阵作业自己记的是 15 到 18 倍，12 秒一段约 190 秒墙钟（来源：`run/uacs_job_bench_stick.m:8-9`）。源码里有一处旧说法与此不符：`uacs_go.m:144` 的注释写「0.0025 s 步长下每秒约 7 秒真实时间」，也就是 7 倍，偏乐观。以实测为准。

---

## 8. 怎么操作这个台架

从打开到看见曲线，按这个顺序做。

1. 在资源管理器里双击 `MATLAB\model\uacs_flight_bench.slx`。不需要先 `addpath`，也不需要先初始化（来源：`docs/handover.md:51-54`）。
2. 打开时第一个回调跑起来：`PreLoadFcn` 运行 `uacs_go.m`，把参数、配平、增益与默认场景推进基础工作区（来源：`model/uacs_flight_bench.slx` 判读，回调 `PreLoadFcn`；`run/build_bench_models.m:39`）。这条链里包含旋钮到参数的映射、配平、线性化与增益计算（来源：`params/uacs_apply_config.m:38-43`）。
3. 紧接着第二个回调：`PostLoadFcn` 调 `uacs_bench_seed`，按那张唯一对应表把八个操纵量的初值写进模型的字面值参数（来源：`model/uacs_flight_bench.slx` 判读，回调 `PostLoadFcn`；`uacs_bench_seed.m:22-34`）。
4. 按绿色 Run。每次按 Run 之前 `InitFcn` 都会跑 `uacs_ensure`，它只补齐缺失的变量，一个都不缺就什么都不做，所以反复按 Run 不会重跑整条参数链与它几分钟的打印（来源：`model/uacs_flight_bench.slx` 判读，回调 `InitFcn`；`uacs_ensure.m:30-32`、`uacs_ensure.m:4-9`）。
5. 仿真时长看工具栏 StopTime 框，里面是变量 `sim_bench_t_end`，缺省 20 秒（来源：`uacs_go.m:144`、`params/uacs_apply_config.m:181`）。第一次 Run 含编译，比较慢，同一会话里后续 Run 快得多（来源：`docs/handover.md:285`）。
6. 跑的过程中直接拖左边操纵台上的控件，不必停下来（来源：`sim/sim_build_bench.m:97`）。
7. 仿真停下来时 `StopFcn` 调 `uacs_bench_record`，把这一轮归档（来源：`model/uacs_flight_bench.slx` 判读，回调 `StopFcn`）。

### 8.1 记录钩子会产出哪些文件

| 文件 | 内容 | 来源 |
|---|---|---|
| `results/bench_last_run.mat` | 最近一轮的完整信号记录，固定名，每跑一次覆盖一次 | `uacs_bench_record.m:8-10`、`uacs_bench_record.m:53-57` |
| `results/bench_report.txt` | 最近一轮的表，由报表函数打出 | `uacs_bench_record.m:60-65`、`verify/bench_report.m:47` |
| `results/runs/bench_<时间戳>.mat` | 同一轮的归档，时间戳形如 `yyyymmdd_HHMMSS` | `uacs_bench_record.m:28`、`uacs_bench_record.m:54` |
| `results/runs/bench_<时间戳>.txt` | 同一轮的表，与归档 `.mat` 同名 | `uacs_bench_record.m:62` |
| `results/bench_run_trace.txt` | 追加写的流水，每轮一行，记时间戳、模型名、两个落点路径与数据来源；哪一步失败也写在这里 | `uacs_bench_record.m:26`、`uacs_bench_record.m:48`、`uacs_bench_record.m:58` |

归档目录只保留最近 5 轮，超出的按时间戳名删旧，`.mat` 与同名 `.txt` 一起删（来源：`uacs_bench_record.m:22`、`uacs_bench_record.m:66-80`）。数据来源两种落法都认，先取 `out.logsout`，退取 `logsout`；两个都不在就写一行说明，不留空（来源：`uacs_bench_record.m:33-48`）。全程包在错误处理里，钩子失败只留痕，绝不打断飞行（来源：`uacs_bench_record.m:17-18`、`uacs_bench_record.m:82-87`）。

### 8.2 报表怎么生成

`verify/bench_report.m` 读基础工作区里的信号记录，按四路信号打一张带通道名字的表，写 `results/bench_report.txt`（来源：`verify/bench_report.m:8-12`、`verify/bench_report.m:39-42`）。表分五节：空速与高度、迎角与失速、姿态跟踪、速率跟踪、舵面与油门，末尾给几句结论（来源：`verify/bench_report.m:50-127`）。结论那一段是给目测用的，它会把「空速停住」这类现象拆成两种：一种是速度环按指令把速度按住，一种是真出问题（来源：`verify/bench_report.m:112-120`）。报表只读工作区与模型参数，不改模型（来源：`verify/bench_report.m:10-11`）。也可以单独发起一次，作业入口是 `run_job('bench_report')`（来源：`docs/handover.md:543`）。

### 8.3 布局改动怎么保住

台架模型由生成脚本整份重建，重建会先删掉盘上那个文件（来源：`sim/sim_new_model.m:13-18`、`sim/sim_build_bench.m:49`）。所以你在画布上摆过的位置只存在于 `.slx` 里，重建一次就没了。为此有三件事。

| 文件 | 作用 | 来源 |
|---|---|---|
| `run/uacs_job_bench_layout.m` | 作业 `run_job('bench_layout')`，读盘上模型，把每个块与每条注释的位置导出，并写一份诊断件 `results/bench_layout.txt` | `docs/handover.md:294` |
| `sim/uacs_bench_layout_list.m` | 自动生成的数据件，记块位置与注释位置两张表，不要手改，改了下次导出会被覆盖 | `sim/uacs_bench_layout_list.m:4-11` |
| `sim/uacs_bench_layout_apply.m` | 重建收尾时套回。只套还存在的块；新建的块在数据里没有记录，留在生成脚本给的位置上 | `sim/uacs_bench_layout_apply.m:10-11`、`sim/sim_build_bench.m:218` |

保住布局的做法固定成两步：在界面上摆好并保存 `.slx`，然后重跑 `run_job('bench_layout')`（来源：`docs/handover.md:333`）。之后任何一次重建都会把这份数据套回去，构建日志会打一行「画布布局：套回 N 个块，数据里有而模型里没有的 M 个；注释套回 K 条，缺 L 条」（来源：`sim/sim_build_bench.m:219-220`）。

注释只能按正文或首行标题当键，因为注释没有独立于正文的身份（来源：`sim/uacs_bench_layout_apply.m:12-15`）。本机判读得到两个数：数据件里记着 271 个块与 9 条注释，盘上模型现在是 271 个块与 3 条注释，注释那一项两边已经不一致：那份数据件是在六块说明框被删掉之前导出的，之后没有重新导出。动过画布之后重跑一次 run_job('bench_layout') 就对齐了。套回结果那个「N 个块」是某一次构建当次的输出，不落盘，两者不要混（来源：`docs/handover.md:301-308`）。

### 8.4 重建与改参数

- 重建台架：`run_job('bench_build')`，脚本体是 `run/build_bench_models.m`，它一次重建台架与气动测试页两个交互模型（来源：`docs/handover.md:541`、`run/build_bench_models.m:42-45`）。
- 改仿真时长：改 `uacs_go.m` 第 1.9 节的 `uacs_sim.bench_run_s` 再按 F5，或者直接把工具栏那个框改成数字（来源：`uacs_go.m:139-144`）。
- 改包线：改第 1.8 节的两个包线角（来源：`uacs_go.m:127-128`）。放宽之后 V1 到 V13 的验收结论不再适用（来源：`sim/sim_build_bench.m:152-153`）。
- 改侧风方向或加阵风：改第 1.10 节，那里是 NED 三分量与阵风开关（来源：`sim/sim_build_bench.m:158-160`）。
- 用手柄：把第 1.11 节的 `uacs_joystick.id` 改成 `Joystick1`，重跑一次重建，再把操纵台的操纵源拨到「手柄」（来源：`sim/sim_build_bench.m:163-167`）。

### 8.5 两个操作上的坑

- 初值推送挂在打开模型时而不是每次 Run。挂在每次 Run 会把你在一个会话里拖好的杆位在按 Run 的瞬间重置回 `uacs_go.m` 的值（来源：`uacs_bench_seed.m:11-12`）。
- 满杆矩阵作业收尾会把五个常量的 `Value` 恢复成变量表达式（来源：`run/uacs_job_bench_stick.m:223-227`）。而控件驱动不了值是表达式的参数，所以那一轮之后同一个模型会话里操纵台会拖不动，重新打开模型或重建才恢复（来源：`sim/sim_hmi.m:33-34`）。

---

## 9. 一页速查

### 9.1 八个控件

| 控件 | 绑的块 | 变量 | 范围 |
|---|---|---|---|
| `俯仰杆` | `驾驶员/k_pitch` | `sim_stick_pitch` | −1 到 1 |
| `滚转杆` | `驾驶员/k_roll` | `sim_stick_roll` | −1 到 1 |
| `方向舵` | `驾驶员/k_yaw` | `sim_stick_yaw` | −1 到 1 |
| `速度指令` | `驾驶员/k_thr` | `sim_stick_thr` | 0 到 1，空速指令 15.4665 到 30 m/s |
| `侧风 东向 mps` | `大气与风扰动/wind_e` | `sim_dist_wind_e` | −10 到 10 m/s |
| `模式` | `制导与模式/mode_const` | `sim_flight_mode` | 0 姿态、1 高度 |
| `控制器` | `控制器选择/k_sel` | `sim_ctrl_sel` | 0 PID、1 LQR |
| `操纵源` | `驾驶员/js_on` | `sim_js_on` | 0 虚拟杆、1 手柄 |

### 9.2 仪表与图

| 类别 | 数量 | 绑什么 |
|---|---|---|
| 圆形仪表 | 12 | 6 块绑导航与测量的分接，3 块绑参考向量的分量，3 块绑混合与分配换算出来的舵面角度 |
| 失速灯 | 1 | 失速标志 |
| 状态读数 | 3 | 三个子系统各自的档位输出 |
| 画布内示波器 | 5 | 姿态 4 路、速率 pq 4 路、速率 r 2 路、舵面 3 路、油门 2 路 |
| 兜底示波器 | 1 | 姿态、速率、舵面与油门三段，三行上下排，不自动弹出 |

### 9.3 四路记录

| 记录 | 来源 | 路数 | 采样间隔 |
|---|---|---|---|
| `inst_log` | `导航与测量` 第 3 路 | 7 | 0.0025 s |
| `att_log` | `att_mux` | 4 | 0.0025 s |
| `rate_log` | `rate_mux` | 6 | 0.0025 s |
| `act_log` | `act_mux` | 5 | 0.01 s |

### 9.4 采样时间与求解器

| 项 | 值 | 变量 |
|---|---|---|
| 求解器 | 定步长 `ode4` | 无 |
| 步长 | 0.0025 s | `sim_ts_ctrl` |
| 停止时间 | 缺省 20 s | `sim_bench_t_end` |
| 内环 | 0.0025 s | `sim_ts_ctrl` |
| 姿态与能量环 | 0.010 s | `sim_ts_outer` |
| 分配器 | 0.010 s | `sim_ts_alloc` |
| 传感器 | 0.0025 s | `sim_ts_sensor`，台架里没有块引用 |

---

## 10. 与 `docs/handover.md` 不一致的地方

以源码为准，逐条列出。

1. 画布内示波器条数。`sim/sim_build_bench.m:10` 的注释写「三条画布内示波器」，实际是五条，同一份文件的自查断言就写着 5（来源：`sim/sim_build_bench.m:10` 与 `sim/sim_build_bench.m:205-206`）。`run/build_bench_models.m:8` 也沿用了「三条」这个旧说法。这是注释没跟上改动，不是模型有问题。
2. 顶层分带条数。`sim/sim_build_bench.m:6-7` 写「顶层只有四类东西，从左到右分成四条带」，紧接着只列出三类：操纵台、控制律与对象、仪表（来源：`sim/sim_build_bench.m:6-10`）。
3. 中文说明的块数。`docs/handover.md:199` 写「画布上有五块中文说明」并列出五条，而实际有六条正文说明加三条分带标题，共 9 条注释（当时）；`docs/handover.md:373` 自己也把 `note_log` 说成新增的一块说明（来源：`sim/sim_build_bench.m:97-167`、`docs/handover.md:199`、`docs/handover.md:373`）。
4. 运行速度。`uacs_go.m:144` 的注释写每秒约 7 秒真实时间，而满杆矩阵作业记的是 15 到 18 倍，用户实测是 13 到 18 倍（来源：`uacs_go.m:144`、`run/uacs_job_bench_stick.m:8`）。以实测为准，那句注释偏乐观。
5. 输出口编号的完整性。`docs/handover.md:136-137` 只提到顶层多两个汇输出口，说 `da_deg` 是第 14 路、`de_deg` 是第 15 路，没有提第 13 路。模型里第 13 路是 `dr_deg`，三路都在（来源：`sim/sim_bench_cockpit.m:156-161`、`model/uacs_flight_bench.slx` 判读）。

另外记一条盘上快照的观察，供复核时留意。本机判读 `.slx` 得到：`驾驶员/k_yaw` 的字面值是 0.9778859456380209，`k_roll` 与 `k_pitch` 是接近零的小数，`k_thr` 没有写 `Value` 参数。这组数字是上一次有人在界面上拖过并保存留下的痕迹，不是 `uacs_go.m` 的缺省；打开模型时 `PostLoadFcn` 会按 `uacs_go.m` 覆盖它们（来源：`uacs_bench_seed.m:22-34`）。`k_thr` 之所以没有那个参数，是因为它的值等于常量块的内置默认值 1，Simulink 不把等于默认值的参数写进文件。
