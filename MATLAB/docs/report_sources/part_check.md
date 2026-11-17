## 需要复核的清单

分三档。第一档只影响文字，第二档是两处口径对不上，第三档要再跑一次仿真才能回答。

### 第一档，只影响注释与文档，不影响模型行为

1. `docs/handover.md:199` 写「五块中文说明」，画布上当时是 6 条正文说明加 3 条分带标题共 9 条注释，现在删掉 6 条，只剩 3 条分带标题。
2. `sim/sim_build_bench.m:10` 与 `run/build_bench_models.m:8` 还写「三条画布内示波器」，实际 5 条。同一份文件 `sim/sim_build_bench.m:205-206` 的断言写的就是 5。
3. `uacs_go.m:144` 的注释写「每秒约 7 秒真实时间」，实测是每仿真秒 13 到 18 秒。
4. `plant/linearize_at.m` 头注释的示例写步长 `1e-5`，代码默认值是 `1e-6`。

### 第二档，口径不一致，需要人判哪一侧对

1. 偏航速率环的稳定性边界。源码注释说大于 0.433 的增益都不稳定，交付缓存里该环增益是 -0.896236 且增益裕度 2.507 倍。详见第一部分第 9.1 节。
2. 增广闭环稳态误差的判据。作业代码用绝对值小于 `1e-9`（来源：`control/uacs_job_control_design.m:64-65`），`docs/test_architecture.md:151` 写「小于指令的百分之一」。两者不是同一个量。
3. 惯量与面积存在两套口径。`docs/acceptance_report.md:73-75` 与 `docs/design_basis_sizing.md:43` 给的是选型阶段构型，展弦比 5.8、面积 1.9931 平方米、`Ixx = 1.9859` 千克平方米；冻结 rev 11 是展弦比 5.00、面积 2.3120 平方米、`Ixx = 0.836125` 千克平方米。跨这两套引用会算错。

### 第三档，要再跑仿真才能回答

1. 受验模型 `model/uacs_lqr.slx` 里是否出现同一个 50 Hz 升降舵极限环。
2. 把作动器一阶滞后与速率限制加进台架的舵面通路之后，两个控制器的速率环表现如何。
3. 重跑一次 `design_pid` 会得到什么。现行源码里偏航速率环在搜索不到稳定增益时会交付为开口，`Kp = 0`；交付缓存里是闭合的 -0.896236。
4. 重跑一次 `design_lqr` 会得到什么。
5. 把空速环的微分项压下去之后，基准工况的油门换向次数能降到多少。

## 回到界面继续飞之前要注意一件事

满杆作业收尾时把五个常值块的参数从字面量改回了变量表达式，比如 `驾驶员/k_roll` 的 `Value` 变回 `sim_stick_roll`。这是为了不把测试值留在模型里。代价是：**在这样的参数上，画布上的旋钮与滑杆拖不动**，因为控件只能驱动字面量参数。

模型关掉再打开就恢复，因为打开时 `PostLoadFcn` 会把 `uacs_go.m` 的杆位推回成字面量。要在同一次打开里恢复，在命令行执行一次：

```matlab
uacs_go;
load_system('uacs_flight_bench');
uacs_bench_seed;
```

`uacs_bench_seed` 会打印推入了几个块，正常是 8 个。

## 本轮做了什么，没有做什么

做了的：

- 以脚本方式跑了满杆矩阵，54 段，每段 12 秒仿真，PID 与 LQR 各 27 段。
- 在跑着的模型上直接取参数，复核了效力矩阵与线性化模型的一致性、气动导数、惯量、采样时间、速率指令限幅。
- 量了本机仿真速度并做了分解，否掉了「仪表盘拖慢」这个猜测。
- 写了这份报告与两个脚本。

没有做的：

- 改了一处模型文件：测试期间 `StopFcn` 记录钩子被摘掉过，收尾时把它装回去并保存了一次 `model/uacs_flight_bench.slx`，同时把五个常值块恢复成字面量。块的拓扑、位置与九条注释没有动过，保存前后顶层 70 块、全模型 271 块一致；注释当时是 9 条，现在盘上是 3 条。
- 没有跑 `design_pid` 或 `design_lqr` 的重设计，所以「重跑一次会得到什么」这一类问题本轮给不出结论。
- 没有测受验模型 `uacs_lqr.slx` 与 `uacs_manual_pid.slx`。

## 本轮留下的文件

| 文件 | 内容 |
|---|---|
| `docs/design_tutorial_report.md` | 本报告 |
| `docs/report_sources/` | 本报告各部分的分册与取证笔记，改一处重拼一次用 `assemble.mjs` |
| `verify/bench_stick_report.m` | 由轨迹算指标与分组表的脚本 |
| `verify/bench_stick_plots.m` | 画图脚本 |
| `verify/bench_stick_tables.mjs` | 由指标表生成 markdown 数据表 |
| `run/uacs_job_bench_stick.m` | 满杆矩阵作业 |
| `run/uacs_job_bench_stick2.m` | 同上，多写一行实时进度，续跑用 |
| `results/bench_stick_traj.mat` | 54 段轨迹，报告第三部分每个数的原始出处 |
| `results/bench_stick_metrics.csv` | 54 段指标，一段一行，列名就是指标名 |
| `results/bench_stick_meta.txt` | 测试时的运行参数与参数集指纹 |
| `results/bench_stick_progress.txt` | 实时进度流水 |
| `results/figs/stick_*.png` | 图 |
| `results/_stick_diag.txt` | 前两个毛病的原始判据 |
| `results/_verify_claims.txt` | 参数一致性复核的原始判据 |
