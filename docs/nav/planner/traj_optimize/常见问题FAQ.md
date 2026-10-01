---
module: traj_optimize
doc: 常见问题FAQ
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T03:00:00Z
status: draft
reviewer: 未审核
---

# traj_optimize 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：为什么优化输出的时间分配和上游给的时间不一样？

**原因**：各段时间是**决策变量**，上游给的时间点只用来构造初始段长
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:223`），
之后由时间代价决定：总时间线性项 + 最小时间二次罚 + 与均值的上下界约束
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:178-222`）。

**处理**：若要时间尽量贴近上游，减小总时间权重、放宽均匀性上下界；
注意这两项都在同一阶段参数里（`config/planner.yaml:60-63`）。

## Q2：日志里出现 `lbfgs opt1 failed` / `lbfgs opt2 failed` 是什么意思？

**原因**：两个阶段各自调用一次 L-BFGS，失败时分别打这两条日志并返回同一错误码
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:235-244`）。
失败判据是「返回码为负、代价非有限、或决策向量非有限」
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:321`）。

**处理**：失败会继续向上传播成 replan 的优化失败错误
（`src/planner/replan/src/fsm_replanner.cpp:101-104`），因此要看的是**是哪个阶段**失败：
阶段一失败通常意味着问题定义或初值有问题，阶段二失败通常是权重/阈值把可行域挤没了。

## Q3：模块里为什么有两套 `SplineTrajectory.hpp` / `SplineOptimizer.hpp`？该改哪一套？

**原因**：模块自带的说明写明了取舍
（`src/planner/traj_optimize/include/traj_optimize/opt.md:1-4`）：
`ma_spline_opt` 是目前主要的代码，`spline_opt` 是后续更新用的代码。
全仓库的 include 检索也印证了这一点：replan、controller、ros2 只引用 `ma_spline_opt/`
（`src/planner/replan/include/replan/fsm_replanner.h:5`）。

**处理**：改 `ma_spline_opt/` 一侧。**不要**为了「顺手统一」把两套合并，
否则会改变当前编译路径。

## Q4：`gcopter` 目录会参与编译吗？

**原因**：作者注明 gcopter 是 Minco 原实现、未使用
（`src/planner/traj_optimize/include/traj_optimize/opt.md:4`），
且全仓库没有任何 include 指向 `gcopter/minco.hpp`、`gcopter/lbfgs.hpp`、`gcopter/sdlp.hpp`。

**处理**：`gcopter/trajectory.hpp` 是个例外——它被 ros2 的可视化头 include
（`src/ros2/include/ros2/misc/visualizer.hpp:7`），
而且该头里确实有一个以 gcopter 的 Trajectory 为形参、调用它的接口的重载
（`src/ros2/include/ros2/misc/visualizer.hpp:349`）。
但这个重载**没有调用点**（唯一调用处已被注释，见 `src/ros2/src/ros2_node.cpp:306`），
因此模板从未被实例化。【待确认】是删掉这个 include 与死重载，还是把它接入流程。

## Q5：yaw 是怎么被优化的？为什么 2D 模式下输出的 yaw 全是 0？

**原因**：模型由输入字段决定：联合模式优化 (x,y,yaw) 三维样条
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:410-457`），
2D 模式只优化 xy，输出时把 yaw 补零并扩展成三维样条
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:120-142`）。

**处理**：需要 yaw 机动就切到联合模式；当前调用方使用的是 2D 模式
（`src/planner/replan/src/fsm_replanner.cpp:97`）。

## Q6：只改 stage1 的权重，为什么在输出上看不出差别？

**原因**：【推断】stage1 的作用是给 stage2 提供一个更好的**初值**，
最终输出的样条、时间段与代价都取自 stage2
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:240-256`）。
当 stage2 的权重足够强时，局部最优解可能被拉回同一处，stage1 的调整就被「洗掉」。

**处理**：要验证 stage1 的影响，应该同时观察中间解或临时关闭 stage2 的强权重，
而不是只看最终输出。

## Q7：`check_trajectory_collision` 返回 true 是「有碰撞」吗？

**原因**：不是。返回 false 表示「未成功或样条未初始化」，
返回 true 表示沿样条以固定步长采样后**没有**发现低于安全距离的点
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:145-162`）。

**处理**：该函数在本仓库**没有任何调用点**（只有声明与定义）。
若要把它作为兜底校验接入，需要显式调用并注意它的真值语义是「通过」。

## Q8：为什么 `prepareContext` 之后再改配置没有效果？

**原因**：上下文在准备时会**快照**时间/空间/辅助状态映射、平滑权重与积分步数；
之后调用 `setConfig` 只影响新准备的上下文
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizerProtocols_zh.md:233-234`）。

**处理**：按「先 `set_config`、再准备上下文」的顺序调用；本模块的阶段函数就是这个顺序
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:269-277`）。

## 参数类问题

| 问题 | 原因 | 锚点 |
|---|---|---|
| 只改 stage2 会不会等于改默认值 | stage2 的 14 个值与代码默认值完全一致 | `config/planner.yaml:75-89` |
| safe_distance 写了 0.35，代码里却是 0.3 | 缺键时回落到声明处的默认值 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:42` |
| integral_num_steps 调大有什么代价 | 每个采样段的积分点数，调大直接增加求值成本 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:271` |
| max_iterations 调小会怎样 | 迭代上限，调小更容易在未收敛时返回 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:305` |
| 上游 time_resolution 影响本模块吗 | 输入点密度决定样条段数与决策向量维度 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:120-122` |

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 日志通道 | 过滤本模块的日志通道 | `src/utils/include/utils/logger.hpp:122` |
| 分阶段失败 | 阶段一 / 阶段二各自有独立日志 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:236` |
| 输入不足 | 参考点少于 2 个时的警告 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:206` |
| 梯度自检 | 框架自带解析/数值梯度对照入口 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:1849` |
| 轨迹可视化 | 下游可视化头按时间采样绘制 | `src/ros2/include/ros2/misc/visualizer.hpp:136` |
