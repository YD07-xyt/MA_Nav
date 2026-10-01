---
module: traj_optimize
doc: README
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T03:00:00Z
status: draft
reviewer: 未审核
---

# traj_optimize 模块

> 本文每条【事实】都带 `文件:行号` 锚点。证据直接读不出的结论标【推断】，
> 两侧对不上的标【待确认】并进入 [待确认清单](./待确认清单.md)。
> 完整符号表（1872 条）见 `docs/_evidence/traj_optimize/inventory.json`。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈（ROS 2 + C++20） |
| 技术栈 | C++20 模板元编程、Eigen、L-BFGS（utils 的 `utils/lbfgs.hpp`）、spdlog |
| 模块 | CMake 库 target `traj_optimize`（`src/planner/traj_optimize/CMakeLists.txt:3`） |
| 职责 | 把上游的梯形时间参考轨迹优化成五阶多项式样条：平滑 + ESDF 避障 + 速度/加速度/yaw 动力学软约束，并（可选）联合优化 yaw |

## 【模块概述】

本模块是「轨迹优化层」：输入 `ma_spline_opt::MaSplineInput`（带时间的参考点、
起终速度/加速度、yaw 角速度），输出 `ma_spline_opt::MAsplineOutput`
（五阶样条 `SplineTrajectory::QuinticSplineND<3>` + 时间段 + 代价 + 成功标志）
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:66-88`）。

主体是**头文件模板实现**：主优化器 `ma_spline_opt::MaSplineTrajectoryOptimizer`
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:20`），
其实现全部写在头内（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:144-480`）；
对应的 .cpp 只有一行注释、没有代码
（`src/planner/traj_optimize/src/ma_spline_opt/traj_optimizer.cpp:1`）。

优化分**两个阶段**，两阶段共用同一问题、同一决策向量，只用不同权重各跑一次
L-BFGS（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:235-244`）；
避障距离场经抽象接口注入，当前实现是 2D 栅格地图适配器
`minco_opt::GridMapESDF`（`src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp:27`）。

【事实】本模块**不使用** ROS 通信：不 include rclcpp，也不解析 YAML
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:3-10` 是配置头的完整 include 列表）。

【事实】模块内**同时存在两套同名样条实现**，且模块自带的 `opt.md` 已给出取舍：
`ma_spline_opt` 是目前主要的代码，`spline_opt` 是后续更新用的代码，
`gcopter` 是 Minco 原实现、未使用
（`src/planner/traj_optimize/include/traj_optimize/opt.md:1-4`）。
本轮文档把 `spline_opt/**` 与 `gcopter/{minco,lbfgs,sdlp}.hpp` 明确列为**停用副本**
（见 [依赖关系](./依赖关系.md) 与 [待确认清单](./待确认清单.md)）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| 样条表示与多项式代数（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1303`） | 栅格搜索与梯形速度时间分配（path_planning 模块） |
| 时间/避障/动力学代价与梯度（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:13`） | ESDF 距离场的构建与维护（map 模块） |
| 决策变量布局、掩码与上下文准备（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:438`） | 参数装载（ros2 模块）与轨迹跟踪控制（controller 模块） |
| L-BFGS 调用与两阶段调度（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:262-322`） | 何时重规划（replan 模块） |

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 唯一持有者 | replan 的 FsmReplan 以成员方式持有优化器 | `src/planner/replan/include/replan/fsm_replanner.h:98` |
| 参数注入 | replan 的 `set_param` 把配置交给优化器 | `src/planner/replan/include/replan/fsm_replanner.h:44` |
| 距离场注入 | 每次优化前把地图适配器指针交给优化器 | `src/planner/replan/src/fsm_replanner.cpp:92-93` |
| 输入转换 | 用 `ma_spline_opt::from_path_planning_trajectory` 把上游轨迹转成本模块输入 | `src/planner/replan/src/fsm_replanner.cpp:94` |
| 调用优化 | `ma_opt_.optimize(ma_intput)` | `src/planner/replan/src/fsm_replanner.cpp:100` |
| 结果校验 | 失败即返回 `MINCO_OPT_FIALED` | `src/planner/replan/src/fsm_replanner.cpp:101-104` |
| 下游消费者 | controller 的轨迹接口 include 本模块头 | `src/planner/controller/include/controller/traj_interface.hpp:4` |
| 可视化 | ros2 可视化头 include 本模块的样条类型 | `src/ros2/include/ros2/misc/visualizer.hpp:5` |

## 源文件清单

17 个文件全部列出；「状态」列依据模块自带说明
（`src/planner/traj_optimize/include/traj_optimize/opt.md:1-4`）与全仓库 include 检索。

| 文件 | 状态 | 职责 |
|---|---|---|
| `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h` | 生效 | 配置结构、输入/输出结构、上游轨迹转换，`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:40` |
| `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h` | 生效 | 主优化器与两阶段 L-BFGS 调度，`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:20` |
| `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp` | 生效 | 时间代价与积分（避障/动力学）代价，`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:13` |
| `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp` | 生效 | 三次/五次/七次样条与 PPoly 表示，`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:81` |
| `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp` | 生效 | 优化器框架：上下文、决策变量布局、掩码、求值，`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:408` |
| `src/planner/traj_optimize/include/traj_optimize/esdf_Interface.hpp` | 生效 | ESDF 抽象接口（CRTP），`src/planner/traj_optimize/include/traj_optimize/esdf_Interface.hpp:10` |
| `src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp` | 生效 | 2D 栅格地图 → ESDF 接口适配器，`src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp:27` |
| `src/planner/traj_optimize/include/traj_optimize/gcopter/trajectory.hpp` | 生效（仅被可视化 include） | 全局命名空间的 `Piece` / `Trajectory` 多项式轨迹，`src/planner/traj_optimize/include/traj_optimize/gcopter/trajectory.hpp:348` |
| `src/planner/traj_optimize/include/traj_optimize/gcopter/root_finder.hpp` | 生效（仅被上一行 include） | 多项式根求解器，`src/planner/traj_optimize/include/traj_optimize/gcopter/root_finder.hpp:1` |
| `src/planner/traj_optimize/src/ma_spline_opt/traj_optimizer.cpp` | 生效（空实现） | 仅一行注释说明已改为头文件模式，`src/planner/traj_optimize/src/ma_spline_opt/traj_optimizer.cpp:1` |
| `src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineTrajectory.hpp` | **停用** | spline_opt 平行副本的样条实现，`src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineTrajectory.hpp:1` |
| `src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineOptimizer.hpp` | **停用** | spline_opt 平行副本的优化器框架，`src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineOptimizer.hpp:1` |
| `src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineConvexHull.hpp` | **停用** | spline_opt 的凸包约束实现，`src/planner/traj_optimize/include/traj_optimize/spline_opt/SplineTrajectory/SplineConvexHull.hpp:1` |
| `src/planner/traj_optimize/include/traj_optimize/spline_opt/tarj_opt.h` | **停用** | 8 行空壳类声明（构造函数无定义），`src/planner/traj_optimize/include/traj_optimize/spline_opt/tarj_opt.h:4` |
| `src/planner/traj_optimize/include/traj_optimize/gcopter/minco.hpp` | **停用** | Minco S3/S4 原实现，`src/planner/traj_optimize/include/traj_optimize/gcopter/minco.hpp:43` |
| `src/planner/traj_optimize/include/traj_optimize/gcopter/lbfgs.hpp` | **停用** | Minco 自带的 L-BFGS 副本（实际用的是 utils 版本），`src/planner/traj_optimize/include/traj_optimize/gcopter/lbfgs.hpp:1` |
| `src/planner/traj_optimize/include/traj_optimize/gcopter/sdlp.hpp` | **停用** | SDLP 线性规划求解器（供 Minco 使用），`src/planner/traj_optimize/include/traj_optimize/gcopter/sdlp.hpp:1` |
| `src/planner/traj_optimize/CMakeLists.txt` | 生效 | 库 target、include 与链接，`src/planner/traj_optimize/CMakeLists.txt:7` |
