---
module: path_planning
doc: README
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T02:00:00Z
status: draft
reviewer: 未审核
---

# path_planning 模块

> 本文每条【事实】都带 `文件:行号` 锚点，行号对应当前工作区（`git_rev` 见 front-matter）。
> 证据直接读不出的结论标【推断】，两侧对不上的标【待确认】并进入
> [待确认清单](./待确认清单.md)。本模块**没有**任何自动化测试，因此
> [测试要点](./测试要点.md) 全部是【推断】。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈，ROS 2 包名 `ma_nav`（`package.xml:4`） |
| 技术栈 | C++20（`CMakeLists.txt:8`）、Eigen、spdlog；模块自身**不使用** rclcpp |
| 模块 | CMake 库 target `path_planning`（`src/planner/path_planning/CMakeLists.txt:3`） |
| 职责 | 栅格图上的起点→终点搜索（A* / JPS）与轨迹后处理（路径剪枝、5D 状态采样、梯形速度时间分配、隧道前折叠降速） |

## 【模块概述】

本模块是「上游几何规划器」：输入一张 ESDF 栅格图、起终点与限制速度，
输出一条带时间的 5D 轨迹，交给下游 traj_optimize 模块做 MINCO 轨迹优化
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:108`）。
对外只暴露一个门面类 `path_planning::PathPlanning`
（`src/planner/path_planning/include/path_planning/path_planning.hpp:11`），
其实现体 `path_planning::PathPlanningImpl` 定义在 .cpp 内
（`src/planner/path_planning/src/path_planning.cpp:4`），
通过 `std::unique_ptr` 隐藏（`src/planner/path_planning/include/path_planning/path_planning.hpp:13`）。

算法有两套可切换：`path_planning::AStar`（默认）与 `path_planning::JPS`
（`src/planner/path_planning/src/path_planning.cpp:41-42`）；
后处理由 `path_planning::PathPostProcessing` 承担
（`src/planner/path_planning/include/path_planning/post_processing.h:8`）。

【事实】本模块**不注册任何 ROS 2 话题、服务或定时器**：CMake 链接列表里没有
rclcpp，源码中也没有任何 rclcpp/ros2 头文件（`src/planner/path_planning/CMakeLists.txt:13-18`）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| 栅格图上的 A* / JPS 搜索（`src/planner/path_planning/src/search/astar.cpp:14`） | 三维占据/ESDF 地图维护（map 模块） |
| 路径剪枝与直线碰撞检查（`src/planner/path_planning/src/post_processing.cpp:4`） | 轨迹平滑与动力学优化（traj_optimize 模块） |
| 5D 状态采样与梯形速度时间分配（`src/planner/path_planning/src/post_processing.cpp:121`） | 重规划状态机与失败重试（replan 模块） |
| 参数结构体与注入接口（`src/planner/path_planning/include/path_planning/post_processing.h:67`） | YAML 解析与参数装载（ros2 模块，见 [配置说明](./配置说明.md)） |

【事实】本模块不读取任何 YAML：全模块源码中没有 `LoadParam` 调用，也没有
yaml-cpp include（`src/planner/path_planning/include/path_planning/post_processing.h:2-4`）。

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 唯一持有者 | replan 的 FsmReplan 以成员方式持有门面对象 | `src/planner/replan/include/replan/fsm_replanner.h:97` |
| 参数注入 | replan 的 `set_param` 把 `PathPostProcessingParams` 传入本模块 | `src/planner/replan/include/replan/fsm_replanner.h:45` |
| 地图注入 | 每次规划前 `set_map(*grid_map)` | `src/planner/replan/src/fsm_replanner.cpp:60` |
| 算法选择 | `set_use_jps(true)`（比赛路径当前固定走 JPS） | `src/planner/replan/src/fsm_replanner.cpp:69` |
| 速度注入 | `set_velocity(current_vel, Vector3d::Zero())` | `src/planner/replan/src/fsm_replanner.cpp:72` |
| 调用规划 | `path_planning(start, goal, yaw0, yaw1, 5000)`，超时 5000 ms | `src/planner/replan/src/fsm_replanner.cpp:74` |
| 参数来源链 | `ros2_node.cpp` 把 YAML 装载好的 `planner_config` 交给 replan | `src/ros2/src/ros2_node.cpp:36` |

【推断】`set_use_jps(true)` 是当前唯一生效的算法选择路径（两处调用都传 true），
因此 A* 在比赛链路中只作为对照实现保留
（依据：`src/planner/replan/src/fsm_replanner.cpp:69` 与
`src/planner/replan/src/fsm_replanner.cpp:124`）。

## 源文件清单

| 文件 | 职责 |
|---|---|
| `src/planner/path_planning/include/path_planning/path_planning.hpp` | 门面类声明（Pimpl），`src/planner/path_planning/include/path_planning/path_planning.hpp:11` |
| `src/planner/path_planning/src/path_planning.cpp` | 门面实现与五步规划流水线，`src/planner/path_planning/src/path_planning.cpp:29` |
| `src/planner/path_planning/include/path_planning/post_processing.h` | 后处理类、轨迹数据结构与参数结构体，`src/planner/path_planning/include/path_planning/post_processing.h:44` |
| `src/planner/path_planning/src/post_processing.cpp` | 剪枝/采样/时间分配/折叠降速实现，`src/planner/path_planning/src/post_processing.cpp:121` |
| `src/planner/path_planning/include/path_planning/search/astar.h` | A* 类与节点结构，`src/planner/path_planning/include/path_planning/search/astar.h:44` |
| `src/planner/path_planning/src/search/astar.cpp` | A* 搜索实现（字符串键哈希表 + 优先队列），`src/planner/path_planning/src/search/astar.cpp:14` |
| `src/planner/path_planning/include/path_planning/search/jps.h` | JPS 类与扁平数组状态，`src/planner/path_planning/include/path_planning/search/jps.h:28` |
| `src/planner/path_planning/src/search/jps.cpp` | JPS 搜索实现（跳点射线 + 强制邻居剪枝），`src/planner/path_planning/src/search/jps.cpp:192` |
| `src/planner/path_planning/CMakeLists.txt` | 库 target 与链接依赖，`src/planner/path_planning/CMakeLists.txt:7` |
