---
module: replan
doc: README
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T05:00:00Z
status: draft
reviewer: 未审核
---

# replan 模块

> 本文每条【事实】都带 `文件:行号` 锚点。证据直接读不出的结论标【推断】，
> 两侧对不上的标【待确认】并进入 [待确认清单](./待确认清单.md)。
> 完整符号表（66 条）见 `docs/_evidence/replan/inventory.json`。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈（ROS 2 + C++20） |
| 技术栈 | C++20、Eigen、期望值风格错误返回（tl::expected 语义）、spdlog |
| 模块 | CMake 库 target `replan`（`src/planner/replan/CMakeLists.txt:3`） |
| 职责 | 重规划决策与编排：判断何时重规划 → 调用 path_planning 搜索 → 调用 traj_optimize 优化 → 返回轨迹或错误 |

## 【模块概述】

本模块是 planner 链路的**编排层**：门面类 `replan::FsmReplan`
（`src/planner/replan/include/replan/fsm_replanner.h:19`）
持有 `path_planning` 门面与 MINCO 优化器（`src/planner/replan/include/replan/fsm_replanner.h:97-98`），
对外只暴露一个入口 `plan`（`src/planner/replan/include/replan/fsm_replanner.h:83`）。

每次调用按三类条件决定是否重规划：

1. 目标点相对上次成功规划的目标变化超过阈值
   （`src/planner/replan/src/fsm_replanner.cpp:35-39`）；
2. 上一轮轨迹被判定为碰撞（或为空）
   （`src/planner/replan/src/fsm_replanner.cpp:41-46`）；
3. 机器人相对参考路径的横向偏差超过阈值
   （`src/planner/replan/src/fsm_replanner.cpp:49-56`）。

需要重规划时：起点/终点先沿 ESDF 梯度外推到安全点
（`src/planner/replan/src/fsm_replanner.cpp:66-67`），
再走 JPS 搜索（`src/planner/replan/src/fsm_replanner.cpp:69-74`），
最后做 MINCO 五阶轨迹优化（`src/planner/replan/src/fsm_replanner.cpp:100`）。

【事实】返回类型是 `replan::FsmReplan::path`（期望值/错误的联合）
（`src/planner/replan/include/replan/fsm_replanner.h:81`），
错误只有两种会被真正返回：规划失败与优化失败
（`src/planner/replan/src/fsm_replanner.cpp:79`、`src/planner/replan/src/fsm_replanner.cpp:103`）。

【事实】本模块**不使用** ROS 通信：不 include rclcpp，也不解析 YAML
（`src/planner/replan/include/replan/fsm_replanner.h:2-17` 是完整 include 列表）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| 重规划触发判定（`src/planner/replan/src/fsm_replanner.cpp:35-56`） | 栅格搜索与轨迹后处理（path_planning 模块） |
| 起终点的安全外推（`src/planner/replan/src/fsm_replanner.cpp:237`） | 轨迹优化的数学模型（traj_optimize 模块） |
| 折线碰撞复检（`src/planner/replan/src/fsm_replanner.cpp:225`） | 底盘指令求解（controller 模块） |
| 两个子模块的调用顺序与错误传播（`src/planner/replan/src/fsm_replanner.cpp:58-110`） | 指令下发、导航状态机、可视化（ros2 模块） |

【事实】模块内保留了一段**未接入**的实现 `one_plan`
（`src/planner/replan/src/fsm_replanner.cpp:114`），
它不做过安全点外推、也不设置「新轨迹」标志；全仓库没有调用点。

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 唯一持有者 | ros2 节点以成员持有重规划器 | `src/ros2/include/ros2/ros2_node.h:104` |
| 参数注入 | 节点把三组参数一次性注入 | `src/ros2/src/ros2_node.cpp:36` |
| 每帧调用 | 用目标、当前状态与地图调用规划入口 | `src/ros2/src/ros2_node.cpp:272` |
| 结果判定 | 失败时按超时决定导航状态 | `src/ros2/src/ros2_node.cpp:273-279` |
| 新轨迹下发 | 成功且是新轨迹时喂给控制器 | `src/ros2/src/ros2_node.cpp:308-313` |

## 源文件清单

| 文件 | 职责 |
|---|---|
| `src/planner/replan/include/replan/fsm_replanner.h` | 门面声明、参数结构、状态与错误枚举、投影结果结构，`src/planner/replan/include/replan/fsm_replanner.h:19` |
| `src/planner/replan/src/fsm_replanner.cpp` | 重规划主流程与全部辅助函数实现，`src/planner/replan/src/fsm_replanner.cpp:13` |
| `src/planner/replan/CMakeLists.txt` | 库 target 与链接，`src/planner/replan/CMakeLists.txt:3` |
