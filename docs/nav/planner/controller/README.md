---
module: controller
doc: README
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T04:00:00Z
status: draft
reviewer: 未审核
---

# controller 模块

> 本文每条【事实】都带 `文件:行号` 锚点。证据直接读不出的结论标【推断】，
> 两侧对不上的标【待确认】并进入 [待确认清单](./待确认清单.md)。
> 完整符号表（80 条）见 `docs/_evidence/controller/inventory.json`。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈（ROS 2 + C++20） |
| 技术栈 | C++20 模板、Eigen（稀疏矩阵）、OSQP（经 OsqpEigen 封装）、spdlog |
| 模块 | CMake 库 target `controller`（`src/planner/controller/CMakeLists.txt:3`） |
| 职责 | 把上游优化好的轨迹追踪成底盘指令：4 状态（x, y, vx, vy）线性 MPC，每帧解一次 QP，输出加速度指令 (ax, ay) |

## 【模块概述】

本模块包含三部分：

1. **MPC 控制器** `control::Mpc`：模板类，以「轨迹接口」为模板参数
   （`src/planner/controller/include/controller/mpc.h:41`），
   把轨迹追踪写成标准 QP，用 OSQP 求解（`src/planner/controller/include/controller/mpc.h:73`）。
2. **轨迹接口** `control::TrajectoryInterface`（CRTP）+ 唯一实现
   `control::MaSplineTrajectoryInterface`
   （`src/planner/controller/include/controller/traj_interface.hpp:29`、`:60`），
   把 traj_optimize 的输出包装成「按时间取参考点」的接口
   （`src/planner/controller/include/controller/traj_interface.hpp:137`）。
3. **yaw 控制占位** `control::YawController`：只有声明与一个枚举，成员函数没有实现
   （`src/planner/controller/include/controller/yaw_plan.hpp:13-22`）。

【事实】主体是**头文件模板**：唯一的 .cpp 只有一行注释、没有代码
（`src/planner/controller/src/mpc.cpp:1`）。

【事实】本模块**不使用** ROS 通信：没有 rclcpp include，也不解析 YAML
（`src/planner/controller/include/controller/mpc.h:3-8` 是 mpc.h 的完整 include 列表）。

【事实】源码中唯一**使用**本模块的是 ros2 模块：`ros2_node.h` 里以
`control::Mpc<control::MaSplineTrajectoryInterface>` 作为成员
（`src/ros2/include/ros2/ros2_node.h:146`），构造时注入 MPC 参数
（`src/ros2/src/ros2_node.cpp:34`），每帧调用求解
（`src/ros2/src/ros2_node.cpp:179`）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| MPC 问题构建与 OSQP 求解（`src/planner/controller/include/controller/mpc.h:148`） | 轨迹优化（traj_optimize 模块） |
| 轨迹接口抽象与样条适配（`src/planner/controller/include/controller/traj_interface.hpp:29`） | 轨迹可视化与话题发布（ros2 模块） |
| 预测状态/控制序列输出（`src/planner/controller/include/controller/mpc.h:496-502`） | 何时下发指令、限速/状态机（ros2 模块） |
| MPC 参数结构体（`src/planner/controller/include/controller/mpc.h:15`） | 参数装载（ros2 模块，见 [配置说明](./配置说明.md)） |

【事实】yaw 控制只在头文件里留了接口骨架，`YawController` 既没有实现也没有调用点
（`src/planner/controller/include/controller/yaw_plan.hpp:20`），
因此当前「云台 yaw」并不由本模块闭环控制。

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 唯一持有者 | ros2 节点以成员持有 `Mpc<MaSplineTrajectoryInterface>` | `src/ros2/include/ros2/ros2_node.h:146` |
| 构造注入 | 用装载好的 MPC 参数构造控制器 | `src/ros2/src/ros2_node.cpp:34` |
| 轨迹注入 | 新轨迹下发时设置轨迹指针 | `src/ros2/src/ros2_node.cpp:313` |
| 每帧求解 | `solve(x0, t_now, u_cmd, predicted_states, predicted_inputs)` | `src/ros2/src/ros2_node.cpp:179` |
| 结果使用 | 求解成功才更新上一帧指令标志 | `src/ros2/src/ros2_node.cpp:189` |

## 源文件清单

| 文件 | 职责 |
|---|---|
| `src/planner/controller/include/controller/mpc.h` | MPC 模板类与全部实现，`src/planner/controller/include/controller/mpc.h:41` |
| `src/planner/controller/include/controller/traj_interface.hpp` | 轨迹接口（CRTP）与样条适配器，`src/planner/controller/include/controller/traj_interface.hpp:29` |
| `src/planner/controller/include/controller/yaw_plan.hpp` | yaw 控制占位（未实现），`src/planner/controller/include/controller/yaw_plan.hpp:13` |
| `src/planner/controller/src/mpc.cpp` | 一行注释说明已改为头文件模式，`src/planner/controller/src/mpc.cpp:1` |
| `src/planner/controller/CMakeLists.txt` | 库 target 与链接（含一个未使用的 qpOASES 链接），`src/planner/controller/CMakeLists.txt:20-29` |