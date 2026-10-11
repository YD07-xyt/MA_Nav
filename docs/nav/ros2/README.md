---
module: ros2
doc: README
git_rev: 8f98daee5bc7
evidence_digest: be346b5265f1410d
worktree_dirty: false
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T14:35:04Z
status: draft
reviewer: 未审核
---

# ros2 模块

> 本文每条【事实】都带 `文件:行号` 锚点。证据直接读不出的结论标【推断】，
> 两侧对不上的标【待确认】并进入 [待确认清单](./待确认清单.md)。
> 完整符号表（191 条）见 `docs/_evidence/ros2/inventory.json`；
> ROS 接口面见 `docs/_evidence/repo/ros2.json`。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈 |
| 技术栈 | ROS 2（rclcpp、标准消息、tf2、PCL 互转）、C++20、yaml-cpp、spdlog |
| 模块 | CMake 库 target `ros2`（`src/ros2/CMakeLists.txt:3`），由可执行文件 `ma_nav` 使用 |
| 职责 | 全栈唯一的 ROS 2 集成层：订阅点云/里程计/目标，驱动建图与规划，求解 MPC 并下发 `cmd_vel`，同时负责可视化、折叠指令与地图存取服务 |

## 【模块概述】

本模块是整条链路的**唯一 ROS 入口**：进程入口 `main` 创建节点、读取参数文件路径
（`src/ma_nav.cpp:7-11`），构造 `planner::GlobalPlanner2d`
（`src/ma_nav.cpp:13`），随后以 1000 Hz 自旋（`src/ma_nav.cpp:15-19`）。
节点名固定为 `ma_nav_node`（`src/ma_nav.cpp:7`），launch 文件启动同名节点并声明
参数默认值（`launch/run.launch.py:17-33`）。

节点内部由四块组成：

1. **配置层** `planner::Config`：一次性解析 YAML 参数文件，并把三组子配置分发给
   planner / controller（`src/ros2/include/ros2/config.hpp:295`）。
2. **节点主体** `planner::GlobalPlanner2d`：持有地图、重规划器、MPC、可视化器与全部
   ROS 接口（`src/ros2/include/ros2/ros2_node.h:58`）。
3. **可视化器** `Visualizer`：24 个 `/ma_nav/*` 可视化话题在这里创建（另有 3 个 `/ma_nav/*` 话题在节点构造函数里创建：点选区域、折叠、导航反馈）
   （`src/ros2/include/ros2/misc/visualizer.hpp:27`）。
4. **颜色工具** `utils::Color`：把 16 进制/浮点颜色转成 ROS 颜色消息
   （`src/ros2/include/ros2/color_msg_utils.hpp:5`）。

【事实】ROS 接口共 **34 个**主题/服务：**28 个发布**（其中 24 个可视化发布者集中在
`Visualizer` 构造函数，`src/ros2/include/ros2/misc/visualizer.hpp:60-83`；
另 4 个在节点构造函数里，`src/ros2/src/ros2_node.cpp:98-101`）、
**4 个订阅**（`src/ros2/src/ros2_node.cpp:51-97`）、
**2 个服务**（`src/ros2/src/ros2_node.cpp:104-120`）；
完整清单见 `docs/_evidence/repo/ros2.json` 的 `interfaces` 表与 [接口文档](./接口文档.md)。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| ROS 接口的创建与回调（`src/ros2/src/ros2_node.cpp:27-129`） | 地图内部实现（map 模块） |
| 参数文件解析与分发（`src/ros2/include/ros2/config.hpp:316`） | 搜索/优化/控制算法（planner 三个子模块） |
| 规划与控制的调度时序（`src/ros2/src/ros2_node.cpp:245-371`） | 可视化的几何生成细节（本模块 Visualizer 内部） |
| 折叠事件生成与折叠指令下发（`src/ros2/src/ros2_node.cpp:915-976`） | 云台折叠的物理执行（外部机构） |
| 全局地图与隧道区域的存取（`src/ros2/src/ros2_node.cpp:715-864`） | 关节/云台控制（不在本仓库） |

【事实】本模块是唯一依赖 rclcpp 的模块：依据是全仓库 `rclcpp` include 检索
（只命中 `src/ros2/**`、`src/ma_nav.cpp` 与两个 CMakeLists）
加上链接声明（`src/ros2/CMakeLists.txt:26`）。

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 进程入口 | `main` 初始化 ROS、声明并读取参数文件路径 | `src/ma_nav.cpp:9-11` |
| 构造节点主体 | 传入节点句柄与参数文件路径 | `src/ma_nav.cpp:13` |
| 自旋 | 1000 Hz 的 `spin_some` 循环 | `src/ma_nav.cpp:15-19` |
| 启动方式 | launch 声明 `planner_params_path` 并启动 `ma_nav` 节点 | `launch/run.launch.py:17-33` |
| 被顶层构建 | ma_nav 可执行文件链接本模块 | `CMakeLists.txt:94` |

## 源文件清单

| 文件 | 职责 |
|---|---|
| `src/ros2/src/ros2_node.cpp` | 节点主体的全部实现：构造、回调、规划、折叠、地图存取，`src/ros2/src/ros2_node.cpp:27` |
| `src/ros2/include/ros2/ros2_node.h` | `GlobalPlanner2d` 声明、回调组/锁/定时器与折叠结构，`src/ros2/include/ros2/ros2_node.h:58` |
| `src/ros2/include/ros2/config.hpp` | `Config` 与 12 个 YAML 装载辅助函数，`src/ros2/include/ros2/config.hpp:295` |
| `src/ros2/include/ros2/misc/visualizer.hpp` | `Visualizer`：全部可视化发布者与绘制函数，`src/ros2/include/ros2/misc/visualizer.hpp:27` |
| `src/ros2/include/ros2/color_msg_utils.hpp` | `utils::Color` 颜色工具，`src/ros2/include/ros2/color_msg_utils.hpp:5` |
| `src/ros2/CMakeLists.txt` | ament 库 target、ROS 依赖与对 planner 各模块的 PRIVATE 链接，`src/ros2/CMakeLists.txt:3` |
