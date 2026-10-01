---
module: map
doc: README
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / clang AST / python extractors）
generated_at: 2026-10-02T00:30:00Z
status: draft
reviewer: 未审核
---

# map 模块

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— 2D/3D 机器人导航（RM 哨兵类比赛栈） |
| 技术栈 | 纯 C++20 + ROS 2；Eigen、PCL、yaml-cpp、spdlog；CMake target `3d_occ_map` |
| 模块 | `map`（源码 `src/map/`，库名 `3d_occ_map`） |
| 职责 | 由点云与位姿构建 3D 概率占据地图、膨胀地图、ESDF、通过性（terrain）分析与 2D 规划栅格，供 planner 查询 |

库名与源文件收集方式：`src/map/CMakeLists.txt:5`。

## 【模块概述】

`map` 模块对外只暴露一个门面 `ma_map::MaMap`，它内部持有两套地图：

1. **3D 概率占据地图** `rog_map::ROGMap` —— 由 `MaMap` 构造并初始化（`src/map/include/map/ma_map.hpp:14`、`src/map/include/map/ma_map.hpp:18`）。
2. **2D 规划栅格** `grid_map::GridMap` —— 供 A*/JPS 做二维规划（`src/map/include/map/ma_map.hpp:13`）。

3D 地图在 `ROGMap` 内部由一条继承链叠起来：`SlidingMap` → `ProbMap` → `ROGMap`，
其中 `ProbMap` 再组合 `InfMap`（膨胀）、`FreeCntMap`（前沿计数）、`ESDFMap`（距离场）
与 `raycaster::RayCaster`（射线投射）。`ProbMap` 的三个子地图成员见
`src/map/include/map/3d_occ_map/prob_map.h:118`（`inf_map_`）、
`src/map/include/map/3d_occ_map/prob_map.h:119`（`fcnt_map_`）、
`src/map/include/map/3d_occ_map/prob_map.h:120`（`esdf_map_`），
射线投射器在 `src/map/include/map/3d_occ_map/prob_map.h:128`。

数据只从 `MaMap` 的两个入口进来：位姿 `update_odom`（`src/map/include/map/ma_map.hpp:99`）
与点云 `update_cloud`（`src/map/include/map/ma_map.hpp:102`）；真正的地图更新发生在
`MaMap::update_map`（`src/map/include/map/ma_map.hpp:50`）里调用
`rog_map_ptr_->updateProbMap`（`src/map/include/map/ma_map.hpp:79`）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| 概率占据更新、膨胀计数、ESDF、terrain 通过性、2D 栅格与语义标记 | 路径搜索与后处理（planner/path_planning） |
| 地图的查询接口（`isLineFree`、`boxSearch`、`getDistance`） | 轨迹优化与控制（planner/traj_optimize、planner/controller） |
| YAML 地图参数解析（`rog_map::Config`） | 重规划状态机（planner/replan） |
| —— | ROS2 话题/服务（ros2 模块持有，`map` 不注册任何 publisher/subscription） |

`map` 不直接持有 ROS2 接口：`src/ros2/src/ros2_node.cpp:33` 构造
`ma_map::MaMap`，点云与里程计由 ros2 层手动喂入。

## 入口与调用关系

| 调用方 | 调用点 | 说明 |
|---|---|---|
| ros2 层 | `src/ros2/src/ros2_node.cpp:33` | 构造 `ma_map::MaMap`，传入 map.yaml 路径 |
| ros2 层 | `src/map/include/map/ma_map.hpp:99` | `update_odom` 更新机器人位姿 |
| ros2 层 | `src/map/include/map/ma_map.hpp:102` | `update_cloud` 缓存点云 |
| ros2 层 | `src/map/include/map/ma_map.hpp:50` | `update_map` 触发一次地图更新 |
| planner | `src/map/include/map/ma_map.hpp:130` | `get_rog_map` 取 3D 地图做查询 |
| planner | `src/map/include/map/ma_map.hpp:124` | `get_grid_map` 取 2D 栅格做搜索 |

## 源文件清单

覆盖模块内全部 22 个源文件（校验器 `coverage.missing_file` 会检查这一点）。

### 门面层

| 文件 | 职责 |
|---|---|
| `src/map/include/map/ma_map.hpp:10` | 模块门面 `ma_map::MaMap`，持有 3D 概率地图 + 2D 栅格 + terrain 分析器 |

### 2D 栅格与通过性

| 文件 | 职责 |
|---|---|
| `src/map/include/map/grid_map.hpp:38` | `grid_map::GridMap`：2D 占据栅格、ESDF、语义标记（隧道） |
| `src/map/src/grid_map.cpp:3` | `GridMap` 实现（init/setMap/inflate/getDistance 等） |
| `src/map/include/map/terrain_analysis.hpp:15` | `Terrain::TerrainAnalyzer`：窗口内高度/间隙分析，输出障碍点 |
| `src/map/src/terrain_analysis.cpp:8` | `TerrainAnalyzer::analyze` 实现 |

### 3D 概率地图继承链

| 文件 | 职责 |
|---|---|
| `src/map/include/map/3d_occ_map/sliding_map.h:43` | `rog_map::SlidingMap`：局部地图基类，全局/局部索引与哈希换算、滑窗 |
| `src/map/src/sliding_map.cpp:46` | `SlidingMap` 实现 |
| `src/map/include/map/3d_occ_map/prob_map.h:35` | `rog_map::ProbMap`：概率占据体素、射线投射更新、遗忘 |
| `src/map/src/prob_map.cpp:29` | `ProbMap` 实现 |
| `src/map/include/map/3d_occ_map/rog_map.h:38` | `rog_map::ROGMap`：对外查询接口（`isLineFree`、最近格搜索、机器人状态） |
| `src/map/src/rog_map.cpp:28` | `ROGMap` 实现 |

### 3D 子地图

| 文件 | 职责 |
|---|---|
| `src/map/include/map/3d_occ_map/counter_map.h:33` | `rog_map::CounterMap`：子格计数基类（占据/未知计数、跳变回调） |
| `src/map/src/counter_map.cpp:29` | `CounterMap` 实现 |
| `src/map/include/map/3d_occ_map/inf_map.h:31` | `rog_map::InfMap`：膨胀计数地图，回答 `isOccupiedInflate` 等 |
| `src/map/src/inf_map.cpp:29` | `InfMap` 实现 |
| `src/map/include/map/3d_occ_map/esdf_map.h:35` | `rog_map::ESDFMap`：欧氏距离场与梯度 |
| `src/map/src/esdf_map.cpp:30` | `ESDFMap` 实现 |
| `src/map/include/map/3d_occ_map/free_cnt_map.h:34` | `rog_map::FreeCntMap`：前沿（frontier）自由邻居计数 |
| `src/map/include/map/3d_occ_map/raycaster.h:55` | `rog_map::raycaster::RayCaster`：体素遍历射线投射 |
| `src/map/src/raycaster.cpp:28` | `RayCaster` 实现 |

### 参数与工具

| 文件 | 职责 |
|---|---|
| `src/map/include/map/3d_occ_map/config.hpp:49` | `rog_map::Config`：全部地图参数，经 `yaml_loader` 读取 map.yaml |
| `src/map/include/map/3d_occ_map/common_lib.hpp:42` | 模块内自由函数：四元数转 ypr、路径长度、线段与盒相交 |
