---
module: utils
doc: README
git_rev: 8f98daee5bc7
evidence_digest: be346b5265f1410d
worktree_dirty: false
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T07:00:00Z
status: draft
reviewer: 未审核
---

# utils 模块

> 本文每条【事实】都带 `文件:行号` 锚点。证据直接读不出的结论标【推断】，
> 两侧对不上的标【待确认】并进入 [待确认清单](./待确认清单.md)。
> 完整符号表（1010 条）见 `docs/_evidence/utils/inventory.json`。

## 【背景】

| 项 | 内容 |
|---|---|
| 项目 | MA_Nav —— RM 哨兵类比赛用的导航栈 |
| 技术栈 | C++20 头文件库；Eigen、spdlog/fmt、yaml-cpp；另含三个第三方单头（L-BFGS、tl::expected、matchit） |
| 模块 | CMake 库 target `utils`（`src/utils/CMakeLists.txt:3`），被所有其它模块依赖 |
| 职责 | 提供日志与计时、Eigen 类型别名与状态类型、YAML 参数装载、Eigen 的 fmt 格式化，以及两个可直接使用的算法单头 |

## 【模块概述】

本模块是**全仓库最底层的公共库**，没有业务逻辑，只有五类基础设施：

1. **日志**：`logger` 命名空间提供 info/warn/error/debug 四个带源码位置的日志包装
   （`src/utils/include/utils/logger.hpp:13`），以及 15 个预定义通道
   （8 个基础通道 + 7 个彩色通道，`src/utils/include/utils/logger.hpp:101-125`）。
2. **计时**：`utils::TimeConsuming` RAII 计时器，析构时按量级自动选择 ns/us/ms/s 并写日志
   （`src/utils/include/utils/scope_timer.hpp:39`、`src/utils/include/utils/scope_timer.hpp:62-79`）。
3. **类型**：`utils` 命名空间下的 Eigen 别名（`utils::Vec3f`、`utils::Matf` 等）
   与机器人状态/枚举（`src/utils/include/utils/eigen_alias.hpp:15`、
   `src/utils/include/utils/type_utils.hpp:16`）。
4. **参数装载**：`yaml_loader::YamlLoader` 提供带默认值与「必填」语义的模板装载接口
   （`src/utils/include/utils/yaml_loader.hpp:25`）。
5. **第三方单头**：`lbfgs.hpp`（L-BFGS 优化，供 traj_optimize 使用）、
   `expected.hpp`（tl::expected，供 traj_optimize 与 replan 使用）、
   `matchit.h`（模式匹配库，**当前无人使用**）。

【事实】本模块是**纯头文件库**：唯一的 .cpp 是 0 字节空文件
（`src/utils/src/utils.cpp:1`），所有实现都在头文件里以 inline/模板形式提供。

【事实】依赖方向是单向的：所有模块都 include 本模块，本模块不 include 任何业务模块
（`src/utils/include/utils/logger.hpp:2-8` 是 logger 头的完整 include 列表）。

## 模块边界

| 属于本模块 | 不属于本模块（由谁负责） |
|---|---|
| 日志通道定义与格式化（`src/utils/include/utils/logger.hpp:101`） | 各模块的日志内容与级别选择 |
| 计时工具（`src/utils/include/utils/scope_timer.hpp:39`） | 各模块的计时点埋放 |
| Eigen 别名与状态结构（`src/utils/include/utils/type_utils.hpp:69`） | 状态的具体语义与更新逻辑（ros2 模块） |
| YAML 装载机制（`src/utils/include/utils/yaml_loader.hpp:25`） | 参数键的定义与默认值（各业务模块） |
| 算法单头（L-BFGS / expected） | 使用它们的具体优化与错误处理逻辑 |

## 入口与调用关系

| 关系 | 代码位置 | 锚点 |
|---|---|---|
| 被地图模块使用 | 别名、日志、计时、YAML 装载 | `src/map/include/map/3d_occ_map/config.hpp:68` |
| 被 path_planning 使用 | 日志与计时 | `src/planner/path_planning/src/path_planning.cpp:36` |
| 被 traj_optimize 使用 | L-BFGS 与 expected | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:7-8` |
| 被 controller 使用 | 日志（隐式经接口头） | `src/planner/controller/include/controller/mpc.h:6` |
| 被 replan 使用 | expected、日志、状态类型 | `src/planner/replan/include/replan/fsm_replanner.h:9-11` |
| 被 ros2 使用 | 日志、状态类型、绘图器 | `src/ros2/include/ros2/ros2_node.h:22-23` |
| 顶层构建顺序 | 最先加入的子目录 | `CMakeLists.txt:66` |

## 源文件清单

| 文件 | 职责 |
|---|---|
| `src/utils/include/utils/logger.hpp` | 日志包装与 15 个通道，`src/utils/include/utils/logger.hpp:13` |
| `src/utils/include/utils/scope_timer.hpp` | RAII 计时器与当前时间工具，`src/utils/include/utils/scope_timer.hpp:39` |
| `src/utils/include/utils/eigen_alias.hpp` | Eigen 类型别名与位姿定义，`src/utils/include/utils/eigen_alias.hpp:15` |
| `src/utils/include/utils/type_utils.hpp` | 返回码/栅格类型枚举与机器人状态，`src/utils/include/utils/type_utils.hpp:16` |
| `src/utils/include/utils/yaml_loader.hpp` | YAML 参数装载器，`src/utils/include/utils/yaml_loader.hpp:25` |
| `src/utils/include/utils/fmt_eigen.hpp` | Eigen 类型的 fmt 格式化特化，`src/utils/include/utils/fmt_eigen.hpp:34` |
| `src/utils/include/utils/plotter.hpp` | UDP JSON 绘图器（调试用，当前无调用点），`src/utils/include/utils/plotter.hpp:14` |
| `src/utils/include/utils/lbfgs.hpp` | L-BFGS 优化实现（第三方单头），`src/utils/include/utils/lbfgs.hpp:8` |
| `src/utils/include/utils/expected.hpp` | tl::expected 实现（第三方单头），`src/utils/include/utils/expected.hpp:14` |
| `src/utils/include/utils/matchit.h` | matchit 模式匹配库（第三方单头，**无人使用**），`src/utils/include/utils/matchit.h:16` |
| `src/utils/src/utils.cpp` | **空文件（0 字节）**：库为纯头文件实现，因此没有任何实现需要放在这里，`src/utils/src/utils.cpp:1` |
| `src/utils/CMakeLists.txt` | 库 target、include 可见性与链接，`src/utils/CMakeLists.txt:7` |
