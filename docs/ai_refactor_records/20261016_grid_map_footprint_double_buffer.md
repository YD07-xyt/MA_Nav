# 2D 栅格按真实占地写入 + 双缓冲交换 改造记录

## User Intent

先验地图 + 动态障碍时，机器人移动过程中障碍整体左右/上下抖一格；同时地图线程与规划线程并发读写同一份 GridMap。要求：按真实占地写入、双缓冲交换；地图实时更新、尽量减少多线程开销；不膨胀、不处理梯度、不加时间保持。

## Scope

- `MaMap::update_map` 中 terrain 点写入 2D 动态层的方式。
- `MaMap` 对外提供的 GridMap 改为双缓冲快照。
- `ros2_node.cpp` 中地图的读取（规划、可视化）与语义标记写入。

## Out of Scope

ESDF 算法与插值、膨胀、terrain 分析算法、时间滤波、配置文件、优化器。

## Explorer Findings

### Root cause

1. 跳格：terrain 格分辨率 0.1（`cfg_.resolution`），输出点为格中心 `0.1·k + 0.05`；grid_map 分辨率 0.05、原点 -6，格中心正好落在 0.05 细格边界上。`posToIndex` 用 `static_cast<int>` 截断，terrain 窗口原点随机器人位置变化带来 float 误差，同一障碍在相邻两格间跳动（float32 模拟：140/141 各约一半）。且一个 0.1 格只写 1 个 0.05 格，占地缺失 3/4。
2. 并发：`map_callback`（map 组）原地 `setMap` → `updateESDF` 改写 `occ_buffer_ / esdf_buffer_`，`planner_callback`（planner 组，MultiThreadedExecutor 并行）同时读同一对象；`grid_map_` 这个 shared_ptr 变量本身也在 map 线程写、control 组 `pub_callback` 读。

## Modifier Changes

### Files changed

- src/map/include/map/ma_map.hpp
- src/ros2/src/ros2_node.cpp
- docs/ai_refactor_records/20261016_grid_map_footprint_double_buffer.md

### Key changes

`ma_map.hpp`：

- 按真实占地写入：每个 terrain 点覆盖其所在 terrain 格 `[c - res/2, c + res/2)` 对应的全部 2D 细格；两端各向格内收 `0.25 × grid_res` 再 `posToIndex`，避开边界截断。0.1/0.05 时每点写 2×2 格。float32 模拟验证：同一障碍在机器人移动全程稳定落在 (140,141)。
- 双缓冲：
  - `grid_map_ptr_` 为后台缓冲，仅在 `back_mtx_` 下写（`update_map` 的 `setMap`、`set_global_map`、`set_semantics_polygon`）。
  - 写完调用 `publish_grid_map()`：拷贝后台缓冲为新快照，在 `front_mtx_` 下与 `front_grid_map_` 交换。
  - `get_grid_map()` 在 `front_mtx_` 下返回当前快照的 shared_ptr；快照发布后不再修改，调用方持有期间内容不变。
  - 新增 `set_semantics_polygon()`：语义标记写入后台缓冲并发布（原来直接改返回对象，改到快照上会被下一帧覆盖）。
  - 构造函数末尾发布初始快照，保证 `get_grid_map()` 非空。

`ros2_node.cpp`：

- `plan_omni`：每次规划取一次快照 `plan_map`，`fsm_replanner.plan`、`generate_fold_events`、`visualizeTunnelAndFold` 共用。
- `pub_callback`：改为 `mapInitialized` 后现取快照可视化，不再读 map 线程写的 `grid_map_` 成员。
- `clicked_point_callback`、`load_clicked_regions`：语义标记改调 `ma_map_->set_semantics_polygon`。

### Behavior preserved

- 地图每帧实时更新；规划/可视化总拿最新一份完整快照。
- ESDF 仍建在真实障碍上，未膨胀，插值与梯度未改。
- `get_occupancy_2d`、`save_global_map`、`load_global_map` 只读几何参数或 occupancy 矩阵，行为不变。

### Behavior intentionally adjusted

- 动态障碍 2D 占地由每点 1 格变为覆盖整个 terrain 格（0.1/0.05 时 2×2）。这是真实占地，不是膨胀；障碍在 ESDF 中会比原来"厚"约半个 terrain 格。
- 规划中途地图更新不再影响本次规划；新障碍最迟在下一次规划（33 ms 定时）生效。

### Cost

- 每次地图更新多一次 GridMap 拷贝（occ 1 B + esdf 8 B + semantics 1 B，560×320 约 1.8 MB）。
- 锁：`back_mtx_` 只在地图线程与语义写入间竞争；`front_mtx_` 只保护指针交换/读取，临界区为一次 shared_ptr 拷贝。

### Notes

- 嵌套前提：terrain 窗口原点是 `cfg_.resolution` 整数倍，且 grid 原点与 grid 分辨率能嵌套 terrain 格。修改 `grid_map.x_left / y_left / resolution` 或 `rog_map.resolution` 后需重新确认。
- `grid_map_` 成员仍在 `map_callback` 赋值并用于 `visualize_occupied_map`（同线程），未删除。
- `map_mutex_` 仍保护 clicked region 容器，未改。

## Auditor Review

### Checks performed

- [x] 关键路径检查（所有 `get_grid_map` 调用点、语义写入点）
- [x] `git diff --check` 通过
- [x] float32 数值模拟写入索引稳定
- [ ] 构建：未执行（需用户许可）
- [ ] 实车 / RViz 验证：未执行

### Final result

PASS（静态审查）
