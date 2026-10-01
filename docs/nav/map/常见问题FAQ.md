---
module: map
doc: 常见问题FAQ
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools
generated_at: 2026-10-02T00:30:00Z
status: draft
reviewer: 未审核
---

# map 模块 常见问题FAQ

## Q1：改了 map.yaml 里的参数，为什么地图行为没变？

**现象**：修改 `/home/xyt/map/src/MA_Nav/config/map.yaml` 后重启，行为与改前一致。

**原因（按可能性排序）**：

1. 改的键**不是代码实际读取的键**。本模块用
   `LoadParam(name_space + "/...", ...)`，前缀 `name_space` 默认为 `rog_map`，
   即 YAML 里的键必须写成 `rog_map` 前缀 + 子路径（点号形式：rog_map 点 子路径）。
   锚点：`src/map/include/map/3d_occ_map/config.hpp:67`、
   `src/map/include/map/3d_occ_map/config.hpp:137`。
2. 参数有**代码默认值兜底**。如果删除或写错键，不会报错而是退回默认值。
   典型：`rog_map.ros_callback.odom_timeout` 的 YAML 值为 0.5，
   代码默认值却是 0.05（见 `配置说明.md` 的不一致项表）。
3. **改错了文件**：本模块读的是 map.yaml，但该路径本身由 planner.yaml 指定。
   锚点：`config/planner.yaml:10`。

**处理**：改完后用 `配置说明.md` 的「行号」列核对读取点；重启节点。

## Q2：点云话题有数据，但地图不动

**现象**：`/lio/cloud_world` 有数据，`/ma_nav/map/occ` 不更新。

**原因**：`update_cloud` 有两道早退检查，任一不满足就丢帧，只打印一条 warn。
两条 warn 的确切位置如下：

| 日志文本 | 触发条件 | 代码位置 |
|---|---|---|
| No odom received, skip cloud callback. | 从未收到里程计 | `src/map/include/map/ma_map.hpp:104` |
| Odom timeout, skip cloud callback. | 最近一次里程计早于 `odom_timeout` | `src/map/include/map/ma_map.hpp:110` |

第一道检查在 `src/map/include/map/ma_map.hpp:103`，
第二道在 `src/map/include/map/ma_map.hpp:109`。

**处理**：确认里程计话题名与 `rog_map.ros_callback.odom_topic` 一致
（YAML 值为 `/lio/base_odom`，代码默认值是 `/lio/odom`，
两者不同是本仓库已知的坑）；确认 `odom_timeout` 不是被退回的默认 0.05。

## Q3：一条告警一直刷：No point cloud input, check the topic name.

**现象**：地图始终保持空，日志每秒打印一次该告警。

**原因**：`map_empty_` 仍为真（从未成功处理过一帧点云），
且 `ros_callback.enable` 为真时按 1 秒节流打印。
锚点：`src/map/include/map/ma_map.hpp:56`。

**处理**：检查 cloud 话题名与 `rog_map.ros_callback.cloud_topic`
（YAML 值 `/lio/cloud_world`）是否一致。锚点：`config/map.yaml:1`。

## Q4：另一条告警：Unfinished frame cnt > 1, the map may not work in real-time

**现象**：地图能更新，但偶发该告警。

**原因**：上一帧还没处理完，`update_cloud` 又追加了新的未处理帧。
锚点：`src/map/include/map/ma_map.hpp:67`。

**处理**：这是**实时性不足的信号**，不是配置错误。需要降低点云频率、
缩小 `rog_map.raycasting.local_update_box` 或提高计算性能。

## Q5：想扩大/缩小 2D 规划栅格范围，改哪里？

**原因**：2D 栅格边界按「原点向四个方向的延伸量」定义：
x 属于负的 `x_left` 到正的 `x_right`，y 同理。
读取点：`src/map/include/map/3d_occ_map/config.hpp:208`；
初始化调用：`src/map/include/map/ma_map.hpp:29`；
边界语义注释：`src/map/include/map/grid_map.hpp:56`。

**处理**：改 `rog_map.grid_map.x_left` / `x_right` / `y_left` / `y_right`。
注意四项都会被读到；当前 YAML 值与代码默认值不同（见 `配置说明.md`）。

## Q6：rviz 里看不到未知区域

**原因**：未知区域可视化由 `rog_map.visualization.pub_unknown_map_en` 控制，
当前 YAML 值为 true，但**代码默认值是 false**——
一旦该项缺失或写错，未知区域可视化会静默关闭。
锚点：`src/map/include/map/3d_occ_map/config.hpp:121`。

## Q7：ESDF 相关查询在 ESDF 关闭时还能用吗？

**现象**：`rog_map.esdf.enable` 当前为 false，但仍可从 `ProbMap` 取到 ESDF 子地图。

**原因**：`ProbMap` 始终持有 `esdf_map_`，并提供访问器，
没有在接口层根据 `esdf.enable` 做分支。
锚点：`src/map/include/map/3d_occ_map/prob_map.h:112`、
`src/map/include/map/3d_occ_map/prob_map.h:120`。

**处理**：【待确认】关闭状态下的返回值语义未定义，见 `待确认清单.md`。

## Q8：修改了代码，之前的文档还能信吗？

**原因**：不能。文档 front-matter 里记录了 `evidence_digest`（证据摘要）
与 `git_rev`。代码或 YAML 一旦变化，重跑提取器后摘要就会变，
校验器会把该模块文档判为 `evidence.stale`。
锚点：`src/map/include/map/3d_occ_map/config.hpp:214`。

**处理**：重跑 `docs/tools/` 下的提取器与校验器，
只信任 `errors == 0` 且非 `stale` 的文档。

## 排查手段

| 手段 | 用法 | 代码位置 |
|---|---|---|
| 地图可视化 | 订阅 `/ma_nav/map/occ` 等话题 | `src/ros2/src/ros2_node.cpp:77` |
| 2D 栅格可视化 | 订阅 `/ma_nav/grid_map_occ` | `src/ros2/src/ros2_node.cpp:72` |
| 局部地图边界 | 订阅 `/ma_nav/map/map_bound` | `src/ros2/src/ros2_node.cpp:83` |
| 地图信息日志 | `writeMapInfoToLog` 写入日志文件 | `src/map/include/map/3d_occ_map/prob_map.h:107` |
| 耗时日志 | `writeTimeConsumingToLog` | `src/map/include/map/3d_occ_map/prob_map.h:105` |
| 地图信息打印 | `printMapInformation` | `src/map/include/map/3d_occ_map/sliding_map.h:64` |
