---
module: path_planning
doc: 常见问题FAQ
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T02:00:00Z
status: draft
reviewer: 未审核
---

# path_planning 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：改了 `config/planner.yaml` 里的路径参数，为什么本模块行为没变？

**现象**：改 YAML 后重启，规划的路径与之前完全一致。

**原因**：本模块根本不解析 YAML，参数必须以参数结构体的形式注入；
装载只在 ros2 侧的 `load_planner_config` 里发生
（`src/ros2/include/ros2/config.hpp:262-263`），
再由 replan 的 `set_param` 转发进来
（`src/planner/replan/include/replan/fsm_replanner.h:45`）。

**处理**：确认改的是 planner_config.path_planning_params 段
（`config/planner.yaml:23`），并确认调用链 ros2 → replan 侧 set_param 真的执行过
（`src/ros2/src/ros2_node.cpp:36`）。

## Q2：为什么代码里保留了 A*，实际却在跑 JPS？

**现象**：日志里的失败信息带 `JPS:` 前缀。

**原因**：调用方显式打开了 JPS 开关
（`src/planner/replan/src/fsm_replanner.cpp:69`），
门面按 `use_jps_` 三元选择搜索器
（`src/planner/path_planning/src/path_planning.cpp:41-42`）。

**处理**：若要回退到 A*，改调用方的开关；注意 A* 与 JPS 的起终点容错行为并不完全一致
（`src/planner/path_planning/src/search/jps.cpp:221-224`）。

## Q3：隧道（折叠）降速为什么没有生效？

**现象**：进入隧道前速度没有降到准备速度。

**原因**：降速区间来自 `detect_tunnel_intervals`；区间为空时函数直接返回，
一个速度上限都不会被改写
（`src/planner/path_planning/src/post_processing.cpp:401-403`）。
区间为空最常见的原因是地图没有隧道语义
（`src/map/include/map/grid_map.hpp:140-145`）。

**处理**：确认地图语义层是否被写入过；本模块没有任何「语义缺失」的告警。

## Q4：`path_planning` 返回 `std::nullopt` 的可能原因有哪些？

| # | 原因 | 证据 |
|---|---|---|
| 1 | 起点或终点落在膨胀区内（图内） | `src/planner/path_planning/src/search/astar.cpp:19-26` |
| 2 | 搜索超时（调用方给 5000 ms） | `src/planner/path_planning/src/search/astar.cpp:43-48` |
| 3 | 搜索耗尽开放集，无路径 | `src/planner/path_planning/src/search/astar.cpp:89` |
| 4 | JPS 起点 1.5 m 内没有可通行栅格 | `src/planner/path_planning/src/search/jps.cpp:226-231` |
| 5 | 时间分配后没有有效时间点 | `src/planner/path_planning/src/path_planning.cpp:58-61` |

【推断】第 3 条在 JPS 路径上等价于「开放集耗尽」，但 JPS 会额外打印根栅格与
扩展节点数，便于区分「真无路」与「起点被围死」
（`src/planner/path_planning/src/search/jps.cpp:346-353`）。

## Q5：轨迹的分段数为什么不是「总时长 ÷ time_resolution」？

**原因**：算出来的段数会先被 `min_traj_num` 抬、再被 `max_traj_num` 压
（`src/planner/path_planning/src/post_processing.cpp:261-262`），
因此短轨迹与长轨迹都会落在 `[min_traj_num, max_traj_num]` 区间内。

**处理**：如果希望段数与时长严格成比例，需要同时放开上下限；
注意 `max_traj_num` 的代码默认值（12）与 YAML 值（16）不同
（`src/planner/path_planning/include/path_planning/post_processing.h:73`）。

## Q6：为什么 replan 每轮开头都要对轨迹再查一次碰撞？

**原因**：被检查的是**上一轮**留下的轨迹：`plan()` 在开始处就调用成员检查函数，
对上一轮结果的剪枝路径按采样步长做整段复检，用来决定是否需要重规划
（`src/planner/replan/src/fsm_replanner.cpp:41-42`）。

它与本轮 MINCO 优化的结果无关：若本轮优化失败，`plan()` 直接返回错误，
调用方丢弃结果且不发布轨迹
（`src/planner/replan/src/fsm_replanner.cpp:101-104`）。

## 参数类问题

| 问题 | 原因 | 锚点 |
|---|---|---|
| 改 safe_threshold 后只有后处理变了 | 该值需要经参数注入同时下发给两个搜索器 | `src/planner/path_planning/src/path_planning.cpp:12-16` |
| 起终点速度不是 YAML 参数 | 由运行时经速度注入接口写入，默认 0.0 | `src/planner/path_planning/src/path_planning.cpp:17-22` |
| distance_weight / yaw_weight 调了没反应 | 两个字段**全仓库没有消费者**（只有声明与装载），YAML 注释描述的「加权长度计算」实际由 rotation_penalty_weight 完成 | `src/planner/path_planning/include/path_planning/post_processing.h:75-76` |
| unfold_time / fold_margin 在本模块找不到使用点 | 它们由 ros2 侧的云台逻辑读取，不属于路径规划 | `src/ros2/src/ros2_node.cpp:318-320` |

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 日志通道 | 过滤 `path_planning` 通道即可看到本模块全部日志 | `src/utils/include/utils/logger.hpp:121` |
| 计时段 | 规划入口处有计时对象，但默认不打印，需要改第二参 | `src/planner/path_planning/src/path_planning.cpp:36` |
| 失败原因 | A* 只打印「起点/终点碰撞」「超时」 | `src/planner/path_planning/src/search/astar.cpp:46` |
| JPS 诊断 | 失败时打印根栅格、占用邻居数、扩展节点数 | `src/planner/path_planning/src/search/jps.cpp:346-353` |
| 原始路径 | 可视化器发布 `raw_path`，可与优化后路径对比 | `src/ros2/src/ros2_node.cpp:298` |
