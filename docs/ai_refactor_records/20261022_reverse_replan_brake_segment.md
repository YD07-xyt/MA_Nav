# 反向重规划刹车段（方案 B） 改造记录

## User Intent

机器人跟踪中目标点变为速度反方向时，重规划轨迹会沿当前速度方向先长距离减速再折返，高速时减速段可能撞障碍。实际底盘加减速很快，希望快速刹停后反向运动。用户确认：

- 不采用方案 A（规划起点速度置零、靠电控硬反向），以免对机器人冲击过大。
- 采用方案 B：参考 navkit backup 算法，沿速度方向在 ESDF 上射线检测得到可用距离 L_geo；空间不够时把减速度提到 v²/(2·L_geo)，上限为底盘真实能力；仍不够则立即发零速急停。
- 起点速度不使用里程计速度。

已验证：把 MPC `u_max` 提到 10 后减速段仍很长，说明长减速来自规划参考本身，而非 MPC 限幅。

## Scope

- `FsmReplan::plan` 的起点速度来源、反向判定、刹车段生成、从刹停点重新规划。
- `MAsplineOutput` 增加刹车段及统一采样接口。
- MPC 轨迹接口与控制回调对刹车段的采样、跟踪游标、急停处理。
- 折叠事件时间、横向偏差检查对刹车段的偏移。
- `planner.yaml` / `config.hpp` 新增刹车参数。

## Out of Scope

优化器代价与约束、时间分配、MPC 权重与约束、`one_plan`、`get_safe_pos` 算法、电控下游。

## Explorer Findings

### Files inspected

- src/planner/replan/src/fsm_replanner.cpp、include/replan/fsm_replanner.h
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h、traj_optimizer.h、cost.hpp、problem.md
- src/planner/controller/include/controller/traj_interface.hpp
- src/ros2/src/ros2_node.cpp、include/ros2/ros2_node.h、include/ros2/config.hpp
- src/map/include/map/grid_map.hpp
- config/planner.yaml
- 3rd/navkit_navigation（backup 刹车轨迹）、3rd/DDR-opt（起点贴障时阈值放宽）

### Active logic path

`planner_callback → plan_omni → FsmReplan::plan → path_planning + ma_opt_.optimize → ma_traj_interface_ → controller_callback → mpc_.solve → /cmd_vel`

### Root cause

1. 反向重规划把当前速度作为样条起点硬边界，优化器只能沿原方向平滑减速再折返，形成长减速段。
2. （上线后发现）刹车段与反向样条在空间上重合。控制器按位置搜索跟踪游标，又被限制在刹车段内；MPC 跟踪滞后，机器人到不了刹停点，游标停在刹车段末尾附近，参考为 v=0，机器人停住。之后给新点时，规划器按位置投影起点速度会落回刹车段，取到前向速度，可能再生成一次前向刹车段，导致仍不动。

### Data flow

`result_.ma_spline_traj`（brake + trajectory）同时用作：规划器下一次的热启动来源、控制器参考、折叠事件和可视化。局部时间 `[0, brake.duration)` 为刹车段，之后为样条。

### Risk notes

- `brake_acc_max` 若大于底盘能力或 MPC `u_max`，参考不可跟踪。
- 急停按刹车段时长发零速，依赖底盘自身响应。

### Recommended modification boundary

只改规划参考的生成与控制端的采样/游标，不改优化器和 MPC 本体。

## Modifier Changes

### Files changed

- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h
- src/planner/replan/include/replan/fsm_replanner.h
- src/planner/replan/src/fsm_replanner.cpp
- src/planner/controller/include/controller/traj_interface.hpp
- src/ros2/include/ros2/ros2_node.h
- src/ros2/src/ros2_node.cpp
- src/ros2/include/ros2/config.hpp
- config/planner.yaml

### Key changes

1. `BrakeSegment {duration, p0, v0, acc, emergency}` 挂在 `MAsplineOutput.brake`；新增 `total_duration()`、`sample_xy()`，刹车段用匀减速解析式，之后采样样条。
2. 起点速度：取机器人在上一条参考上的计划 v/a，不用里程计；无可用参考时按静止。
   - 上一条的刹车段尚未走完（发布后经过时间 < 刹车时长）时，按经过时间取刹车状态。
   - 否则只在样条部分做最近点投影。
3. 反向判定：起点速度与新路径首段夹角 > 90° 且速度 > 0.2 m/s 时，调用 `plan_brake_segment`，从刹停点（`get_safe_pos` 处理后）以零速重新搜索路径并优化。速度 ≤ 0.2 m/s 时直接零速起步。夹角 60°~90° 时只保留沿首段方向的速度分量。
4. `plan_brake_segment`：
   - 沿 v 方向在 ESDF 上射线检测，步长取 max(0.5·分辨率, 0.02 m)，最远检查到常规刹车距离 v²/(2·brake_acc)，得到 L_geo。
   - 起点本身贴障时，阈值取 min(safe_threshold, 0.85·起点距离)（同 DDR-opt）。
   - 放得下用 `brake_acc`；放不下用 v²/(2·L_geo)；超过 `brake_acc_max` 则取上限并置 `emergency=true`。
5. 控制端：
   - 新轨迹带刹车段时游标从 0 开始；刹车段内游标按控制周期的实际 dt 推进（单步限幅 0.05 s）。
   - 刹车段结束后才按位置搜索，搜索下界为刹车段终点，不会落回刹车段。
   - `emergency=true` 时，刹车时长内直接发零速（不经 MPC），结束后从样条上离机器人最近点开始跟踪。
6. 折叠事件时间加上刹车时长；可视化采样减去刹车时长。横向偏差检查把刹车起点接到参考路径前面。
7. 新参数 `replan_params.brake_acc: 6.0`、`brake_acc_max: 10.0`。

### Behavior preserved

非反向重规划流程、优化器、MPC 求解、到达判定不变。

### Behavior intentionally adjusted

- 重规划起点速度不再使用里程计速度。
- 反向时先匀减速刹停再反向规划；空间不足时急停。

### Notes

`last_trajectory_time_` 改为 `steady_clock`，在每次成功输出新轨迹时更新，用于规划端按时间推进刹车段。

## Auditor Review

### Checks performed

- [x] 关键路径检查（plan → 接口采样 → 控制回调游标/急停）
- [x] diff 检查
- [x] grep 检查（`t_track_`、`brake`、`last_trajectory_time_` 引用）
- [x] XML / launch / yaml 检查（planner.yaml 新增两项，config.hpp 读取）
- [x] `cmake --build /home/xyt/map/build/ma_nav` 编译通过
- [x] 用户在仿真中验证：反向给点后快速刹停并反向运动；刹停后再给点不再卡住

### Issues found

首版刹车段内按位置搜索游标，导致刹停后卡住、再给点仍不动（见 Root cause 2），已改为按时间推进修复。

### Final result

PASS
