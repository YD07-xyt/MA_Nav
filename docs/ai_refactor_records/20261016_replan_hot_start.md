# 重规划热启动 改造记录

## User Intent

修复重规划后起点附近出现圆弧、轨迹扭曲的问题。按 navkit 做法实现热启动；起点被挪开时速度只保留沿新路径首段方向的分量；顺手修复 MPC 首帧速度多余的坐标旋转。不改其他内容。

## Scope

- `FsmReplan::plan` 中起点速度/加速度的来源。
- `GlobalPlanner2d::controller_callback` 首帧 MPC 状态速度。

## Out of Scope

碰撞复检、错误码放宽、`one_plan`、`get_safe_pos` 算法、时间分配算法、优化器、配置文件、中科大式旧轨迹拼接。

## Explorer Findings

### Files inspected

- src/planner/replan/src/fsm_replanner.cpp
- src/planner/replan/include/replan/fsm_replanner.h
- src/planner/path_planning/src/path_planning.cpp
- src/planner/path_planning/src/post_processing.cpp
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h
- src/planner/controller/include/controller/traj_interface.hpp
- src/ros2/src/ros2_node.cpp
- 3rd/navkit_navigation/minco_planner/src/minco_core/minco_planner.cpp（determinePlanningState / prepareHotStart）
- /home/xyt/NavX/src/location/Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp、src/lio/ESKF.cpp

### Root cause

1. 重规划起点速度直接使用里程计瞬时速度作为样条硬边界，起点加速度强制为 0。转弯时瞬时速度仍沿旧方向，样条被迫先沿旧方向再折回新路径，起点附近形成圆弧。
2. `get_safe_pos` 挪开起点后，起点速度仍为机器人实际速度，与挪开后的位置和新路径方向不匹配（靠近障碍时）。
3. Super-LIO `/lio/base_odom` 的 `twist.linear` 为世界系速度（ESKF `new_v = v + R*acc*dt + g*dt`，发布时未旋转；`child_frame_id: base_link` 仅为标注）。规划侧当世界系使用是正确的；MPC 首帧却按机体系再旋转一次 yaw，是错误的（仅影响首帧）。

### Risk notes

热启动依赖上一条样条 `result_.ma_spline_traj`，该样条是重规划器最后一次成功输出，即 MPC 正在跟踪的轨迹。

## Modifier Changes

### Files changed

- src/planner/replan/src/fsm_replanner.cpp
- src/ros2/src/ros2_node.cpp
- docs/ai_refactor_records/20261016_replan_hot_start.md

### Key changes

`FsmReplan::plan`：

- 记录挪开前的机器人位置 `robot_pos`，`start_moved` 表示 `get_safe_pos` 是否挪动了起点。
- 热启动：上一条样条有效时，按 `projection_search_resolution`（0.02 s）在旧样条全时长上搜索机器人最近投影时刻 `best_t`，取 `v(best_t)`、`a(best_t)` 作为起点速度、加速度。
- 冷启动回退条件（用户确认 1、2）：
  - 投影时刻越界（`best_t <= 0` 或 `best_t >= duration`）；
  - 旧速度模长 > 0.1 且与新路径首段方向夹角 > 60°（`dot < 0.5`，同 navkit）。
  - 冷启动：里程计速度（世界系），加速度 0。
- 时间分配起点速度（`set_velocity` → `vmax.front()`）同样使用热启动速度。方向校验在路径生成后进行，若校验回退冷启动，只影响样条边界，时间分配已按热启动速度完成。
- 起点被挪开：速度投影到新路径首段方向（`optimized_path[0] → [1]`），负投影置 0；加速度置 0。
- 最终写入 `ma_intput.start_vel / start_acc`，覆盖 `from_path_planning_trajectory` 默认值。
- 增加 debug 日志：hot / moved / v / a。

`GlobalPlanner2d::controller_callback`：

- 首帧 MPC 状态速度直接使用里程计世界系速度，去掉按 yaw 的机体→世界旋转。

### Behavior preserved

- 首次规划（无旧样条）仍为冷启动，行为与原来一致（里程计速度、加速度 0），仅当起点被挪开时新增速度投影。
- `one_plan`、重规划触发条件、`get_safe_pos`、优化器、碰撞检查、配置均不变。
- `current_pose.wz` 仍作为 yaw 速率传入 `set_velocity`。

### Behavior intentionally adjusted

- 重规划起点速度/加速度改为旧样条在投影时刻的状态，起点加速度不再强制为 0。
- 起点被挪开时，起点速度不再是机器人实际速度。
- MPC 首帧速度不再被旋转。

### Notes

- 未启用 `minco_traj_validity_duration`、`minco_traj_continuity_threshold` 作为回退条件（用户只选择了 1、2）。
- navkit 的跟踪误差/速度误差检查在其源码中实际被禁用（仍返回 HOT_START），本次未引入。
- 已知边界：方向校验回退冷启动时，时间分配的起点速度仍为热启动速度模长，与样条边界速度模长可能不同；如需严格一致，可将方向校验前移到 `path_planning` 之前（需要先有路径方向，目前不可行，或改为二次调用）。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查：`git diff --check` 通过
- [ ] XML / launch / yaml 检查：本次未涉及
- [ ] 构建：用户选择不编译，未执行

### Issues found

静态审查未发现问题；未进行编译或运行验证。

### Final result

PASS（静态审查）
