# stage2 ESDF 探测步修正 改造记录

## User Intent

按中科大原文修正第二次优化：探测步用一个栅格，自由方向保持安全距离违反量并归一梯度，势谷方向用分辨率乘 sqrt(梯度模长) 作为违反量。第二次失败时保留第一次结果。第二次不再使用段时间均值约束。

## Scope

轨迹优化 ESDF 代价、地图 ESDF 适配器、两阶段失败回退、planner.yaml 的 stage2 段时间权重。

## Out of Scope

不改第一次优化、不改 L-BFGS、不改障碍和动力学权重、不加输出碰撞检查。

## Explorer Findings

### Files inspected

- docs/nav/开源参考/RM2025中科大导航.md
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h
- src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp
- config/planner.yaml

### Active logic path

OMNI_XY 走 optimize_xy。stage2 的 split_esdf 去掉切向后沿法向探测。

### Data flow

GridMapESDF 把 GridMap 的距离、梯度和分辨率交给 RobotIntegralCost。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。势谷分支仍是软惩罚，不保证轨迹距离大于 safe_distance。

### Recommended modification boundary

只改探测步、违反量尺度、stage2 失败回退和段时间约束开关。

## Modifier Changes

### Files changed

- src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h
- config/planner.yaml

### Key changes

探测步改为地图分辨率。差分小于 0.5 时违反量为分辨率乘 sqrt(横向梯度模长)。差分不小于 0.5 时违反量保持 safe_distance 减距离，梯度归一到 1。二维和三维的 stage2 失败都回退到 stage1。stage2 的 weight_mean_time 改为 0。

### Behavior preserved

stage1 的完整梯度和 0.9 到 1.1 段时间约束不变。障碍、速度、加速度权重不变。

### Behavior intentionally adjusted

stage2 不再因为线搜索失败丢掉 stage1。段时间均值约束只作用于 stage1。

### Notes

ESDFInterface 的既有方法调用 Impl，但代价直接调用具体 ESDF，因此分辨率只加在 GridMapESDF。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [ ] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

无。

### Final result

PASS
