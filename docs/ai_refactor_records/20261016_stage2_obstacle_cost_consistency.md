# stage2 避障代价一致化 改造记录

## User Intent

修复 stage2 L-BFGS 线搜索失败（`Line search reaches the maximum try number`）及轨迹可能穿过障碍物的问题。本次只重写 stage2 的避障分支，不做其他更改。

## Scope

`RobotIntegralCost::operator()` 中 `split_esdf == true` 的 ESDF 避障分支。

## Out of Scope

stage1 避障、速度/加速度约束、yaw 约束、TimeCost、L-BFGS、两阶段失败回退、错误码处理、碰撞复检、积分步数、planner.yaml。

## Explorer Findings

### Files inspected

- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp
- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h
- src/utils/include/utils/lbfgs.hpp
- src/map/src/grid_map.cpp
- docs/nav/开源参考/RM2025中科大导航.md
- 3rd/pb_minco_smoother/include/pb_minco_smoother/minco_optimizer.hpp
- 3rd/navkit_navigation/minco_planner/src/traj_opt/minco_optimizer.cpp
- 3rd/DDR-opt/back_end/src/optimizer.cpp

### Active logic path

OMNI_XY → optimize_xy → optimize_2d_stage(stage2, split_esdf=true) → RobotIntegralCost<2> 的 split 分支。

### Root cause

1. 势谷分支把 `violation` 替换为 `step * sqrt(orth_norm)`，该值与距离 d 无关，但梯度仍沿 `grad_pos` 方向，代价与梯度不一致；阈值 0.5 两侧代价跳变。线搜索的 Armijo/Wolfe 条件因此难以满足。
2. split 分支没有 `violation > 0` 前置判断，远离障碍但位于 ESDF 脊线的点也会受到最多 `0.05 * w_obs` 的假惩罚。
3. 穿越薄墙时墙内负 ESDF 同样梯度接近零，会进入势谷分支，惩罚被压到不超过一个分辨率（0.05），远小于真实违反量 `safe_distance - d`，stage2 的避障项压不住时间项和能量项，轨迹可能穿墙。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。梯度方向仍是投影后的“伪梯度”（中科大方法本身如此），线搜索失败概率预计下降，但不保证消失。

### Recommended modification boundary

只改 split 分支内部，参考 pb_minco_smoother 的做法：代价保持真实违反量，势谷只调整梯度模长。

## Modifier Changes

### Files changed

- src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp
- docs/ai_refactor_records/20261016_stage2_obstacle_cost_consistency.md

### Key changes

- 分支条件 `else` 改为 `else if (violation > 0.0)`：仅在违反安全距离时介入。
- 势谷分支（单栅格探测差分 < 0.5）：不再替换 `violation`，梯度改为 `direction * sqrt(orth_norm)`。
- 自由分支（差分 ≥ 0.5）：梯度归一到 1，代价不变。
- 新增：横向分量模长 ≤ 1e-6 时，梯度退回完整 ESDF 梯度。
- 代价始终为 `w_obs * smoothedL1(safe_distance - d)`。

### Behavior preserved

stage1 完整梯度路径、速度/加速度/yaw 约束、单栅格探测步长、阈值 0.5、切向投影、权重与配置均不变。

### Behavior intentionally adjusted

- stage2 避障代价在势谷与穿墙处不再被压小，与真实违反量一致。
- 未违反安全距离的点不再产生避障代价和梯度。
- 势谷处的增强从“改代价”变为“改梯度模长”。

### Notes

上一版记录 20261015_stage2_esdf_step.md 中“差分小于 0.5 时违反量为分辨率乘 sqrt(横向梯度模长)”的行为被本次替换。

后续待确认项（本次未做）：
- 线搜索类错误码（MAXIMUMLINESEARCH、WIDTHTOOSMALL 等）视为部分成功并保留 stage2 结果。
- 输出前接入 check_trajectory_collision，按不超过一个栅格的空间步长采样，失败时回退 stage1 / 上一条安全轨迹。
- integral_num_steps 或 time_resolution 加密采样。
- JPS no path 与地图通道临时被封的问题单独排查。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] 用户允许范围内的静态检查：git diff --check 通过
- [ ] XML / launch / yaml 检查：本次未涉及
- [ ] 如需构建，已取得用户明确许可：未申请、未执行构建

### Issues found

静态审查未发现问题；未进行编译或运行验证。

### Final result

PASS（静态审查）
