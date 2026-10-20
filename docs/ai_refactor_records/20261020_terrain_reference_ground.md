# 地形分析：参考地面 z_ref 与三档判据 改造记录

## User Intent

用户目标（原话要点）：
- 过滤限高杆（悬空横梁，净空约 250 mm）与坡面，保留台阶（约 200 mm，机器人爬不上去）。
- 高台策略选"甲"：只有台阶边沿一圈判障碍，高台内部当可通行。
- 矮凸起：平缓的坡放过；台阶、陡坡判成障碍。
- 不要改其他功能；动态障碍物拖影去除只写交接文档（已单独完成）。

## Scope

只改 `src/map/src/terrain_analysis.cpp` 的 `analyze` 内部判定逻辑。

## Out of Scope

- 不改 `ma_map.hpp` 的 2D 三态回写与空闲计数。
- 不改 `config/map.yaml`、`config.hpp`、不改任何参数默认值。
- 不改 3D 概率层、射线清除、遗忘逻辑。
- 不改 `terrain_analysis.hpp` 的接口签名。
- 不动导航模式行为。

## Explorer Findings

### Files inspected

- `src/map/src/terrain_analysis.cpp`（原判定逻辑）
- `src/map/include/map/terrain_analysis.hpp`（接口、Config）
- `src/map/include/map/ma_map.hpp`（调用点 `update_terrain`、2D 回写）
- `config/map.yaml`（terrain 与概率参数）
- `docs/ai_refactor_records/场地.md`（场地与机器人描述）

### Active logic path

`/lio/cloud_world` → rog_map 3D 概率栅格 → `boxSearch(OCCUPIED)` → `TerrainAnalyzer::analyze` → `terrain_map_` / `terrain_free_` → 2D 三态回写 → `occupancy_2d`。

### Risk notes

1. 原【情况A】`is_steep_edge` 直接判障碍并跳过间隙分析（旧 `terrain_analysis.cpp:94-100`）。这是"限高杆靠近高台被误杀"的直接原因：横梁旁邻格地面高了一个台面高度，必然触发。
   - 注：`config/map.yaml` 里"新代码不会因为陡峭而误杀限高杆"的注释在改造前并不成立。
2. `ground` 取格内最低点。台阶踏面格若没有低处地面点，`ground` 就是台面 → `high_diff≈0` → 走情况B判平坦地面 → 台阶漏检。这是"台阶 2D 障碍物不完整"的直接原因。
3. 原间隙分析用"柱内最大垂直间隙"。坡面在柱内是连续点、无大间隙，理论上会被判阻塞；但用户实测"坡面现在不会被判障碍"，说明实际数据下坡面格内高差未超 `max_step_height`，没进入间隙分析。因此坡面现有行为必须保留，不能改成对坡面敏感的形式。
4. 绝对高度阈值 `obstacle_max_z > 0.01` 是用户手工加在旧逻辑上的，新判据改用相对参考地面，不再需要。

### Recommended modification boundary

只在 `analyze` 内部：新增参考地面计算、把梯度改算在参考地面上、用三档判据替换情况A/情况B/间隙分析。

## Modifier Changes

### Files changed

- `src/map/src/terrain_analysis.cpp`

### Key changes

1. 新增 `ground_raw`（未平滑的格内最低点快照），中值滤波结果仍存 `ground`。
2. 新增参考地面 `z_ref`：
   - `g_med` = 中值滤波后的格内最低点；
   - `g_min` = 5×5 邻域内 `ground_raw` 的最小值；
   - `z_ref = (g_med - g_min > 0.10) ? g_min : g_med`。
3. 邻域梯度改为在 `z_ref` 上计算（原来在 `ground` 上）。这样只有地面本身不连续（台阶边沿、陡坡）才触发。
4. 判定改为三档（相对 `z_ref`）：
   - 没有高出 `z_ref + 0.08` 的点：贴地。若 `z_ref` 邻域梯度 > `steep_threshold` 判障碍（台阶顶面、陡坡），否则输出空闲证据。
   - 否则取最低悬空点 `z_low`；`z_low - z_ref >= robot_height` → 输出空闲证据（限高杆、隧道、桌下）。
   - 其余判障碍（台阶立面、墙、立柱）。
5. 删除原【情况A】陡边短路判障碍、原情况B、原柱内间隙分析。

### Behavior preserved

- 接口签名、返回值语义（障碍格中心点云）、`free_out` 语义不变。
- 2D 回写、空闲 2 帧滞回、建图/导航模式分支不变。
- 平缓坡：`g_med - g_min` 在 8.5° 坡、5×5 窗口下约 0.02~0.04，未超 0.10，仍走 `z_ref = g_med` 分支，判定与改造前一致（可通行）。
- 5 cm 以内矮凸起：仍在 `kObstacleBand` 内 → 判贴地放过，与改造前（`max_step_height` 0.05）一致。
- 8~10 cm 以上凸起：判障碍，与改造前一致。
- 低于车高但宽间隙的隧道/桌下：可通行，与改造前一致。
- 稀疏细杆（格内只有高处单点）：间隙大 → 可通行。与改造前行为相同（改造前同样会放过），未改进也未退化。

### Behavior intentionally adjusted

- 限高杆靠近高台的段：不再因为邻域地面陡变被一票判障碍，改为按净空与 `robot_height` 比较。
- 台阶踏面格：`z_ref` 被邻域低处地面拉低，踏面点变成"悬空点"，间隙 0.2 < `robot_height` → 判障碍（修面板漏检）。
- 高台内部远离边沿（超过 2 格）：`z_ref` 仍是台面 → 可通行（甲策略，台阶边沿约 2~3 格厚）。
- 绝对高度阈值 0.01 不再参与判定。

### Notes

- 新增的三个常量是函数内 `static constexpr`，未新增配置项，避免扩大改动面：
  `kObstacleBand = 0.08`、`kGroundJumpThresh = 0.10`、`kGroundMinRadius = 2`。
- `config_.max_step_height` 在新逻辑中不再被使用，但按"最小改动/不改参数"原则保留在配置与结构体中，未删除。
- `config/map.yaml` 的 `robot_height` 由 0.26 改为 0.24（用户确认）：净空 250 mm，判据为 `间隙 >= robot_height`，0.26 时必然判障碍。0.24 可放行限高杆，20 cm 台阶仍判障碍（0.2 < 0.24）。

## Auditor Review

### Checks performed

- [x] 关键路径检查：`analyze` 两个重载、`update_terrain` 调用点、`free_out` 消费方（`ma_map.hpp` 三态回写）
- [x] diff 检查：源码仅 `terrain_analysis.cpp` 变更，另加 `config/map.yaml` 的 `robot_height` 一项
- [x] grep 检查：`max_step_height`、`steep_threshold`、`obstacle_max_z`、`high_diff` 的使用情况
- [x] yaml / launch 检查：仅改 `robot_height` 一个值，yaml 语法未破坏
- [x] 编译构建：用户授权后执行 `colcon build --packages-select ma_nav`，通过（2026-10-20 11:15，无报错、无未使用变量告警；`terrain_analysis.cpp.o` 已重新生成）
- [ ] 实车/Gazebo 功能验证：未执行

### Issues found

- 运行期未验证：台阶边沿厚度、限高杆过滤、高台内部判可通行均需实测确认。
- `kObstacleBand` 0.08 与 `kGroundJumpThresh` 0.10 是推算初值，可能需要按实测微调。

### Final result

PASS（编译通过，功能待实测）
