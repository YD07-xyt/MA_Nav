# 地形分析：多层表面片 + 本体包络 改造记录

## User Intent

用户目标：
- 过滤限高杆（净空实测约 250 mm）与坡面（11°/15°），保留 20 cm 台阶（机器人爬不上，能爬的最大台阶 <10 cm）。
- 高台策略保"甲"：只台阶边沿一圈判障碍，高台内部可通行。
- 2D 里"未知"不写障碍。
- 接受把 terrain 的 min/max + 间隙分析整套换成"多层片 + 本体包络"。
- 参考 `3rd/traversability_generator3d/`（DFKI MLS 穿透性生成库）的思路。

前序状态：上一版"参考地面 z_ref + 三档判据"解决了限高杆平地段与台阶边沿漏检，但留下四类问题：
门形限高杆整段仍进 2D、台阶边大片白块、孤立白点、无法区分"无地面点"与"真障碍"。

## Scope

- `src/map/src/terrain_analysis.cpp`：`analyze` 判定主体重写，新增文件内的分位数滤波辅助函数。
- `config/map.yaml`：仅更新 terrain 段注释，说明 `max_step_height` / `steep_threshold` 当前未使用。

## Out of Scope

- 不改 `ma_map.hpp` 的 2D 三态回写（未加"障碍 2 帧确认"，本轮的"未知不写障碍"已抑制大部分噪点）。
- 不改 `terrain_analysis.hpp` 接口、不改 `ma_map.hpp` 调用点。
- 不改 3D 概率层、射线清除、遗忘逻辑、拖影逻辑。
- 不改 `robot_height` / `kernel_size` 的值（`robot_height` 沿用 0.24）。
- 不引入新配置项（新阈值用函数内 `constexpr`，遵循最小改动）。

## Explorer Findings

### Files inspected

- `3rd/traversability_generator3d/src/TravGenNode.hpp`、`TraversabilityGenerator3d.cpp`、`TraversabilityConfig.hpp`、`README.md`、`gui/config/parameters.yaml`
- `src/map/src/terrain_analysis.cpp`、`include/map/terrain_analysis.hpp`
- `src/map/include/map/ma_map.hpp`、`config/map.yaml`
- `docs/ai_refactor_records/场地.md`

### Active logic path

`/lio/cloud_world` → rog_map 3D 概率栅格 → `boxSearch(OCCUPIED)` → `TerrainAnalyzer::analyze` → `terrain_map_` / `terrain_free_` → 2D 三态回写 → `occupancy_2d`。

### Key findings

1. traversability_generator3d 的核心是 **MLS（多层表面）**：每格存一组垂直"片"，地面 / 悬空横梁 / 墙各自成片；判定用**本体 OBB 包络**（`buildBodyPolyhedron`：底面 = 地面 + maxStepHeight，顶面 = + robotHeight）与片求交；从机器人位置**波前扩展**传播地面参考；支撑不足判 `UNKNOWN` 而不是 `OBSTACLE`。
2. 该库**不能直接接入**：依赖 PCL + CGAL + Boost.Serialization + OpenSceneGraph + Rock base-types + Rock maps(slam-maps) + orogen typelib，且只消费 MLS、不含建 MLS 的部分。
3. 上一版 terrain 的四个问题共同根源：用"格内最低点/最高点"当唯一信息，无法表达"同一柱里既有地面又有悬空结构"。
4. `robot_height` 原本只用于"间隙 ≥ 车高则放行"。若照搬该库的 `maxStepHeight + robotHeight` 包络，会与 250 mm 净空冲突（0.05+0.24 > 0.25）。因此本实现**不引入 maxStepHeight 到包络底部**，包络从地面起算。

### Recommended modification boundary

只改 `terrain_analysis.cpp` 的判定部分，接口与 2D 回写保持不变。

## Modifier Changes

### Files changed

- `src/map/src/terrain_analysis.cpp`
- `config/map.yaml`（仅注释）

### Key changes

1. **新增文件内辅助函数 `quantileFilter`**：邻域分位数滤波，忽略 NaN。用 25% 分位而不是中值，使台阶边沿的参考地面落到低处地面，而不是低地与台面的中间值。
2. **垂直连通段切分为多层片**：每柱点按 z 排序，相邻差 > `kPatchGap`(0.12) 断开成另一片，得到 `{bottom, top}` 列表。
3. **地面参考 `z_ground`**：只有"最低片厚度 ≤ `kGroundPatchMaxThickness`(0.12)"的柱才算观测到地面；悬空厚片（横梁）和竖直结构（墙/杆/台阶立面）不参与统计，避免污染邻域。再对该候选场做 25% 分位滤波。
4. **判定（两类障碍 + 未知）**：
   - a. 贴地柱（片底 ≤ 地面 + `kGroundedTol` 0.06）且 `max_z - 地面 ≥ kVerticalStructureHeight`(0.12)：从地面往上算最大可钻空隙，`< robot_height` 判**障碍**（墙、杆、台阶立面、立柱）。
   - b. 悬空片下沿 `< 地面 + kOverheadBlockMax`(0.12) → **障碍**（过低悬挂）。
   - c. 悬空片下沿 ∈ [`0.12`, `robot_height`) → **灰区，不写**（2D 保持原值）。
   - d. 其余 → 可通行，但要再通过第 5 条。
5. **悬空结构横向连续性**：判为可通行的柱，若其悬空片在 8 邻域里有同高度（±0.08）片底的柱少于 3 个 → 视为**孤立竖直物** → 障碍。这条专门保护细杆/稀疏立柱，防止它们被"可从下方通过"误放行。
6. 删除上一版的 `z_ref`（邻域最低地面）、`ground`/`ground_raw`、`gradient`；`medianFilter` 保留但已不再被调用；`config_.max_step_height`、`config_.steep_threshold` 不再被使用。

### Behavior preserved

- `analyze` 两个重载的签名、返回值语义（障碍格中心点云）、`free_out` 语义不变。
- 2D 三态回写、空闲 2 帧滞回、建图/导航分支、拖影逻辑不变。
- 平地、11°/15° 坡面：贴地薄片 → 可通行（与改造前一致）。
- 20 cm 台阶立面：竖直结构 → 障碍（与改造前一致）。
- 隧道/桌下（净空远大于车高）：可通行（与改造前一致）。

### Behavior intentionally adjusted

- **限高杆**：不再依赖"这一格有没有地面点"。横梁片是独立片，其下沿 0.25 ≥ `robot_height` 0.24 → 可通行；即使横梁下方地面点缺失（该柱只有横梁片）也判可通行。这是本轮的核心目标。
- **无观测柱**：不再因为"最低点被抬高"而判障碍，改为不写（未知），2D 保持原值。
- **高台内部**：`z_ground` 由台面自身薄片提供 → 判可通行（甲策略成立）。
- **灰区**（悬空下沿 0.12~0.24）：不下结论、不写障碍，也不写空闲。
- **细杆**：新增横向连续性判据，稀疏垂直点也能判障碍（旧的 max_gap 判据在 `robot_height` 0.24 下会漏）。

### Notes

- 新增阈值均为函数内 `static constexpr`：`kPatchGap` 0.12、`kGroundPatchMaxThickness` 0.12、`kGroundedTol` 0.06、`kVerticalStructureHeight` 0.12、`kOverheadBlockMax` 0.12、`kOverheadSupportMin` 3、`kOverheadSupportTol` 0.08。调参优先动 `kPatchGap` / `kOverheadSupportMin`。
- 未包含该库的"波前扩展"（用户本轮只要 1~4 项），地面参考仍按每帧邻域统计，不跨帧记忆。
- `robot_height` 语义已变为"可通行所需最小净空 + 灰区上界"，yaml 注释已同步。

## Auditor Review

### Checks performed

- [x] 关键路径检查：`analyze` 两个重载、`update_terrain` 调用点、`free_out` 消费方
- [x] diff 检查：源码只有 `terrain_analysis.cpp`，`map.yaml` 只有注释
- [x] grep 检查：确认无残留 `ground_raw` / `z_ref` / `gradient` / `kObstacleBand` 引用（仅历史注释块内保留）
- [x] 判定表验证：用 Python 复刻判据，对 14 个典型柱型逐一验证（平地、坡面、墙、细杆稀疏、台阶立面、台面内部/边沿、限高杆有/无地面、限高杆端部、矮悬挂、隧顶）
- [x] 编译构建：用户授权后执行 `colcon build --packages-select ma_nav`，通过（无报错、无告警，`terrain_analysis.cpp.o` 已重新生成）
- [ ] 实车/Gazebo 功能验证：未执行

### Issues found

- 逻辑验证时发现并修复一个回归：稀疏细杆（垂直点间距 > `kPatchGap`）会被切成多个悬空片并被判"可从下方通过"。已用横向连续性判据修复。
- 限高杆端部 1~2 格会判障碍（横向同伴不足），属预期代价。
- 坡面若因掠射导致片厚接近 0.12，可能触发竖直结构判据；需实测确认，必要时提高 `kVerticalStructureHeight`。
- `medianFilter`、`max_step_height`、`steep_threshold` 成为未使用项，按最小改动原则保留。

### Final result

PASS（编译通过 + 判定表验证通过，功能待实测）
