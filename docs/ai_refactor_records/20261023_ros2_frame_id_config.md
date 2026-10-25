# ros2 发布 frame_id 统一并写入 yaml 改造记录

## User Intent

检查 `src/ros2/` 下所有发布消息的 frame_id 是否统一，并把它改为从 yaml 加载。

## Scope

- `config/planner.yaml`
- `src/ros2/include/ros2/config.hpp`
- `src/ros2/include/ros2/ros2_node.h`
- `src/ros2/src/ros2_node.cpp`
- `src/ros2/include/ros2/misc/visualizer.hpp`

## Explorer Findings

- 改动前，工作区里有一处未提交的改动：`ros2_node.h` 中的 `frame_id="map"` 写死在代码里。凡是传了这个成员的调用（2D 栅格、全局/优化路径、隧道折叠可视化、点选区域、`map_viz_callback`），发布的都是 `map`。
- 但 `map_viz_callback` 内部没有把 frame_id 传下去：
  - `vecEVec3fToPC2` 走默认值，`/ma_nav/map/occ|unk|inf_occ|inf_unk|frontier` 实际发布的仍是 `world`；
  - `visualizeBoundingBox`、`visualizeText`、`visualizePoint` 同样走默认值，`/ma_nav/map/map_bound` 发布的是 `world`；
  - `visualizeText` 函数内直接写死了 `"world"`。
- 结果是同一个节点混用 `map` 和 `world`。上游 LIO 的 `/lio/cloud_world` 和 `/lio/base_odom` 的 header 都是 `world`，TF 也只有 `world→base_link`，没有 `map` 帧。因此 RViz 中 Fixed Frame 选 `world` 时，栅格和路径会显示不出来。
- 本节点不做 TF 变换，坐标直接按输入消息使用。所以发布用的 frame_id 必须和输入一致。
- `rog_map.visualization.frame_id`（`src/map/include/map/3d_occ_map/config.hpp:122`）只被加载，没有被使用，本次不改。

## Modifier Changes

- `planner.yaml` 的 `ros2` 节新增 `frame_id: "world"`。
- `Config` 新增 `frame_id`，默认值为 `"world"`；yaml 中没有这个键时保持默认。
- `GlobalPlanner2d::frame_id` 去掉写死的 `"map"`，在构造函数初始化列表中从 `config.frame_id` 赋值。成员声明位于 `nh` 之后、`visualizer` 之前，初始化顺序安全。
- `visualizeText` 新增 `frame_id` 参数，默认值为 `"world"`，与其他函数一致，替换掉函数内写死的值。
- `map_viz_callback` 内的 5 处 `vecEVec3fToPC2`、4 处 `visualizeBoundingBox`、4 处 `visualizeText`、1 处 `visualizePoint` 全部改为显式传 `frame_id`。
- 点云、里程计、目标点这三个回调中，消息 `header.frame_id` 与配置不一致时告警一次。只告警，不做 TF 变换，也不丢弃消息。

## Verification

- [x] 静态检查：`visualizer.hpp` 中除注释外，`"world"` 只作为参数默认值出现；`ros2_node.cpp` 所有可视化调用都传入了成员 `frame_id`。
- [x] 编译构建：用户授权后执行 `colcon build --packages-select ma_nav`，通过，无 error 和 warning；`ros2_node.cpp.o` 为本次重新生成。
- [x] 短回放（bag `rosbag2_2026_10_23-16_26_25` 前约 6 s）：
  - `frame_id: "world"`：`/ma_nav/grid_map`、`grid_map_occ`、`grid_map_esdf`、`map/occ`、`map/unk`、`map/inf_occ`、`map/map_bound` 全部为 `world`，没有不一致告警。
  - 临时 yaml 设为 `frame_id: "map"`：上述话题全部变为 `map`；`/lio/cloud_world` 和 `/lio/base_odom` 各告警一次。临时 yaml 已删除。
- [ ] 路径类话题（`global_path`、`opt_path`、`route`、`yaw_profile`）需要下发目标后才会发布，本次回放没有覆盖到。它们在代码中都传了成员 `frame_id`。

## Rollback

还原上述 5 个文件。若删除 yaml 中的键，代码默认使用 `world`。
