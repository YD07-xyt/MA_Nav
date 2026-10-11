---
module: ros2
doc: 常见问题FAQ
git_rev: 8f98daee5bc7
evidence_digest: be346b5265f1410d
worktree_dirty: false
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T14:35:04Z
status: draft
reviewer: 未审核
---

# ros2 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：为什么 `/cmd_vel_chassis` 没有输出？

**原因**：控制回调有三级前置条件，任一不满足就静默返回：
没有当前位姿、没有有效轨迹、MPC 求解失败
（`src/ros2/src/ros2_node.cpp:131-136`、`src/ros2/src/ros2_node.cpp:179-183`）；
即使求解成功，指令越界（±20）或含 NaN 也会被丢弃
（`src/ros2/src/ros2_node.cpp:229-240`）。

**处理**：按顺序看日志：先确认有没有「MPC solve failed」，
再确认是否有有效轨迹（需要先规划成功并下发新轨迹，
`src/ros2/src/ros2_node.cpp:308-313`）。

## Q2：目标点发了，但规划没有反应？

**原因**：目标回调在**地图未初始化**时直接忽略并打警告
（`src/ros2/src/ros2_node.cpp:467-484`）；
另外与上次目标坐标完全相同的目标会被去重忽略
（`src/ros2/src/ros2_node.cpp:469-472`）。
规划入口还会在缺目标或缺当前位姿时直接返回
（`src/ros2/src/ros2_node.cpp:249-256`）。

**处理**：确认先有 `/lio/cloud_world` 让地图初始化（`mapInitialized` 置真），
再用不同坐标下发目标。

## Q3：为什么没有折叠动作，或者折叠时机不对？

**原因**：折叠事件完全由「轨迹 + 地图隧道语义」生成：
若轨迹上没有任何点落在隧道语义区，事件列表为空
（`src/ros2/src/ros2_node.cpp:928-930`）；
事件时间以折叠时间与裕量提前、展开时间与裕量延后
（`src/ros2/src/ros2_node.cpp:950-959`）。
折叠时间/展开时间/裕量来自**搜索后处理参数**，不是节点头里的常量。

**处理**：确认地图写入过隧道语义（见 [map 模块文档](../map/README.md)），
并检查 `path_planning_params` 里的折叠参数；
下发新轨迹时会跳过已过去的事件并把应有状态写进记账变量，但**补发折叠指令的那行已被注释**（`src/ros2/src/ros2_node.cpp:334-337`），因此状态只记账不发指令
（`src/ros2/src/ros2_node.cpp:329-338`）。

## Q4：改了 `planner.yaml` 里的参数为什么不生效？

**原因**：配置只在节点构造时解析一次，之后不会重载
（`src/ros2/include/ros2/config.hpp:316`）。此外参数文件的**路径来源**是
ROS 参数 `planner_params_path`，launch 只提供默认值
（`src/ma_nav.cpp:9-11`、`launch/run.launch.py:17-21`）。

**处理**：改完文件后重启节点；若用 launch 覆盖了该参数，请确认改的是被加载的那份文件。
注意 `ros2.save_global_map_path` 等 4 个路径是绝对值
（`config/planner.yaml:8`），换机器要同步修改。

## Q5：为什么第一帧速度被限制在 ±3？

**原因**：首帧没有上一帧 MPC 指令可参考，代码用里程计速度经坐标变换得到世界系速度，
并**硬编码**夹到 ±3，避免一开始就不可行
（`src/ros2/src/ros2_node.cpp:157-169`）。

**处理**：该阈值不受 YAML 控制；若要跟随 MPC 配置，需要改代码。
后续帧用上一帧 MPC 指令作为速度状态（`src/ros2/src/ros2_node.cpp:153-156`）。

## Q6：`cmd_vel` 里的角速度为什么总是 0？

**原因**：这是设计选择：MPC 只优化 (x, y, vx, vy)，输出时角速度写 0
（`src/ros2/src/ros2_node.cpp:226`），yaw 只用于世界系到车体系的变换
（`src/ros2/src/ros2_node.cpp:144-147`）。

**处理**：需要 yaw 控制时，本模块的 `yaw` 控制器仍是未实现的骨架
（见 [controller 模块文档](../planner/controller/常见问题FAQ.md)）。

## Q7：全局地图怎么保存与加载？

**原因**：保存走服务 `save_global_map`，把当前栅格写成目录下的 PGM + YAML
（`src/ros2/src/ros2_node.cpp:104-111`、`src/ros2/src/ros2_node.cpp:715-740`）；
加载在构造时进行，且**只支持 P5 二进制 PGM**
（`src/ros2/src/ros2_node.cpp:558-566`）。
规划模式下构造期会加载地图与隧道区域，但该分支被写了两遍
（`src/ros2/src/ros2_node.cpp:122-128`）。

**处理**：确认 `mapping_model` 为 false 才会加载；保存路径取自配置的绝对路径。

## Q8：规划一直失败会怎样？

**原因**：失败时只在超过 20 秒后把导航状态置为失败并打警告，否则每帧继续重试
（`src/ros2/src/ros2_node.cpp:271-280`）；
每次触发超时都会把计时起点刷新为当前时刻
（`src/ros2/src/ros2_node.cpp:275`）。

**处理**：观察 `/ma_nav/nav_feedback` 与警告日志；
重规划本身的触发与退避在 [replan 模块](../planner/replan/常见问题FAQ.md)。

## Q9：可视化的发布频率怎么调？

**原因**：可视化回调挂在 10 ms 定时器上，但内部按**地图配置**里的
可视化频率节流（`src/ros2/src/ros2_node.cpp:441-458`）。

**处理**：改地图配置里的可视化频率参数，而不是本模块的定时器周期；
本模块的 24 个可视化话题清单见 [接口文档](./接口文档.md)。

## 参数类问题

| 问题 | 原因 | 锚点 |
|---|---|---|
| 话题名改了却不生效 | 话题名来自配置，需要重启节点 | `src/ros2/include/ros2/config.hpp:320-324` |
| 折叠/反馈话题为什么配不了 | 这三处话题名硬编码 | `src/ros2/src/ros2_node.cpp:99-101` |
| `is_minco` 改了没反应 | 该键被读入配置但没有任何使用点 | `src/ros2/include/ros2/config.hpp:331` |
| `mapping_model` 影响什么 | 决定建图模式与是否加载已有地图 | `src/ros2/src/ros2_node.cpp:121-128` |
| 参数文件路径怎么覆盖 | launch 声明了同名参数并可覆盖 | `launch/run.launch.py:17-21` |

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 日志通道 | 过滤本模块的日志通道 | `src/utils/include/utils/logger.hpp:123` |
| MPC 失败 | 警告日志带当前 4 状态 | `src/ros2/src/ros2_node.cpp:181` |
| 规划超时 | 警告日志区分「无路」与「状态非成功」 | `src/ros2/src/ros2_node.cpp:276` |
| 折叠动作 | 每次下发折叠/展开都会打日志 | `src/ros2/src/ros2_node.cpp:207` |
| 导航反馈 | 订阅反馈话题看状态码 | `src/ros2/src/ros2_node.cpp:370` |
| 路径可视化 | 原始与优化路径各一个话题 | `src/ros2/src/ros2_node.cpp:298-299` |
