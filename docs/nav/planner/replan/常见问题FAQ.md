---
module: replan
doc: 常见问题FAQ
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T05:00:00Z
status: draft
reviewer: 未审核
---

# replan 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：目标变了却一直没有重规划，是怎么判定的？

**原因**：只有三类条件之一成立才会重规划：目标相对**上次成功规划的目标**
在任一轴上超过阈值、上一轮轨迹被判碰撞（或为空）、
机器人相对参考折线的横向偏差超阈值
（`src/planner/replan/src/fsm_replanner.cpp:35-56`）。

**处理**：先确认三件事：目标变化是否超过阈值
（`config/planner.yaml:94`）、旧轨迹是否仍然安全、
横向偏差是否小于阈值（`config/planner.yaml:96`）。
注意到达半径内的调用会直接返回成功、根本不进入判定
（`src/planner/replan/src/fsm_replanner.cpp:21-25`）。

## Q2：为什么规划失败之后每一帧都在重试？

**原因**：失败分支会把重规划标志**重新置真**并返回错误
（`src/planner/replan/src/fsm_replanner.cpp:76-79`），
而代码里没有任何退避、重试上限或最小间隔判断
（YAML 里那个 min_replan_interval 键既没有装载、也没有字段与读取点）。

**处理**：如果要限流，需要自己加退避或把那个未被读取的 YAML 键接上；
调用方只在连续失败 20 s 后才把导航状态置为失败
（`src/ros2/src/ros2_node.cpp:271-279`）。

## Q3：优化失败时会回退到原始轨迹吗？

**原因**：不会。代码在优化失败时直接返回错误
（`src/planner/replan/src/fsm_replanner.cpp:101-104`），
调用方收到错误后丢弃结果且不发布轨迹
（`src/ros2/src/ros2_node.cpp:273-279`）。
第 83 行的注释承诺了「保持原始轨迹」的兜底，但实现里没有对应分支。

**处理**：若要兜底，需要在优化失败分支里继续使用搜索得到的轨迹，
并同步检查它的安全性；【待确认】这是否是作者的意图。

## Q4：改了 `minco_traj_validity_duration` 等三个参数为什么没有效果？

**原因**：这三个参数（有效期、连续性阈值、投影搜索分辨率）被装载进结构体，
但源码中**没有任何读取点**；它们属于「用上一条轨迹做热启动」的能力，
而该能力尚未接线（`src/planner/replan/include/replan/fsm_replanner.h:26-33`）。

**处理**：不要以调参方式期待效果；要么补实现，要么从 YAML 中移除。

## Q5：起点或终点落在障碍物里怎么办？

**原因**：重规划前会先把两点沿 ESDF 梯度外推到安全距离之外
（`src/planner/replan/src/fsm_replanner.cpp:65-67`），
外推有上限：最多 200 次迭代、累计不超过 1.5 m
（`src/planner/replan/src/fsm_replanner.cpp:249-251`）。
地图外的点不做外推。

**处理**：如果外推达到上限仍未安全，点会保持在不安全位置并交给搜索器，
此时搜索大概率返回空 → 触发「规划失败」
（`src/planner/replan/src/fsm_replanner.cpp:76-80`）。

## Q6：为什么碰撞复检用的阈值比规划阈值松这么多？

**原因**：复检用的硬净空由「安全距离 × 0.2」得到，并夹到 0.05 m 下限
（`src/planner/replan/src/fsm_replanner.cpp:33`）；
而搜索与后处理用的是完整的安全距离
（`src/planner/replan/src/fsm_replanner.cpp:32`）。

**处理**：头文件里对「太大/太小」的取舍有一段注释（原本是独立参数，
现已被注释掉），【推断】它的定位是「安全网」而不是精确阈值；
如果希望它与规划阈值一致，需要改这一行的系数。

## Q7：`one_plan` 什么时候会被调用？

**原因**：全仓库没有任何调用点（只有声明与定义）
（`src/planner/replan/src/fsm_replanner.cpp:114`）。
它比 `plan` 少了安全点外推、也不设置「新轨迹」标志与成功状态；
内部还有一处日志时序问题：在把搜索结果赋给成员**之前**就打印点数
（`src/planner/replan/src/fsm_replanner.cpp:134-148`）。

**处理**：视为未接入的历史实现；启用前先补外推、状态与日志顺序。

## Q8：用带参构造创建重规划器后，优化器参数为什么没生效？

**原因**：两个构造入口行为不同：`set_param` 会同时分发优化配置与搜索后处理参数
（`src/planner/replan/include/replan/fsm_replanner.h:42-46`），
而带参构造只分发了搜索后处理参数
（`src/planner/replan/include/replan/fsm_replanner.h:49-52`）。

**处理**：改用默认构造 + `set_param`，或在带参构造里补上优化配置分发。

## Q9：横向偏差是怎么算的？

**原因**：把当前点投影到参考折线的每一段（投影参数夹紧到线段内），
取所有段的最短距离（`src/planner/replan/src/fsm_replanner.cpp:158-179`）；
若折线点数少于 2，直接返回最大浮点值，等价于「必然触发重规划」
（`src/planner/replan/src/fsm_replanner.cpp:159-161`）。

**处理**：参考折线优先取上次优化路径，缺省时取结果里的优化路径
（`src/planner/replan/src/fsm_replanner.cpp:51`）。

## 参数类问题

| 问题 | 原因 | 锚点 |
|---|---|---|
| 删掉触发阈值键会怎样 | 字段没有初始化器，装载函数存在才赋值 | `src/planner/replan/include/replan/fsm_replanner.h:23-33` |
| 硬净空能不能配 | 头文件里的对应字段已被注释，实际由安全距离推算 | `src/planner/replan/include/replan/fsm_replanner.h:29-30` |
| 安全距离从哪里来 | 复用 path_planning 参数组 | `src/planner/replan/src/fsm_replanner.cpp:32` |
| 复检采样步长怎么定 | 取后处理采样分辨率与 0.02 m 的较大者 | `src/planner/replan/src/fsm_replanner.cpp:233` |
| 搜索超时能否配 | 写死 5000 ms | `src/planner/replan/src/fsm_replanner.cpp:74` |

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 日志通道 | 过滤重规划器的日志通道 | `src/utils/include/utils/logger.hpp:120` |
| 耗时 | 每次重规划都会打印计时（默认开启） | `src/planner/replan/src/fsm_replanner.cpp:59` |
| 搜索失败 | 警告日志「planning failed」 | `src/planner/replan/src/fsm_replanner.cpp:77` |
| 优化失败 | 信息日志「ma opt failed」 | `src/planner/replan/src/fsm_replanner.cpp:102` |
| 连续失败 | 调用方 20 s 超时警告 | `src/ros2/src/ros2_node.cpp:276` |
| 新轨迹 | 结果结构里的新轨迹标志 | `src/planner/replan/include/replan/fsm_replanner.h:71` |
