---
module: controller
doc: 常见问题FAQ
git_rev: 2eed7d3e884d
evidence_digest: 7c453c4e8af72579
worktree_dirty: true
generated_by: dsh + docs/tools（clangd documentSymbol / python extractors）
generated_at: 2026-10-02T04:00:00Z
status: draft
reviewer: 未审核
---

# controller 模块 常见问题FAQ

> 只收录能在代码里找到依据的问题；属于经验判断的标【推断】。

## Q1：给权重矩阵填了非对角元素，会有什么后果？

**原因**：这类矩阵在两处的用法**不一致**：Hessian 只取对角线元素
（`src/planner/controller/include/controller/mpc.h:185`），
而线性项用整矩阵乘参考状态与控制
（`src/planner/controller/include/controller/mpc.h:327`、`src/planner/controller/include/controller/mpc.h:333`）。
因此非对角项**不是被忽略**，而是只进入线性项，使 QP 不再等价于
以该矩阵为权重的二次型（H≠2Q），最优点会偏离预期。

**处理**：当前 YAML 只提供对角列表，所以现状没有问题；
若要支持完整二次型，需要让 Hessian 也用整矩阵（并同步改增量项），
或显式拒绝非对角输入。【待确认】哪种做法是作者意图。

## Q2：控制增量权重是干什么的？为什么跨帧还是有跳变？

**原因**：控制增量项惩罚的是**同一次求解内**相邻控制之差
（`src/planner/controller/include/controller/mpc.h:197-207`）。
参数里确实有一个「上一帧控制量」字段并在求解末尾写入
（`src/planner/controller/include/controller/mpc.h:525`），
但全仓库没有任何读取点，因此它**不参与**跨帧惩罚。

**处理**：若要跨帧平滑，需要在构建 Hessian 或线性项时把该字段用起来；
当前它只是占位。

## Q3：`yaw_plan.hpp` 里的 yaw 控制器怎么用？

**原因**：它只有骨架：枚举与成员函数声明
（`src/planner/controller/include/controller/yaw_plan.hpp:15-20`），
成员函数没有定义，全仓库也没有任何调用点。

**处理**：要用得先实现 `control` 并接入下发链路；
当前云台 yaw 不由本模块闭环控制，折叠事件也在 ros2 侧生成
（`src/ros2/src/ros2_node.cpp:315-321`）。

## Q4：QP 的规模有多大？

**原因**：决策变量 = 状态 4×(N+1) + 控制 2×N，约束 = 4 + N×4 + N×2 + N×2
（`src/planner/controller/include/controller/mpc.h:156-162`）。
按当前 N=20 计算，即 124 个变量、164 个约束。

**处理**：调大 N 会同时放大变量与约束数；N 与 dt 都在参数里
（`config/planner.yaml:110-111`）。

## Q5：日志出现 `[MPC] ...` 失败怎么排查？

**原因**：失败点按顺序是：输入非法 → 构建问题失败（无轨迹/参考点不足）→
QP 数据非法（非有限、维度不符、上下界矛盾）→ OSQP 报错或未求解成功 →
解非法（尺寸、NaN）
（`src/planner/controller/include/controller/mpc.h:408-480`）。

**处理**：按上面顺序读日志；调用方还会补一条带当前状态的警告
（`src/ros2/src/ros2_node.cpp:181`），
如果这条警告里的速度已经贴限幅，优先怀疑参考轨迹或权重整定。

## Q6：为什么必须先把轨迹设进控制器？

**原因**：构建问题的第一步就是检查轨迹指针与接口有效性
（`src/planner/controller/include/controller/mpc.h:299-301`），
并把参考点采样作为 QP 线性项的唯一来源
（`src/planner/controller/include/controller/mpc.h:311-334`）。

**处理**：每次规划出新轨迹后都要调用设置接口
（`src/ros2/src/ros2_node.cpp:313`）。

## Q7：`src/mpc.cpp` 为什么是空的？

**原因**：实现已经全部改成头文件模板
（`src/planner/controller/src/mpc.cpp:1`）。

**处理**：不要再往 .cpp 加实现；模板需要在使用方（ros2）实例化
（`src/ros2/include/ros2/ros2_node.h:146`）。

## Q8：为什么 `x_min` / `x_max` 里给 x、y 设限没有用？

**原因**：状态不等式只对 vx、vy 两个分量建立，且从第 1 步开始
（`src/planner/controller/include/controller/mpc.h:363-369`）；
x、y 分量从未进入约束。

**处理**：这与 YAML 注释一致（x/y 不设硬约束，给很大值）
（`config/planner.yaml:125`），
若确实需要位置约束，要改约束构建代码。

## Q9：底盘的速度指令是怎么从 MPC 解里得到的？

**原因**：调用方取预测状态序列的第 1 步（而不是直接取解里的状态 0）
作为速度指令
（`src/ros2/src/ros2_node.cpp:186-189`），
并把它记成「上一帧 MPC 指令」。

**处理**：调试时对比预测状态与真实状态，可判断模型误差。

## 参数类问题

| 问题 | 原因 | 锚点 |
|---|---|---|
| 删掉权重键会怎样 | 权重默认是零矩阵，追踪代价退化为 0 | `src/planner/controller/include/controller/mpc.h:20-25` |
| N 和 dt 改了要注意什么 | 两者共同决定预测时域，且变量/约束数随 N 线性增长 | `src/planner/controller/include/controller/mpc.h:156-162` |
| 控制限幅在哪里生效 | 作为控制不等式边界进入 QP | `src/planner/controller/include/controller/mpc.h:374-380` |
| 速度限幅在哪里生效 | 作为状态不等式边界进入 QP（只对 vx/vy） | `src/planner/controller/include/controller/mpc.h:361-371` |
| 参数结构体在哪里装载 | ros2 侧装载后构造注入，模块自身不读 YAML | `src/ros2/include/ros2/config.hpp:272-289` |

## 排查手段

| 手段 | 用法 | 锚点 |
|---|---|---|
| 日志通道 | 过滤控制器的日志通道 | `src/utils/include/utils/logger.hpp:124` |
| 求解状态 | 失败日志里带 OSQP 状态码 | `src/planner/controller/include/controller/mpc.h:458` |
| 数据自检 | 非有限/维度/上下界三类问题分别有独立日志 | `src/planner/controller/include/controller/mpc.h:421` |
| 调用方降级 | 求解失败时本帧不下发，并打印当前状态 | `src/ros2/src/ros2_node.cpp:181` |
| 预测序列 | 求解成功的重载会回填预测状态与控制序列，可用于离线复现 | `src/planner/controller/include/controller/mpc.h:52-58` |
