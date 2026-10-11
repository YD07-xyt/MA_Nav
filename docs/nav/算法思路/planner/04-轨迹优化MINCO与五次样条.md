# planner/04 · 轨迹优化：MINCO 思路与五次样条实现

> 本文隶属 `docs/nav/算法思路/`：讲**为什么这么做、数学上怎么定义、工程上落在哪里**。
> 行为与数值结论均已与 `docs/nav/planner/traj_optimize/` 的已验证文档核对；
> 两者冲突时以模块文档 + 代码为准。行号基线：`git_rev = 8f98daee5bc7`。
> 参考写法：`docs/nav/开源参考/RM2026哈工大(威海)哨兵导航方案.md` §2.4.3.2 的 (2)(3)(4)(5) 小节。

## 1 方案概述

轨迹优化位于搜索与控制的中间：**接收上游"带时间戳的参考折线"，输出 C² 连续的五次样条**
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:79-88`）。
它是全链路里唯一把"几何可通行"变成"动力学可执行"的环节
（作者需求原文：`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/problem.md:23-25`）。

| 项 | 内容 | 锚点 |
|---|---|---|
| 调用链位置 | `jps → post_processing → traj_opt → mpc` | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/problem.md:5` |
| 入口 | `MaSplineTrajectoryOptimizer::optimize`，按 `OptModel` 分派 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:165-193` |
| 输入 | `MaSplineInput`：带时间的参考点 + 边界速度/加速度 + 模型枚举 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:66-78` |
| 输出 | `MAsplineOutput`：五次样条 + 每段时长 + 起始时间 + 末次代价 + 成功标志；模板实参为 `<3>`，**2D 模式下第三维 yaw 被置零** | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:79-88`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:120-142` |
| 决策变量 | 每段时长 $T_i$ + 中间路点坐标（首尾路点固定） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:225-231` |
| 求解器 | 两阶段 L-BFGS（第三方单头 `lbfgs`） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:235-244`、`src/utils/include/utils/lbfgs.hpp:438` |
| 生效实现 | `ma_spline_opt/**`；`spline_opt/**` 为后续版本、`gcopter/**` 为 MINCO 原实现（均未使用） | `src/planner/traj_optimize/include/traj_optimize/opt.md:1-5` |

与参考方案的差异：

| 维度 | 哈工大（威海）§2.4.3.2(4)(5) | 本项目 |
|---|---|---|
| 轨迹表示 | MINCO **S3NU**：五次多项式样条、最小化 jerk 能量 | 同为五次样条 + jerk 能量，但是**自研样条类**（`QuinticSplineND`），非 gcopter 实现 |
| 避障 | ESDF 软约束 + 正交梯度投影 + 二次插值查询 | 同为软约束 + 正交投影，距离场是 **2D 栅格 ESDF 缓冲 + 双线性插值**（§4.5） |
| 二度优化 | 二次轨迹优化（参考 DDR-opt） | **同一问题的两次 L-BFGS**（权重/迭代预算不同），不做拓扑重选 |
| 折叠/隧道 | 文中未涉及 | 折叠时机由上游后处理 + ros2 折叠事件负责，优化器只管轨迹（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/problem.md:25`） |

## 2 核心思想（微分平坦 / MINCO 表示）

1. **几何可通行 ≠ 动力学可通行**：上游输出是离散折线，没有速度/加速度，控制器无法直接跟踪
   （需求原文：`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/problem.md:23`）。
2. **微分平坦把问题降到位置空间**：全向底盘取平坦输出 $z=[x,y]^\top$，控制量 $u=[v_x,v_y]^\top$
   可由 $z$ 的一阶导数解析给出。【推断】依据：代码只对位置建样条，速度/加速度全部由解析求导得到
   （§3.2 的基函数），且掩码只放开路点与时长的优化（§3.3）。因此在优化器里无需写微分约束。
3. **五次样条 = 最小化 jerk 能量**：五次片段的三阶导数是分段一次函数，最小化 $\int\|\text{jerk}\|^2$
   恰好使位置/速度/加速度连续（C²）；这与参考文档「S3NU 对应五次多项式样条、最小化加加速度」一致
   （`docs/nav/开源参考/RM2026哈工大(威海)哨兵导航方案.md` §2.4.3.2(4)），
   本项目样条阶数与系数个数吻合（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1311-1313`）。
4. **只把"路点 + 时间"当决策变量**（不取全部多项式系数）：既大幅降维，又天然保证轨迹**精确过点**。
5. **本项目与 gcopter 的关系**：作者旁注明确 `ma_spline_opt` 是主代码、`gcopter` 是 MINCO 原实现且未使用
   （`src/planner/traj_optimize/include/traj_optimize/opt.md:1-5`）——即借 MINCO 的表示与目标，重写了自己的样条/优化框架。

## 3 数学建模

### 3.1 记号约定

| 记号 | 含义 | 代码对应 |
|---|---|---|
| $D$ | 空间维度（2D 模式 $D=2$；联合模式 $D=3$，由模板实参决定） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:22-23` |
| $N$ | 段数 = 参考点数 − 1 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:223` |
| $T_i$ | 第 $i$ 段时长（决策变量） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:226` |
| $q_j\in\mathbb{R}^D$ | 第 $j$ 个路点坐标（中间点为决策变量） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:227-229` |
| $c_{i,k}$ | 第 $i$ 段第 $k$ 个系数（$k=0..5$） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1351` |
| $K$ | 每段积分采样**区间数**（`integral_num_steps`） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:2474` |
| $d(p)$ | 位置 $p$ 处的距离场取值 | `src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp:31-33` |

### 3.2 轨迹表示

第 $i$ 段是六系数五次多项式（局部时间 $t\in[0,T_i]$）：

$$p_i(t)=\sum_{k=0}^{5}c_{i,k}\,t^{k},\qquad
p_i'''(t)=6c_{i,3}+24c_{i,4}t+60c_{i,5}t^{2}$$

代码用基函数行向量直接取各阶导数（位置/速度/加速度/jerk/snap/crackle 六个基：
`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1478-1494`）。片段间由样条类保证
位置/速度/加速度连续，边界条件取自输入（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:217-221`）。

【待确认】输入字段命名误导：`start_state` / `final_state` 名字像位姿，**实际存的是边界速度**
（赋值处 `src/planner/path_planning/src/post_processing.cpp:337-338`；
类型是 3 维速度向量 `src/planner/path_planning/include/path_planning/post_processing.h:93-94`）。
`from_path_planning_trajectory` 取其前两维作线速度、第三维作 yaw 角速度
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:142-148`），语义自洽但命名容易误用，建议改注释。

### 3.3 决策变量与维度

本项目使用的掩码只放开**每段时长**与**中间路点**（首尾固定）：
$\text{mask.time}=[1]_N$、$\text{mask.waypoints}=[0,1,\dots,1,0]$
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:225-229`）。时间与空间都用**恒等参数化**
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:263-266`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:316-318`），故

$$x=\big[T_1,\dots,T_N,\;q_2,\dots,q_{N}\big]^{\top},\qquad \dim(x)=N+(N-1)D$$

维度由布局对象给出（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:1950`）。
【事实】时长有 1 ms 量级的下限校验：`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:1040`、`:2262-2265`。

### 3.4 目标函数

$$J=\rho_{energy}E+J_{time}+\sum_{i=1}^{N}\sum_{k=0}^{K} w_k\,\Delta t_i\,L\big(p_i(t_{ik}),v_i(t_{ik}),a_i(t_{ik})\big)$$

$\rho_{energy}$ 由阶段配置给出（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:270`）。

**平滑项（jerk 能量，闭式）**：

$$E=\sum_{i=1}^{N}\int_{0}^{T_i}\big\|p_i'''(t)\big\|^{2}\mathrm{d}t$$

代入 §3.2 的 $p_i'''$ 展开（【推断】：标准展开，结果与代码闭式逐项一致）：

$$E_i=36\|c_{i,3}\|^{2}T_i+144\,c_{i,3}\!\cdot\!c_{i,4}T_i^{2}+192\|c_{i,4}\|^{2}T_i^{3}+240\,c_{i,3}\!\cdot\!c_{i,5}T_i^{3}+720\,c_{i,4}\!\cdot\!c_{i,5}T_i^{4}+720\|c_{i,5}\|^{2}T_i^{5}$$

【事实】该闭式与代码一致，且**不出现 $c_{i,2}$**（jerk 与 $c_2$ 无关）：
`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1519-1524`；能量梯度同样解析给出
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:1536-1571`），以 $\rho_{energy}$ 加权汇总。

**时间项**：

$$J_{time}=\sum_{i=1}^{N} w_T T_i+\sum_{i:T_i<T_{min}} w_{min}\big(T_i-T_{min}\big)^{2}+\text{band}$$

band 是"段时长相对平均时长"的软带：记 $\bar T=\frac1N\sum_j T_j$，
当 $T_i<0.9\bar T$ 或 $T_i>1.1\bar T$ 时罚 $w_{mean}(T_i-0.9\bar T)^2$ / $w_{mean}(T_i-1.1\bar T)^2$，
对 $T_j$ 的梯度含 $\delta_{ij}-c/N$ 项（$c=0.9$ 或 $1.1$）
（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:173-222`）。作用：压总时间、避免极短段（会让曲率/加速度爆掉）、
并用 band 让各段时长**均匀**，防止时间都堆到某一段。

**采样代价（避障与动力学，smoothedL1 软约束）**，$p_e=10^{-2}$：

$$f(x)=\begin{cases}0, & x\le 0\\[2pt] \frac{x^{3}}{p_e^{2}}-\frac{x^{4}}{2p_e^{3}}, & 0<x<p_e\\[4pt] x-\frac{p_e}{2}, & x\ge p_e\end{cases}\qquad
f'(x)=\begin{cases}0, & x\le 0\\[2pt] \frac{3x^{2}}{p_e^{2}}-\frac{2x^{3}}{p_e^{3}}, & 0<x<p_e\\[4pt] 1, & x\ge p_e\end{cases}$$

【事实】与代码逐项相同（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:30-51`，`f3c=1/p_e²`、`f4c=-0.5/p_e³`）；
参考 §2.4.3.2(4) 给出同一形式（注明参考自 SUPER / DDR-opt）。选它的理由：
轻微违规时给出**接近恒定**的推力，且在 0 与 $p_e$ 处可导。各分项：

$$L_{obs}=w_{obs}f\big(d_{safe}-d(p)\big)\ (\text{仅当 }p\text{ 在距离场范围内}),\qquad
L_{vel}=w_{vel}f\big(\|v_{xy}\|^{2}-v_{max}^{2}\big),\qquad
L_{acc}=w_{acc}f\big(\|a_{xy}\|^{2}-a_{max}^{2}\big)$$

$$L_{\omega}=w_{\omega}f\big(|\omega|-\omega_{max}\big),\qquad
L_{\alpha}=w_{\alpha}f\big(|\alpha|-\alpha_{max}\big)\quad(\text{仅 }D=3)$$

（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:73-134`、`:136-149`）。注意速度/加速度用**平方量**，
故梯度含 $2v$ / $2a$ 因子（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:127`、`:133`）。

避障梯度做两步处理（对应参考文档的"正交投影 + 梯度增强"）：设 $\hat t=v/\|v\|$，
先去切向分量

$$g_\perp=\nabla d-(\nabla d\cdot\hat t)\,\hat t$$

再按模长增强：$\|g_\perp\|\in(10^{-6},0.5)$ 时
$g_\perp\leftarrow g_\perp\sqrt{\|g_\perp\|}/\|g_\perp\|$；
$\|g_\perp\|\le10^{-6}$ 且 $\|\nabla d\|>10^{-6}$ 时
$g_\perp\leftarrow 0.5\,\nabla d/\|\nabla d\|$；最终

$$\partial L_{obs}/\partial p=-w_{obs}\,f'(\cdot)\,g_\perp$$

（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:88-119`）。正交投影只让避障力改变**几何形状**、
不干扰沿轨速度分配，避免"纵向挤牙膏"式局部最优。

### 3.5 数值积分

每段等距**梯形积分**：$\Delta t_i=T_i/K$、节点 $t_{ik}=k\,T_i/K$，
权重 $w_k=0.5$（两端）或 $1$（内部），即
$\int_0^{T_i}L\,\mathrm dt\approx\sum_{k=0}^{K}w_k\Delta t_i L(t_{ik})$。
【事实】`K = integral_num_steps`，权重与步长见
`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:2474-2510`；当前 $K=8$（`config/planner.yaml:50`），
即每段 **9 个采样点**。

### 3.6 梯度与 L-BFGS

1. **采样点 → 系数**：每个采样点调一次代价函子，函子回填
   $(\partial L/\partial p,\partial L/\partial v,\partial L/\partial a)$
   （形参见 `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:53-66`），优化器用基函数把梯度累加到**系数梯度**
   （`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:2514-2530` 起）。
2. **系数 → 决策变量**：样条类给出解析梯度（能量对时长/内点/边界：
   `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp:956-975`），优化器组装成对 $x$ 的梯度
   （`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:1528-1553`、`:1426-1470`）——
   **全程解析求导，不依赖数值差分**。
3. **L-BFGS**：把 $(J,\nabla_x J)$ 交给第三方单头（`src/utils/include/utils/lbfgs.hpp:438`），
   用有限内存近似 Hessian 逆（标准 two-loop recursion、历史长度 `mem_size`；
   【推断】该单头是标准实现，本项目只配置参数）。成功判据为
   「返回码非负 + 代价有限 + $x$ 有限」（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:321`）。

## 4 工程实现

### 4.1 调用链

```
fsm_replanner.plan
  └─ GridMapESDF 适配器（包住 2D 栅格地图）
     └─ ma_opt_.set_esdf_interface(&grid_map_esdf)      // 只存裸指针
        └─ from_path_planning_trajectory(trajectory)   // 上游轨迹 → MaSplineInput
           └─ ma_intput.model = OMNI_XY                // 当前唯一被设置的模型
              └─ ma_opt_.optimize(input) → MAsplineOutput
```

| 步骤 | 锚点 |
|---|---|
| 构造距离场适配器并注入 | `src/planner/replan/src/fsm_replanner.cpp:92-93` |
| 上游轨迹转优化输入 | `src/planner/replan/src/fsm_replanner.cpp:94`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:108-153` |
| 选定 2D 模型 | `src/planner/replan/src/fsm_replanner.cpp:97` |
| 调用优化并处理失败 | `src/planner/replan/src/fsm_replanner.cpp:100-106` |

### 4.2 两阶段流程

两阶段在**同一个** `Eigen::VectorXd x` 上串行执行（按引用传递 ⇒ 阶段 2 以阶段 1 的解为初值）：

| 阶段 | 做什么 | 锚点 |
|---|---|---|
| 0 构造问题 | 参考点 → 时间点 + 路点矩阵、边界条件、掩码 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:210-231` |
| 1 粗解 | 用 `stage1` 配置解一次 L-BFGS | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:235-238` |
| 2 精修 | 用 `stage2` 配置在同一 $x$ 上再解一次 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:240-244` |
| 3 同步 | `synchronizeWorkingState` → `getWorkingSpline` | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:247-252` |
| 4 组装输出 | 2D 样条扩 3D（yaw 置零）、时长/起始时间/代价/成功标志 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:253-257` |

【事实】失败返回空输出并置 `success=false`（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:236-243`），
上层据此返回 `MINCO_OPT_FIALED`（`src/planner/replan/src/fsm_replanner.cpp:102-104`）。

### 4.3 单阶段内部

| 步骤 | 内容 | 锚点 |
|---|---|---|
| 阶段配置 | `rho_energy`、`integral_num_steps` 写入求解器配置 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:269-273` |
| 准备上下文 | `prepareContext`（校验 + 布局 + 缓存） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:274` |
| 初值 | $x$ 为空则生成初值；否则校验维度一致（**阶段 2 复用阶段 1 解的位置**） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:278-282` |
| 代价组装 | `TimeCost` + `RobotIntegralCost<2>`（避障/速度/加速度） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:284-299` |
| 回调桥接 | `makeEvaluateSpec` + `CallbackCtx`（成员函数桥给 C 风格回调） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:301-302` |
| 求解 | `lbfgs::lbfgs_optimize(x, cost, cost_callback_2d, …, params)` | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:304-320` |

### 4.4 2D → 3D 输出扩展

2D 结果用 `extend_xy_with_zero_yaw` 升维：位置列照抄、**yaw 列整体置零**、
边界角速度与角加速度也置零（`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:120-142`）。
因此 2D 模式下下游拿到的样条**不携带航向信息**，这与控制器不做 yaw 控制一致
（`docs/nav/planner/controller/模块设计说明.md`）。

### 4.5 距离场的真实来源（避免误读"ESDF 关闭"）

- 优化器的 `esdf_` 是接口抽象（`src/planner/traj_optimize/include/traj_optimize/esdf_Interface.hpp:9-40`），
  实际注入 `GridMapESDF`，它包住 **2D 栅格地图**（`src/planner/traj_optimize/include/traj_optimize/grid_map_esdf.hpp:27-45`，
  注入点 `src/planner/replan/src/fsm_replanner.cpp:92-93`）。
- 该栅格地图**自带一份 ESDF 缓冲**：占据变化时 `updateESDF()` 重建
  （`src/map/src/grid_map.cpp:25`、`:56`、`:200`），距离查询为**双线性插值**
  （`src/map/src/grid_map.cpp:60-88`），梯度由同一缓冲差分（`src/map/src/grid_map.cpp:91-110`）。
- 因此 `config/map.yaml:32` 关闭的是"**3D 概率地图的 ESDF 子地图**"，与这条链路无关；
  优化器的避障代价**确实在生效**，用的是 2D 栅格距离场。
  【事实】唯一注意点：地图外 `isInside` 为假 ⇒ **不产生避障惩罚**
  （`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:78`、`src/map/src/grid_map.cpp:61`）。

### 4.6 空转与死代码（必须区分）

| 项 | 状态 | 锚点 |
|---|---|---|
| `check_trajectory_collision` | **无任何调用点**（声明 + 定义即全部出现处），且不带 yaw 形参 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:38`、`:145-163` |
| 3D 联合分支 `optimize_xy_yaw_joint` / `optimize_3d_stage` | 逻辑完整但**运行时不经过**：全仓库只设置 `OMNI_XY` | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:325-408`、`:411-480`、`src/planner/replan/src/fsm_replanner.cpp:97` |
| yaw 权重与 yaw 上限（4 个参数） | 只在 $D=3$ 分支被读 ⇒ 当前**不生效** | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:136-149`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:364-370` |
| 热启动字段 `last_xy_spline_` / `has_last_traj_` | 声明后**从未使用**（无读无写） | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:49-50` |
| `spline_opt/**`、`gcopter/**` | 停用副本，不在任何 include 链上 | `src/planner/traj_optimize/include/traj_optimize/opt.md:1-5` |

## 5 关键参数与细节

### 5.1 顶层参数（两阶段共用）

| 参数（YAML 键路径） | 当前值 | 代码默认 | 说明 | 锚点 |
|---|---|---|---|---|
| planner_config.ma_spline_opt_params.safe_distance | 0.35 | 0.3 | 距离场安全距离（与搜索同口径） | `config/planner.yaml:47`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:42` |
| planner_config.ma_spline_opt_params.v_max | 3.0 | 3.0 | 线速度软约束上限 | `config/planner.yaml:48`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:43` |
| planner_config.ma_spline_opt_params.a_max | 5.0 | 5.0 | 线加速度软约束上限 | `config/planner.yaml:49`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:44` |
| planner_config.ma_spline_opt_params.integral_num_steps | 8 | 8 | 每段积分区间数（采样点 $K+1=9$） | `config/planner.yaml:50`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:53` |
| planner_config.ma_spline_opt_params 的 yaw 四项（weight_yaw_vel / weight_yaw_acc / yaw_rate_max / yaw_acc_max） | 40 / 40 / 1.5 / 1.5 | 10 / 5 / 1.0 / 2.0 | **当前不生效**（只被 $D=3$ 分支读取） | `config/planner.yaml:52-55`、`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:47-50` |

【事实】参数经 `load_stage_config` / `load_ma_spline_opt_config` 读入
（`src/ros2/include/ros2/config.hpp:169`、`:188`）；`safe_distance` 与四个 yaw 参数
在 YAML 与代码默认值之间**不一致**，属有意调参。

### 5.2 两阶段差异与"为什么两阶段"

| 参数 | 代码默认 | stage1（YAML） | stage2（YAML） |
|---|---|---|---|
| rho_energy | 5.0 | **10.0** | 5.0 |
| weight_time | 20.0 | **80.0** | 20.0 |
| weight_obstacle | 8000.0 | **800.0** | 8000.0 |
| weight_vel | 40.0 | **5.0** | 40.0 |
| weight_acc | 40.0 | **5.0** | 40.0 |
| max_iterations | 200 | **100** | 200 |
| weight_mean_time / mean_lower / mean_upper / min_time / weight_min_time / g_epsilon / lbfgs_mem_size / lbfgs_delta | 40 / 0.9 / 1.1 / 0.1 / 100 / 1e-4 / 64 / 5e-3 | 同默认 | 同默认 |

（YAML 锚点：`config/planner.yaml:59-72`、`config/planner.yaml:76-89`；
代码默认：`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:16-37`。
【事实】两阶段共 **6 处**不同，且 **stage2 与代码默认值逐项相同**。）

【推断】分工：stage1 用「强平滑（$\rho$ 翻倍）+ 强时间压力（4 倍）+ 弱避障（1/10）+ 弱动力学（1/8）+ 少迭代（100）」
先得到**形状好、时间紧**的粗解；stage2 把避障放大 10 倍、动力学放大 8 倍、迭代加到 200，
在粗解附近**精修**到贴障碍且满足动力学。这是参考文档 §2.4.3.2(5)「二度优化」的本项目版本，
但**不是拓扑重选**。

### 5.3 求解与积分细节

| 项 | 值 / 行为 | 锚点 |
|---|---|---|
| L-BFGS 迭代上限 | stage1 100 / stage2 200 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:305` |
| 收敛阈值 / 历史长度 / 线搜索 | $10^{-4}$ / 64 组 $\{s,y\}$ / `past=3`、`delta=5e-3`、`min_step=1e-32` | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:306-310` |
| 成功判据 | 返回码 $\ge0$、代价有限、$x$ 有限 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:321` |
| 每段采样 | $K=8$ ⇒ 9 点梯形 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:2474-2510` |
| 最短段长 | 1 ms 量级下限校验 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:1040` |

【事实】时间变量**没有**对数/平方重参数化（默认映射是恒等：
`src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:263-266`），负时长靠下限校验与 `min_time` 惩罚排除。

## 6 已知限制与优化方向

| # | 限制 | 证据 | 优化方向 |
|---|---|---|---|
| 1 | **2D 输出 yaw 恒为 0**，下游拿不到航向 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:127`、`:131-134` | 接上 `OMNI_XY_YAW_JOINT` 分支（逻辑已在，未接线：`src/planner/replan/src/fsm_replanner.cpp:97`）并重新验证 yaw 权重/上限 |
| 2 | **没有输出后的可行性兜底**：后校验函数无调用点 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:38`、`:145-163` | 把它接到 `optimize_xy` 返回前；失败时降级到上游折线 |
| 3 | 距离场是**双线性插值**，狭窄隧道处梯度精度有限 | `src/map/src/grid_map.cpp:60-88` | 参考 §2.4.3.2(3) 改用邻域二次插值，改善等距线附近梯度 |
| 4 | 图外不惩罚：`isInside` 为假时避障代价为 0 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:78`、`src/map/src/grid_map.cpp:61` | 对未知区域加惩罚，或把轨迹约束在地图范围内 |
| 5 | 速度/加速度是**软约束**，无硬边界 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/cost.hpp:122-134` | 加安全走廊式硬约束，或输出侧做投影校验 |
| 6 | 积分精度固定为每段 9 点，长段采样稀疏 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:2474-2510` | 采样点数按段长自适应（$K_i\propto T_i$） |
| 7 | 时间变量恒等参数化 + 1 ms 下限校验 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp:263-266`、`:1040` | 对数重参数化（$\tau_i=\log T_i$）可去掉校验、改善条件数 |
| 8 | 两阶段只调权重，**不做拓扑重选**；局部最优时无自救 | §5.2 | 引入对侧绕障候选（拓扑采样/走廊生成）或失败换初值重试 |
| 9 | 未使用接口/热启动字段 + 停用副本带来阅读成本；上游 `start_state` 命名误导 | `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/traj_optimizer.h:49-50`、`src/planner/path_planning/src/post_processing.cpp:337-338` | 清理或标注"预留"；重命名/补注释 |

**与模块文档的关系**：接口签名、参数键路径、调用链与失败处理以
`docs/nav/planner/traj_optimize/` 为准（该 9 篇已过校验器与两轮独立审核）；
本文只补充"数学形式与动机"。不一致处以模块文档 + 代码为准。
