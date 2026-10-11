# planner/05 · MPC 控制器（4 状态线性 MPC + OSQP）

> 本文隶属 `docs/nav/算法思路/`：讲**为什么这么做、数学上怎么定义、工程上落在哪里**。
> 行为与数值结论均已与 `docs/nav/planner/controller/` 的已验证文档核对；
> 两者冲突时以模块文档 + 代码为准。行号基线：`git_rev = 8f98daee5bc7`。
> 参考写法：`docs/nav/开源参考/RM2026哈工大(威海)哨兵导航方案.md` §2.4.3.3、
> `docs/nav/开源参考/RM2026北京理工大学追梦战队哨兵导航.md` §9。

---

## 1 方案概述

控制器位于整条链路的末端：接收 MINCO 五次样条轨迹作为参考，
以 **10 ms** 节拍求解一次带约束二次规划（QP），输出底盘速度指令。

| 项 | 内容 | 锚点 |
|---|---|---|
| 调用链 | `controller_callback`（10 ms 定时器）→ `Mpc::solve` → 组装 `Twist` → `/cmd_vel_chassis` | `src/ros2/src/ros2_node.cpp:130`、`src/ros2/src/ros2_node.cpp:179`、`src/ros2/src/ros2_node.cpp:242` |
| 状态 | $x = [x, y, v_x, v_y]^\top$（世界系位置 + 世界系速度） | `src/planner/controller/include/controller/mpc.h:11-13` |
| 控制 | $u = [a_x, a_y]^\top$（世界系加速度，等效单位质量推力） | `src/planner/controller/include/controller/mpc.h:12-14` |
| 求解器 | OsqpEigen / OSQP，开启热启动、关闭打印 | `src/planner/controller/include/controller/mpc.h:279-280` |
| 实例 | 全仓库只有一个实例，模板实参是样条轨迹接口 | `src/ros2/include/ros2/ros2_node.h:146` |
| 实现形态 | 纯头文件（`mpc.cpp` 仅一行说明） | `src/planner/controller/src/mpc.cpp:1` |

与两份参考文档的差异（【事实】，按各自源码/正文对照）：

| 维度 | 哈工大（威海）§2.4.3.3 | 北理工 §9 | 本项目 |
|---|---|---|---|
| 状态 | 位置 + 速度（4 维） | $[x,y,yaw]$（3 维，SE(2)） | 位置 + 速度（4 维），**无 yaw** |
| 控制 | 加速度 | 速度 + 角速度 | 加速度 |
| 求解器 | ACADOS + qpOASES | qpOASES | OSQP（qpOASES 仅出现在链接行、未使用，`src/planner/controller/CMakeLists.txt:23`） |
| 坐标系 | 未在正文限定 | **世界系**平移速度，底盘层再按 yaw 旋转 | 世界系，节点侧按 yaw 旋转成车体系（`src/ros2/src/ros2_node.cpp:222-226`） |

---

## 2 核心思想

1. **双积分模型足够描述全向底盘**：状态取位置与速度、控制取加速度，
   连续模型是线性的双积分链（【推断】依据 §3.2 的矩阵形式）。
2. **参考轨迹保留高阶信息**：MINCO 输出的是五次样条而不是离散点，
   因此每一步的参考状态 $[p,v]$ 与前馈加速度 $a$ 都可以解析求导得到
   （`src/planner/controller/include/controller/traj_interface.hpp:77-89`）；
   这与哈工大文档强调的「绕开只传离散点的架构」是同一条思路。
3. **把约束交给 QP，而不是事后限幅**：速度与控制量都作为**硬约束**进入 QP，
   保证解本身可行（`src/planner/controller/include/controller/mpc.h:252-272`）。
4. **滚动时域**：每帧只执行解的第一段控制量，下一帧用新的状态重新求解
   （`src/planner/controller/include/controller/mpc.h:490`）。
5. **世界系速度而非车体系速度**：这样参考轨迹不必随车头朝向旋转，
   小陀螺时依然能描述「向赛场某个固定方向移动」；车体系换算放在节点侧一层完成
   （`src/ros2/src/ros2_node.cpp:222-226`；与北理工正文的约定一致）。

---

## 3 数学建模

### 3.1 记号约定

| 记号 | 含义 |
|---|---|
| $N$ | 预测步数（当前 20） |
| $\Delta t$ | 离散步长（当前 0.05 s，预测 1 s） |
| $x_k \in \mathbb{R}^4$ | 第 $k$ 步状态，$k = 0 \dots N$ |
| $u_k \in \mathbb{R}^2$ | 第 $k$ 段控制，$k = 0 \dots N-1$ |
| $r_k = [p_k, v_k]^\top$ | 第 $k$ 步参考状态（样条解析求值） |
| $r_k^u = a_k$ | 第 $k$ 步参考加速度（作为前馈） |
| $Q, Q_N \in \mathbb{R}^{4\times4}$ | 过程/终端状态权重 |
| $R, R_d \in \mathbb{R}^{2\times2}$ | 控制权重、控制增量权重 |
| $[x_{\min}, x_{\max}]$、$[u_{\min}, u_{\max}]$ | 状态/控制硬边界 |

### 3.2 连续模型与离散化

连续双积分模型【推断：由 `discretize` 中 A/B 的放置位置反推】：

$$\dot{x}(t) = A_c x(t) + B_c u(t), \quad
A_c = \begin{bmatrix} 0&0&1&0\\0&0&0&1\\0&0&0&0\\0&0&0&0\end{bmatrix}, \quad
B_c = \begin{bmatrix} 0&0\\0&0\\1&0\\0&1\end{bmatrix}$$

代码采用**一阶欧拉离散化** $A = I + A_c \Delta t$、$B = B_c \Delta t$：

$$A = \begin{bmatrix} 1&0&\Delta t&0\\0&1&0&\Delta t\\0&0&1&0\\0&0&0&1\end{bmatrix}, \qquad
B = \begin{bmatrix} 0&0\\0&0\\ \Delta t&0\\0&\Delta t\end{bmatrix}$$

【事实】$A$ 与 $B$ 的构造分别是「单位阵 + (0,2) 块乘 $\Delta t$」与「(2,0) 块乘 $\Delta t$」
（`src/planner/controller/include/controller/mpc.h:126-132`）。

【推断】这意味着 $x_{k+1} = x_k + v_k \Delta t$、$v_{k+1} = v_k + u_k \Delta t$：
加速度在**同一拍内不改变位置**（少了一项 $\tfrac{1}{2}\Delta t^2 u_k$）。
参考文档用的是二阶项版本 $B = B_c\Delta t + \tfrac12 A_cB_c\Delta t^2$（哈工大 §2.4.3.3(2)），
因此我们的位置预测存在一拍滞后；在 $\Delta t = 0.05$ s 下该项为 $1.25\times10^{-3}$ m/单位加速度。

### 3.3 预测方程

$$x_{k+1} = A x_k + B u_k, \quad x_k = A^k x_0 + \sum_{i=0}^{k-1} A^{k-1-i} B u_i$$

代码不显式构造 $\Phi,\Theta$，而是把「动力学等式」直接写进约束矩阵（§3.4），
等价于要求解满足上面的递推。

### 3.4 QP 标准型与维度推导

决策变量按「先状态、后控制」堆叠：

$$z = \begin{bmatrix} x_0^\top & x_1^\top & \cdots & x_N^\top & u_0^\top & \cdots & u_{N-1}^\top \end{bmatrix}^\top$$

维度（$N = 20$）：

$$n_z = 4(N+1) + 2N = 4\times 21 + 40 = \mathbf{124}$$

约束行数：

$$n_c = \underbrace{4}_{\text{初始状态}} + \underbrace{4N}_{\text{动力学}} + \underbrace{2N}_{\text{速度不等式}} + \underbrace{2N}_{\text{控制不等式}} = 4 + 80 + 40 + 40 = \mathbf{164}$$

【事实】上式与代码逐项对应：变量数在 `src/planner/controller/include/controller/mpc.h:156`，
约束数在 `src/planner/controller/include/controller/mpc.h:159-162`，
注释也写明了同一推导（`src/planner/controller/include/controller/mpc.h:138-146`）。

OSQP 求解的标准型：

$$\min_z \; \tfrac12 z^\top H z + g^\top z \quad \text{s.t.} \quad l \le A_{\text{lin}} z \le u$$

其中 $H$ 与 $A_{\text{lin}}$ 是**常数**（构造期一次性装配，
`src/planner/controller/include/controller/mpc.h:176-274`），
每帧只更新 $g$ 与 $[l,u]$（`src/planner/controller/include/controller/mpc.h:321-380`）。

### 3.5 代价函数：H 与 g 的口径不一致（H≠2Q）

$H$ **只取三个权重矩阵的对角元素**：

$$H = \operatorname{blkdiag}\big(\underbrace{2Q,\dots,2Q}_{k<N},\; 2Q_N,\; \underbrace{2R,\dots}_{\text{对角}},\; R_d\text{ 的相邻块}\big)$$

$$H_{x_k} = 2Q(i,i), \quad H_{u_k} = 2R(i,i), \quad
H_{u_{k-1},u_k} = -2R_d(i,i) \;(k \ge 1)$$

【事实】状态项与 $R_d$ 项的装配见
`src/planner/controller/include/controller/mpc.h:180-208`：
状态只写 $(i,i)$、控制只写 $R(i,i)$，$R_d$ 只写 $(i,i)$ 与相邻控制的 $(u_k,u_k)$/$(u_{k-1},u_{k-1})$/
$(u_k,u_{k-1})$ 四个位置，且**从 $k=1$ 开始**（`:198`），因此 $u_0$ 没有任何增量惩罚。

线性项 $g$ 却乘的是**整矩阵**：

$$g_{x_k} = -2 Q_k r_k, \qquad g_{u_k} = -2 R\, r_k^u$$

【事实】见 `src/planner/controller/include/controller/mpc.h:327` 与
`src/planner/controller/include/controller/mpc.h:333`。

两者联立后的**平稳点**（无约束时）：

$$2Q_{\text{diag}} x_k - 2Q r_k = 0 \;\Longrightarrow\; x_k = Q_{\text{diag}}^{-1} Q\, r_k$$

【事实】因此当权重矩阵有非对角元素时，QP 并不等价于「以 $Q$ 为权重的跟踪误差二次型」
（常写作 $H \ne 2Q$）：跟踪目标被 $Q_{\text{diag}}^{-1}Q$ 扭曲。
当前 YAML 只给对角列表（`config/planner.yaml:114-123`），此时 $Q_{\text{diag}}^{-1}Q = I$，
所以**现状不暴露该问题**。

$R_d$ 的情况更弱一层：【事实】$g$ 中**完全没有 $R_d$**（`:321-334` 只有 $Q_k$ 与 $R$），
$H$ 中又只用了 $R_d(i,i)$，因此 **$R_d$ 的非对角元素完全无效**；
$R_d$ 现在的实际语义是「惩罚**同一次求解内**相邻控制量之差」，
而不是「惩罚相对上一帧已下发指令的增量」——后者需要把上一帧控制量放进 $g$，
代码里没有（见 §4.6 的 `u_prev`）。

### 3.6 约束

| 约束 | 形式 | 行数 | 锚点 |
|---|---|---|---|
| 初始状态 | $x_0 = x_{\text{meas}}$（等号，$l=u=x_0$） | 4 | `src/planner/controller/include/controller/mpc.h:344-349` |
| 动力学 | $x_{k+1} - A x_k - B u_k = 0$ | $4N$ | `src/planner/controller/include/controller/mpc.h:351-358` |
| 速度硬边界 | 仅 $v_x, v_y$：$x_{\min}(2) \le v_x \le x_{\max}(2)$、$x_{\min}(3) \le v_y \le x_{\max}(3)$，$k = 1\dots N$ | $2N$ | `src/planner/controller/include/controller/mpc.h:360-371` |
| 控制硬边界 | $u_{\min}(i) \le u_i \le u_{\max}(i)$ | $2N$ | `src/planner/controller/include/controller/mpc.h:373-380` |

【事实】**位置 $x, y$ 没有硬约束**：状态不等式只用了索引 2、3，
`x_min(0/1)`、`x_max(0/1)` 在 QP 里从未被使用（`:360-371`）。

---

## 4 工程实现

### 4.1 构造期：离散化 + 求解器初始化

| 步骤 | 行为 | 锚点 |
|---|---|---|
| 构造函数 | `discretize()` → `init_solver()`，两步都只做一次 | `src/planner/controller/include/controller/mpc.h:91-97` |
| 离散化 | 装配 $A, B$ | `src/planner/controller/include/controller/mpc.h:120-133` |
| 分配维度 | 由 $N$ 算出 124 / 164 并 resize 全部 QP 数据结构 | `src/planner/controller/include/controller/mpc.h:156-172` |
| 装配 $H$ | 对角状态/控制块 + $R_d$ 相邻块 | `src/planner/controller/include/controller/mpc.h:176-210` |
| 装配 $A_{\text{lin}}$ | 四类约束按行号顺序写入三元组 | `src/planner/controller/include/controller/mpc.h:215-274` |
| OSQP 初始化 | 关闭打印、开启热启动、绑定常数矩阵与空边界 | `src/planner/controller/include/controller/mpc.h:279-291` |

【事实】由于 $H$、$A_{\text{lin}}$ 只在构造期设置，**运行期改 $Q/R/R_d$ 不会生效**；
`Mpc` 也只暴露 `set_trajectory`（`src/planner/controller/include/controller/mpc.h:48`），
没有参数更新接口。

### 4.2 每帧流程（`solve` 五参版本）

| # | 步骤 | 行为 | 锚点 |
|---|---|---|---|
| 1 | 输入检查 | $x_0$ 与 $t_{\text{now}}$ 必须有限 | `src/planner/controller/include/controller/mpc.h:408-411` |
| 2 | 构建 QP 数据 | `build_problem`：采样参考 → 写 $g$ → 写 $[l,u]$ | `src/planner/controller/include/controller/mpc.h:414-417` |
| 3 | 数据自检 | $g,l,u$ 全有限、维度匹配、$l \le u$ | `src/planner/controller/include/controller/mpc.h:420-447` |
| 4 | 更新求解器 | `updateGradient` + `updateBounds`（$H$、$A_{\text{lin}}$ 不动） | `src/planner/controller/include/controller/mpc.h:450-453` |
| 5 | 求解 | `solveProblem()`，返回码非 `NoError` 即失败 | `src/planner/controller/include/controller/mpc.h:456-460` |
| 6 | 状态复核 | 只有 `Solved` / `SolvedInaccurate` 视为成功 | `src/planner/controller/include/controller/mpc.h:462-467` |
| 7 | 取解 | 校验尺寸与有限性，取 $u_0$、$x_{0..N}$、$u_{0..N-1}$ | `src/planner/controller/include/controller/mpc.h:470-502` |
| 8 | 输出复检 | $u_0$、全部预测状态/控制再次查有限性 | `src/planner/controller/include/controller/mpc.h:505-522` |
| 9 | 记录 | `param_.u_prev = u_cmd`（**只写不读**） | `src/planner/controller/include/controller/mpc.h:525` |

【事实】共有 **10 类（13 处）**失败分支：输入非有限（`:408`）、`build_problem` 失败（`:414`）、
QP 数据非有限（`:420`）、维度不符（`:431`）、$l>u$（`:444`）、求解器数据更新失败（`:450`）、
`solveProblem` 返回码异常（`:457`）、求解状态非 Solved/Inaccurate（`:464`）、解尺寸或有限性异常（`:472`、`:477`）、
输出复检失败（`:505`、`:510`、`:517`）；全部 `return false` 并打 `controller` 通道日志。

【事实】三参重载只是转发到五参版本（`src/planner/controller/include/controller/mpc.h:389-394`）；
全仓库唯一外部调用走五参版本（`src/ros2/src/ros2_node.cpp:179`），
因此三参重载属**未使用的公开 API**。

### 4.3 参考轨迹接口

| 接口 | 语义 | 锚点 |
|---|---|---|
| `ReferencePoint` | $\{t,\; [p_x,p_y,v_x,v_y],\; [a_x,a_y]\}$ | `src/planner/controller/include/controller/traj_interface.hpp:21-25` |
| `sampleImpl` | 把 $t$ 钳到 $[0, T]$，再加样条起始时间求值，取 $p,v,a$ 的前两维 | `src/planner/controller/include/controller/traj_interface.hpp:77-89` |
| `sampleSequenceImpl` | 从 $t_{\text{start}}$ 起按 $\Delta t$ 采 $N+1$ 个参考点 | `src/planner/controller/include/controller/traj_interface.hpp:117-128` |
| `updateTrackTimeImpl` | 在 $[\text{hint}-0.3,\ \text{hint}+3.0]$ 内以 0.02 s 步长搜索离机器人最近的轨迹时间 | `src/planner/controller/include/controller/traj_interface.hpp:103-115` |
| `nearestTimeImpl` | 全程 0.1 s 粗扫，用于新轨迹的游标定位 | `src/planner/controller/include/controller/traj_interface.hpp:91-101` |

【事实】`build_problem` 要求参考点数 $\ge N+1$ 才继续
（`src/planner/controller/include/controller/mpc.h:312-316`）；
由于采样实现把超界时间钳制到端点，短轨迹会得到「末端重复」的参考点。

### 4.4 状态量 $x_0$ 的来源（节点侧）

$$x_0 = [x_{\text{odom}},\ y_{\text{odom}},\ v_x^{\text{state}},\ v_y^{\text{state}}]^\top$$

- 【事实】速度状态**优先取上一帧 MPC 预测序列第 1 步的速度分量**，
  以避免里程计噪声/超速导致 QP 不可行
  （`src/ros2/src/ros2_node.cpp:153-156`、`src/ros2/src/ros2_node.cpp:186-190`）。
- 【事实】首帧没有历史指令时回退到里程计速度，并按 yaw 旋到世界系，
  再**硬编码**夹到 $\pm 3$ m/s（`src/ros2/src/ros2_node.cpp:157-169`，
  常量在 `src/ros2/src/ros2_node.cpp:166`）。
- 【事实】参考时间 `t_now` 就是跟踪游标 `t_track_`
  （`src/ros2/src/ros2_node.cpp:139-142`），由 `update_track_time` 每帧更新。

### 4.5 输出：控制量被丢弃，实际用预测速度组指令

【事实】调用方声明了 `u_cmd` 并接收它，但**之后从未使用**；
真正下发的是预测状态序列第 1 步的速度分量：

$$v^{\text{world}} = [\,x_1(2),\ x_1(3)\,]^\top, \qquad
\begin{bmatrix} v_x^{\text{body}} \\ v_y^{\text{body}} \end{bmatrix}
= \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} v_x^{\text{world}} \\ v_y^{\text{world}} \end{bmatrix},
\quad \omega_z = 0$$

锚点：控制量声明与传入 `src/ros2/src/ros2_node.cpp:175-179`、
预测速度取值 `src/ros2/src/ros2_node.cpp:219-220`、
世界系→车体系与「不控制 yaw」`src/ros2/src/ros2_node.cpp:222-226`。

【事实】发布前还有三道保护：线性分量超过 $\pm 20$、低于 $-20$、含 NaN 时告警并丢弃本帧
（`src/ros2/src/ros2_node.cpp:229-240`）。

### 4.6 与折叠（云台）的耦合

控制器同时承担折叠事件的时间消费：按跟踪游标排空到期事件，
每个事件发布一次 `/ma_nav/fold`（`src/ros2/src/ros2_node.cpp:197-216`）。
事件本身由 `generate_fold_events` 在规划侧生成，细节见 [06 重规划策略](./06-重规划策略.md)。

---

## 5 关键参数与细节

| 参数 | YAML 值 | 代码默认 | 说明与锚点 |
|---|---|---|---|
| `N` | 20 | 20 | 预测 20 步（`config/planner.yaml:110`、`src/planner/controller/include/controller/mpc.h:16`） |
| `dt` | 0.05 s | 0.05 | 预测时域 1 s（`config/planner.yaml:111`） |
| `Q` | [30, 30, 1, 1] | 零阵 | 位置权重是速度的 30 倍（`config/planner.yaml:114`、`src/planner/controller/include/controller/mpc.h:20`） |
| `QN` | [40, 40, 2, 2] | 零阵 | 终端权重略大于过程权重（`config/planner.yaml:117`、`src/planner/controller/include/controller/mpc.h:21`） |
| `R` | [0.05, 0.05] | 零阵 | 控制权重（`config/planner.yaml:120`、`src/planner/controller/include/controller/mpc.h:24`） |
| `Rd` | [0.05, 0.05] | 零阵 | 控制增量权重；**只有对角有效**（`config/planner.yaml:123`、`src/planner/controller/include/controller/mpc.h:25`） |
| `x_min` / `x_max` | ±1000（位置）、±3（速度） | ±10 | 位置上下界**在 QP 中未被使用**（`config/planner.yaml:126-127`、`src/planner/controller/include/controller/mpc.h:28-29`） |
| `u_min` / `u_max` | ±3 m/s² | ±3 | 控制硬边界（`config/planner.yaml:130-131`、`src/planner/controller/include/controller/mpc.h:30-31`） |
| `u_prev` | 无对应键 | 零向量 | 声明 + 每帧写，**从不读**（`src/planner/controller/include/controller/mpc.h:34`、`:525`） |
| 变量数 / 约束数 | — | 124 / 164 | 由 $N$ 推导（`src/planner/controller/include/controller/mpc.h:156-162`） |
| 首帧速度夹紧 | 无对应键 | ±3 m/s | 硬编码，不跟 `u_max` 联动（`src/ros2/src/ros2_node.cpp:166`） |
| 异常阈值 | 无对应键 | ±20 | 硬编码发布保护（`src/ros2/src/ros2_node.cpp:229-236`） |

【待确认】任务说明中提到的 `p_prev`（上一帧状态热启动项）在代码里**不存在**：
全仓库检索 `p_prev` 只命中 traj_optimize 的样条求导局部变量，
控制器的 `MpcParam` 只有 `u_prev`
（`src/planner/controller/include/controller/mpc.h:15-35`）。
因此「状态热启动」在当前实现里没有对应字段；实际生效的热启动是 OSQP 求解器内部的
warm start（`src/planner/controller/include/controller/mpc.h:280`）。

【事实】控制器 CMake 链接了 `libqpOASES.a`（`src/planner/controller/CMakeLists.txt:23`），
但源码里没有任何 qpOASES 引用（全仓库仅此一处命中），属冗余链接。

---

## 6 已知限制与优化方向

| # | 限制 | 说明 | 优化方向 |
|---|---|---|---|
| 1 | $H$ 取对角而 $g$ 用整矩阵 | 非对角权重会扭曲跟踪目标（$x_k = Q_{\text{diag}}^{-1}Q r_k$）；当前对角配置下不可见 | 二者口径统一：要么 $g$ 也只取对角，要么 $H$ 装配整矩阵（`src/planner/controller/include/controller/mpc.h:180-208`） |
| 2 | $R_d$ 语义与预设不符 | 无线性项 ⇒ 非对角无效；惩罚的是同一解内相邻控制量之差，**不含上一帧已下发指令** | 把 $u_{\text{prev}}$ 写进 $g$ 的 $R_d$ 项，实现真正的增量惩罚 |
| 3 | `u_prev` 写而不读 | 每帧赋值后无消费者（`src/planner/controller/include/controller/mpc.h:525`） | 接入第 2 条，或删除该字段 |
| 4 | 位置无硬约束 | 位置只由参考跟踪 + 速度约束间接约束 | 若需走廊约束，可加位置不等式（会增加约束行数） |
| 5 | 离散化缺二阶项 | $B = B_c\Delta t$，位置预测滞后一拍 | 改为 $B = B_c\Delta t + \tfrac12 A_cB_c\Delta t^2$（与参考文档一致） |
| 6 | 控制量被调用方丢弃 | 控制器输出 $u$ 未参与指令，实际用预测速度 | 明确「位置-速度-加速度」三选一的接口契约，避免两套语义并存 |
| 7 | yaw 控制器未实现 | `YawController::control` 只有声明（`src/planner/controller/include/controller/yaw_plan.hpp:20`），零引用；`Twist.angular.z` 恒为 0 | 实现 SPIN/PLAN 模式后再接入（`src/planner/controller/include/controller/yaw_plan.hpp:15-19`） |
| 8 | 参数构造后不可改 | $H$ 与 $A_{\text{lin}}$ 只在构造期装配 | 增加参数更新接口或在改参后重建求解器 |
| 9 | 硬编码保护值 | 首帧 ±3、发布 ±20 都不受 YAML 控制 | 收入 `MpcParam` 或统一走 `u_max/x_max` |
| 10 | 缺少工程补偿 | 未见延迟补偿、杆臂补偿、速度死区（对照北理工 §9 的清单） | 若实车出现跟踪滞后，按北理工清单逐项排查 |

【推断】第 1、2 条在**当前对角配置下不影响数值结果**，但它们会让后来者
（或未来的非对角整定）得到与直觉不符的行为，因此仍应视为技术债。
