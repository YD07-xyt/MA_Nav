# planner/02 全局搜索：JPS 与 A\*

> 行号基线：`git_rev = 8f98daee5bc7`；标注约定与锚点口径见 [算法思路 README](../README.md) §6。
> 本文只描述**本仓库当前实现**；提到 `docs/nav/开源参考/` 的地方仅用于对照取舍，不照抄其方案。

## 1 方案概述

前端搜索位于「重规划 FSM → 路径搜索 → 后处理 → MINCO 优化」这条链的第一环：

| 环节 | 入口 | 产物 |
|---|---|---|
| 触发 | `FsmReplan::plan`（`src/planner/replan/src/fsm_replanner.cpp:58`） | 是否需要重规划 |
| 起终点预处理 | `get_safe_pos` 沿距离场梯度外推（`src/planner/replan/src/fsm_replanner.cpp:66-67`） | 无碰撞的起终点 |
| **栅格搜索** | `PathPlanning::path_planning`（`src/planner/path_planning/src/path_planning.cpp:41-42`） | 折线航点 `raw_path` |
| 后处理 | 同函数 :48-57 | 带时间的参考点（见 [planner/03](./03-路径后处理与时间分配.md)） |

【事实】本模块**同时保留两条搜索实现**，由门面的一个布尔开关选择：
`use_jps_ ? jps_.jps_search(...) : astar_.original_astar_search(...)`
（`src/planner/path_planning/src/path_planning.cpp:41-42`），开关默认值 `false`
（`src/planner/path_planning/src/path_planning.cpp:68`）。

【事实】但**生产路径上生效的是 JPS**：重规划在每次调用前显式打开
（`src/planner/replan/src/fsm_replanner.cpp:69`，无调用点的 `one_plan` 里同样为 :124），
并传入 **5000 ms** 的超时（`src/planner/replan/src/fsm_replanner.cpp:74`），
而不是函数签名里的默认 1000 ms（`src/planner/path_planning/include/path_planning/search/jps.h:45`）。
因此 A\* 在当前系统里是**对照/回退实现**，不参与实际导航。

【事实】两条实现的对外契约一致：输入 2D 栅格 + 起终点 + 超时，返回航点折线；
返回空 vector 表示失败（`src/planner/path_planning/include/path_planning/search/jps.h:42-46`、
`src/planner/path_planning/src/search/astar.cpp:14-15`）。
门面再包一层：`raw_path` 为空即返回 `std::nullopt`
（`src/planner/path_planning/src/path_planning.cpp:44-46`）。

## 2 核心思想

### 2.1 为什么前端要用格栅搜索

后端 MINCO/五次样条是**局部**优化器（依赖初值与梯度），它需要一个拓扑正确、无碰撞的
初始折线；这个折线的几何质量与时间初值直接决定优化是否收敛。
项目在 `docs/nav/开源参考/RM2025中科大导航.md:271-272` 记录了同一取舍：
JPS 在无权重格子上最快，而 Kinodynamic A\*/RRT\* 这类融合运动学的方法
「局部性太强，难以利用全局信息」。本项目的选择与此一致：**搜索层只解决"怎么绕过去"，
运动学约束全部交给后处理**（见 [planner/03](./03-路径后处理与时间分配.md) §2）。

### 2.2 JPS 的核心：不逐格扩展，而是"跳跃"

普通 A\* 每次弹出节点就枚举 8 个邻居；JPS 观察到：**在无障碍的直线/对角走廊上，
中间格子不可能出现在任何最优路径上**。于是它沿方向"跳跃"：

- 只把三类格子作为候选（跳点）：**强制邻居**所在格、**终点**所在格、
  对角移动时**正交分量上存在跳点**的格（`src/planner/path_planning/src/search/jps.cpp:88-121`）；
- 展开一个跳点时，只扫「父节点来向的自然延续方向 + 强制邻居方向」，
  而不是全部 8 个方向（`src/planner/path_planning/src/search/jps.cpp:123-169`）。

【事实】作者在头文件里声明了收益：同样栅格图上「通常只需要展开比普通 A\* 少一到两个
数量级的节点」（`src/planner/path_planning/include/path_planning/search/jps.h:14-16`）。
【待确认】该数量级是设计预期，仓库内没有任何基准测试代码可以复现它（`src/` 下无测试）。

### 2.3 为什么 JPS 可以直接替换 A\*

两者都在同一张栅格、同一套碰撞判定、同一组代价（1 / $\sqrt2$）上求解，
JPS 的剪枝是**保最优**的（见 §3.3），因此替换只影响扩展数、不影响解的性质。
这也是门面能用一个开关切换的前提（`src/planner/path_planning/src/path_planning.cpp:26-28`）。

### 2.4 A\* 保留在哪、差在哪

A\* 用的是「哈希键 + `unique_ptr` 节点存储 + 无封闭集」的教科书写法
（`src/planner/path_planning/src/search/astar.cpp:29-39`），
与 JPS 的「扁平数组 + 代际计数」形成鲜明对比（§4.2）。它在本文档中的价值主要是**对照基线**。

## 3 算法原理与数学描述

### 3.1 记号约定

| 记号 | 含义 | 出处 |
|---|---|---|
| $r,\ o$ | 栅格分辨率、栅格原点 | `src/map/include/map/grid_map.hpp:42`、`:48` |
| $n_x, n_y$ | 栅格数（x、y 方向） | `src/map/include/map/grid_map.hpp:80` |
| $c(x,y)$ | 第 $(x,y)$ 格的**中心**：$\big((x{+}0.5)r+o_x,\ (y{+}0.5)r+o_y\big)$ | `src/map/include/map/grid_map.hpp:93-94` |
| $a(x,y)$ | 线性地址 $x\cdot n_y+y$ | `src/planner/path_planning/include/path_planning/search/jps.h:73-75` |
| $d(p)$ | 位置 $p$ 到最近障碍的距离（双线性插值） | `src/map/src/grid_map.cpp:60-89` |
| $\tau$ | 安全阈值 `safe_threshold` | `config/planner.yaml:24` |

碰撞语义（两条实现完全一致）：

$$
\text{blocked}(p)=
\begin{cases}
\text{false}, & p\notin \text{map}\quad(\text{地图外视为无碰撞})\\
d(p)<\tau, & p\in \text{map}
\end{cases}
$$

【事实】地图外 `getDistance` 返回 `double` 最大值（`src/map/src/grid_map.cpp:61`），
所以「地图外无碰撞」与同一判据自洽，不需要额外分支
（`src/planner/path_planning/src/search/astar.cpp:112-118`、
`src/planner/path_planning/src/search/jps.cpp:171-175`）。
【事实】$d$ 在障碍内部取**负值**（`src/map/src/grid_map.cpp:262-265`），
故障碍内必然满足 $d<\tau$。

### 3.2 代价与启发

**A\***（以米为单位）：步长 $\rho=1$ 或 $\sqrt2$，代价 $\rho\cdot r$
（`src/planner/path_planning/src/search/astar.cpp:71-72`），启发为欧氏距离
$h(p,q)=\lVert p-q\rVert$（`src/planner/path_planning/src/search/astar.cpp:92-94`）。

**JPS**（以栅格为单位）：射线上走 $n$ 格、方向为对角时代价 $n\sqrt2$，否则 $n$
（`src/planner/path_planning/src/search/jps.cpp:326-328`，其中
$n=\max(|\Delta x|,|\Delta y|)$）。启发用**八叉距离**（octile）：

$$
h(\Delta x,\Delta y)=\max(|\Delta x|,|\Delta y|)+(\sqrt2-1)\min(|\Delta x|,|\Delta y|)
$$

（`src/planner/path_planning/src/search/jps.cpp:63-69`，常数 $\sqrt2$ 定义在 :16）

【推断】在 8 连通、代价 $\{1,\sqrt2\}$ 的度量下 octile 是**一致**启发：它恰好等于
无障碍时按对角优先的最短代价，且沿任一允许移动其下降量不超过该移动代价。
代码本身也依赖这一性质：

- 已展开节点不再被改进——`if (closed_gen_[jlin] == gen) continue;`
  （`src/planner/path_planning/src/search/jps.cpp:324`，注释「一致启发 -> 已展开节点不可能再被改进」）；
- 因此不需要「重开」机制，只需跳过过期堆条目
  （`src/planner/path_planning/src/search/jps.cpp:287`）。

### 3.3 JPS 的两条规则

**(a) 强制邻居判据**（`src/planner/path_planning/src/search/jps.cpp:71-86`）。
设来向为 $(d_x,d_y)$，$\text{blk}(x,y)$ 表示该格被占用：

- 对角来向：$\text{blk}(x-d_x,y)\wedge\neg\text{blk}(x-d_x,y+d_y)$
  或 $\text{blk}(x,y-d_y)\wedge\neg\text{blk}(x+d_x,y-d_y)$；
- 水平来向：$\text{blk}(x,y\pm1)\wedge\neg\text{blk}(x+d_x,y\pm1)$；
- 垂直来向：$\text{blk}(x\pm1,y)\wedge\neg\text{blk}(x\pm1,y+d_y)$。

直观：**如果绕过一个障碍能让路径更短，那么该格必须成为候选**。

**(b) 射线规则**（`jump_ray`，`src/planner/path_planning/src/search/jps.cpp:88-121`）。
从 $c$ 沿 $(d_x,d_y)$ 反复前进一步命中 $(n_x,n_y)$，按顺序判：

1. 该格是终点格 $\Rightarrow$ 立即成功（**在占用检查之前**）——因为精确终点
   已在入口处验过无碰撞，终点格中心可能仍在膨胀区内
   （`src/planner/path_planning/src/search/jps.cpp:95-100`）；
2. 该格被占用/越界 $\Rightarrow$ 射线失败（`:101`）；
3. 该格存在强制邻居 $\Rightarrow$ 它是跳点（`:107-110`）；
4. 若为对角移动，且两个正交分量射线任一存在跳点 $\Rightarrow$ 它是跳点
   （`:113-119`，递归调用 `jump_ray(cx,cy,dx,0)` / `jump_ray(cx,cy,0,dy)`）。

【事实】第 4 条的递归深度有界：正交分量的调用里 `dx != 0 && dy != 0` 为假，
不会再进入递归分支（`src/planner/path_planning/src/search/jps.cpp:113`）。

**(c) 方向剪枝**（`neighbor_directions`，`src/planner/path_planning/src/search/jps.cpp:123-169`）：

| 来向 | 展开的方向 |
|---|---|
| 无（起点） | 全部 8 个（`:134-142`） |
| 对角 $(d_x,d_y)$ | $(d_x,d_y),(d_x,0),(0,d_y)$ + 条件附加 $(-d_x,d_y)$、$(d_x,-d_y)$（`:145-152`） |
| 水平/垂直 | 3 个自然方向（`:153-167`） |

【事实】水平/垂直分支里的两个「条件附加」与它前面**无条件 add 的方向完全相同**，
而 `add` 自带去重（`src/planner/path_planning/src/search/jps.cpp:125-132`），
所以那两行实际不产生新方向（`:158-159`、`:165-166`）；对角分支的两个附加方向是新的，
确有作用（`:151-152`）。这是实现层的冗余，不影响正确性。

### 3.4 搜索主循环

JPS（`src/planner/path_planning/src/search/jps.cpp:192-355`）与 A\*
（`src/planner/path_planning/src/search/astar.cpp:14-90`）都是标准优先队列 A\*，
差别在于「邻居」是射线跳点而不是相邻格：

$$
g(j)=g(i)+n\cdot\rho_{\text{dir}},\qquad f(j)=g(j)+h\big(c(j),c(\text{goal})\big)
$$

A\* 的终止判据是**距离容差**而不是格点相等：
$\lVert p_{\text{cur}}-p_{\text{goal}}\rVert<r$（`src/planner/path_planning/src/search/astar.cpp:53-56`）。

【事实】**A\* 不吸附栅格中心**：它以**原始起点**为根（`src/planner/path_planning/src/search/astar.cpp:37`），
后续位置按 $p_{k+1}=p_k+\rho r$ 步进（`:63`），回溯时原样返回这些位置（`:102-110`）。
只有 JPS 的路径点取自栅格中心（`src/planner/path_planning/src/search/jps.cpp:177-190`），
并把首尾覆盖为精确起终点（`:293-294`）。这与 `docs/nav/planner/path_planning/接口文档.md`
中「仅 JPS 返回栅格中心」的结论一致。

### 3.5 复杂度

| 项 | JPS | A\* |
|---|---|---|
| 时间复杂度 | 最坏同 A\*（$O(N\log N)$，$N$ 为栅格数）；典型扩展数显著更少（作者声明，见 §2.2） | $O(N\log N)$ |
| 单条射线 | 最坏 $O(n_x+n_y)$ 格（`src/planner/path_planning/src/search/jps.cpp:91-120`） | 每格固定 8 邻居（`src/planner/path_planning/src/search/astar.cpp:59-86`） |
| 空间 | $O(N)$ 扁平数组（5 个）+ 优先队列（`src/planner/path_planning/include/path_planning/search/jps.h:66-71`） | $O(N)$ 节点对象 + 哈希表（`src/planner/path_planning/src/search/astar.cpp:33-34`） |

【推断】A\* 每个候选点都要 `new Node`（`unique_ptr` 持有）并插入
`unordered_map<string, Node*>`（`src/planner/path_planning/src/search/astar.cpp:79-83`），
常数与内存明显大于 JPS 的数组写法；且它**没有封闭集**，只有「更优才更新」的判断
（`:77`），同一格被更优路径发现后会再次入队并再次展开。

## 4 工程实现

### 4.1 调用链与开关

| 步骤 | 锚点 |
|---|---|
| 重规划设置地图 | `src/planner/replan/src/fsm_replanner.cpp:60` |
| 起终点沿距离场外推 | `src/planner/replan/src/fsm_replanner.cpp:66-67` |
| **打开 JPS** | `src/planner/replan/src/fsm_replanner.cpp:69`、`:124` |
| 注入速度（供后处理） | `src/planner/replan/src/fsm_replanner.cpp:71-72` |
| 发起搜索，超时 5000 ms | `src/planner/replan/src/fsm_replanner.cpp:74` |
| 门面选择实现 | `src/planner/path_planning/src/path_planning.cpp:41-42` |
| 门面默认值 `false` | `src/planner/path_planning/src/path_planning.cpp:68` |
| 失败即 `nullopt` | `src/planner/path_planning/src/path_planning.cpp:44-46` |

### 4.2 JPS 的数据结构：扁平数组 + 代际计数

【事实】所有搜索状态按 $a(x,y)$ 存在 5 个等长数组里：`open_gen_`、`closed_gen_`、
`g_score_`、`f_score_`、`parent_`（`src/planner/path_planning/include/path_planning/search/jps.h:66-71`），
配一个单调递增的 `search_gen_`：一次搜索只写 $\text{gen}=++\text{search\_gen\_}$，
用「代际是否等于当前 gen」代替整表清零（`src/planner/path_planning/src/search/jps.cpp:251-256`）。

【事实】`set_map` **故意不重置** `search_gen_`，注释给出了原因：`set_map` 每次规划前都会调用，
一旦重置，下一轮会复用旧代际，把上一轮的 `closed_gen_` 误判为当前轮记录，
导致起点刚入队就被当成过期条目跳过，搜索以 `expanded=0` 失败
（`src/planner/path_planning/src/search/jps.cpp:39-43`）。
仅在数组被重建（尺寸变化）或 int32 溢出时才允许复位（`:243-255`）。

### 4.3 占用判定与惰性缓存

【事实】`blocked(x,y)` 用三值缓存（0 未知 / 1 自由 / −1 占用），
按格惰性求值，求值点是**栅格中心**，因此结果等于距离场的原始栅格值
（`src/planner/path_planning/src/search/jps.cpp:46-61`，注释与
`src/map/src/grid_map.cpp:60-89` 的插值实现在中心处退化为该值一致）。
越界一律视为占用（`:48`）。阈值改变时整体失效（`src/planner/path_planning/include/path_planning/search/jps.h:36-39`）。

### 4.4 起点吸附（JPS 独有的补偿逻辑）

【事实】JPS 需要一个**自由格**作为扩展起点，而 A\* 只检查离散点、能容忍起点格被占用；
为保持行为一致，JPS 在起点格不可通行时做环形搜索，吸附到最近的
「自由且至少有一个自由邻居」的格（`src/planner/path_planning/src/search/jps.cpp:381-397`），
半径上限 `max(1, int(1.5/r + 0.5))` ≈ **1.5 m**（`:224`）；
找不到则报错返回（`:226-231`），发生吸附时打警告（`:233-239`）。
【推断】「至少有一个自由邻居」是为了避免吸附进只有一个出口的凹坑/孤立格。

### 4.5 路径重建与首尾约定

【事实】`reconstruct_path` 由 `parent_` 回溯并取每格中心，最后反转
（`src/planner/path_planning/src/search/jps.cpp:177-190`）；
返回前：若未发生吸附，把首点替换为**精确起点**，并始终把末点替换为**精确终点**
（`:291-295`）。这正是后处理能直接在其上做视线检查与插值的前提。

### 4.6 A\* 的实现要点（对照）

| 项 | 做法 | 锚点 |
|---|---|---|
| 节点 | `Node{position, g, f, parent*}`，`operator>` 供最小堆 | `src/planner/path_planning/include/path_planning/search/astar.h:47-56` |
| 存储 | `vector<unique_ptr<Node>>` 持有 + `unordered_map<string, Node*>` 索引 | `src/planner/path_planning/src/search/astar.cpp:33-34` |
| 键 | `int(x*100) , int(y*100)`：**1 cm 截断** | `src/planner/path_planning/src/search/astar.cpp:97-99` |
| 更新 | 仅当 `tentative_g < 记录值` 才新建节点并覆盖键 | `src/planner/path_planning/src/search/astar.cpp:75-84` |
| 终止 | 与目标的欧氏距离 < $r$ | `src/planner/path_planning/src/search/astar.cpp:53-56` |

【推断】键的 1 cm 量化对当前 $r=0.05$ 是安全的（相邻格中心相差 5 cm），
但当 $r<0.01$ 时不同格可能落进同一键（`static_cast<int>` 向零截断）。
【事实】A\* 没有封闭集，也没有过期条目判据；被更优路径更新的格会再次入队。

### 4.7 失败分支

| 分支 | JPS | A\* |
|---|---|---|
| 地图未设置 | `src/planner/path_planning/src/search/jps.cpp:196-199` | 无此检查（直接搜索空图） |
| 起点碰撞 | `src/planner/path_planning/src/search/jps.cpp:202-205` | `src/planner/path_planning/src/search/astar.cpp:19-22` |
| 终点碰撞 | `src/planner/path_planning/src/search/jps.cpp:206-209` | `src/planner/path_planning/src/search/astar.cpp:23-26` |
| 超时 | `src/planner/path_planning/src/search/jps.cpp:277-281` | `src/planner/path_planning/src/search/astar.cpp:43-48` |
| 无路 | 带诊断上下文（根格、占用邻居数、扩展数）`src/planner/path_planning/src/search/jps.cpp:345-354` | 静默返回空 `src/planner/path_planning/src/search/astar.cpp:89` |

【事实】两者的超时都是**每次弹出节点时**检查，粒度一致
（`src/planner/path_planning/src/search/jps.cpp:276-281`）。

## 5 关键参数与细节

| 参数 / 常量 | 当前值 | 读取点 | 生效位置 |
|---|---|---|---|
| `safe_threshold` | 0.35 m | `config/planner.yaml:24` → `src/ros2/include/ros2/config.hpp:236` | 占用判定 `src/planner/path_planning/src/search/jps.cpp:59`、`src/planner/path_planning/src/search/astar.cpp:117` |
| 搜索超时 | **5000 ms**（调用点） | `src/planner/replan/src/fsm_replanner.cpp:74` | 主循环弹出检查 `src/planner/path_planning/src/search/jps.cpp:277` |
| 超时默认值 | 1000 ms | `src/planner/path_planning/include/path_planning/search/jps.h:45` | 仅在直接调用时生效 |
| 栅格分辨率 | 0.05 m | `config/map.yaml:64` | 栅格中心与吸附半径 `src/planner/path_planning/src/search/jps.cpp:224` |
| 吸附半径 | $1.5/r\approx30$ 格 | `src/planner/path_planning/src/search/jps.cpp:224` | 起点恢复 `:381-397` |
| 对角步长常数 | $\sqrt2$ | `src/planner/path_planning/src/search/jps.cpp:16` | 启发 `:68`、代价 `:327` |
| `use_jps` | `true`（生产） | `src/planner/replan/src/fsm_replanner.cpp:69` | 门面选择 `src/planner/path_planning/src/path_planning.cpp:41-42` |

【事实】搜索层**不读** `max_vel` / `max_acc` / `time_resolution` 等运动学参数：
它们只在后处理阶段使用（`src/planner/path_planning/src/post_processing.cpp:210-266`）。
即搜索与运动学是**解耦**的。

## 6 已知限制与优化方向

| # | 限制 | 依据 | 类型 |
|---|---|---|---|
| 1 | **对角移动允许"切角"**：对角前进一步只检查目标格，不检查两个正交邻格；A\* 同 | `src/planner/path_planning/src/search/jps.cpp:92-101`、`src/planner/path_planning/src/search/astar.cpp:59-68` | 【事实】 |
| 2 | 切角的实际风险被安全阈值缓解：占用判定本身带 $\tau=0.35$ m 膨胀（≈7 格），两障碍夹角处通常已判占用 | 由 #1 与 `src/planner/path_planning/src/search/jps.cpp:59` 推导 | 【推断】 |
| 3 | 搜索完全无运动学约束，折角处需要后处理用"等效距离"补偿，代价是折角处明显变慢 | `docs/nav/开源参考/RM2025中科大导航.md:283`；本项目实现见 [planner/03](./03-路径后处理与时间分配.md) §3.4 | 【事实】+【推断】 |
| 4 | A\* 没有封闭集，可能重复扩展同一格 | `src/planner/path_planning/src/search/astar.cpp:75-84` | 【事实】 |
| 5 | A\* 的哈希键是 1 cm 截断，$r<0.01$ 时可能碰撞 | `src/planner/path_planning/src/search/astar.cpp:97-99` | 【推断】 |
| 6 | 每次 `set_map` 都**值拷贝**整张 `GridMap`（`map_ = map;`），且门面对 A\*、JPS、后处理各拷一次 | `src/planner/path_planning/src/search/jps.cpp:23-24`、`src/planner/path_planning/src/path_planning.cpp:7-10` | 【事实】 |
| 7 | 上述拷贝的规模：0.05 m 下 $n_x\cdot n_y=(28/0.05)\times(16/0.05)=560\times320\approx1.8\times10^5$ 格（x∈[−6,22]、y∈[−8,8]），每帧 3 份即约 5×10⁵ 格级缓冲 | `config/map.yaml:60-63`、`src/map/include/map/grid_map.hpp:56-57`、`src/map/src/grid_map.cpp:15` | 【推断】 |
| 8 | JPS 的代际数组尺寸随地图变化重建；`search_gen_` 一旦被误重置会导致搜索立即失败（代码注释已警示） | `src/planner/path_planning/src/search/jps.cpp:39-43` | 【事实】 |
| 9 | 「少 1~2 个数量级扩展」缺乏仓库内实测数据 | `src/planner/path_planning/include/path_planning/search/jps.h:14-16` | 【待确认】 |
| 10 | 吸附半径 1.5 m 硬编码在搜索层，不随 `safe_threshold` 或地图尺度变化 | `src/planner/path_planning/src/search/jps.cpp:224` | 【事实】 |

**优化方向**（均未实现，供作者取舍）：

1. **消除每帧整图拷贝**：把 `GridMap` 成员改成 `shared_ptr`/引用，或让门面只传一次；
   预计收益随栅格数与规划频率线性增长（对应 #6/#7）。
2. **对角切角检查**：在对角移动时增加两个正交邻格的占用判定（一行判断），
   代价是射线略慢、路径略保守；是否必要取决于实车对夹角的容忍度（对应 #1）。
3. **给 A\* 补封闭集与过期条目判据**，或直接删掉 A\*（当前默认路径不走它），
   以减少读者与维护者的误判（对应 #4）。
4. **把吸附半径提为参数**，与 `safe_threshold` 同源配置（对应 #10）。
5. **补一个离线基准**（同一张栅格图、同一组起终点，对比 JPS/A\* 的扩展数与耗时），
   把"少 1~2 个数量级"从注释变成可复现数据（对应 #9）。
