# nav 算法思路文档族（docs/nav/算法思路/）改造记录

记录日期：2026-10-02
任务范围：**新增** `docs/nav/算法思路/`（1 篇总览 + map 3 篇 + planner 6 篇，共 10 篇 / 2724 行 / 842 条锚点）
写作风格：参考 `docs/nav/开源参考/` 中三份兄弟队伍技术报告（问题动机 → 核心思想 → 数学建模 → 工程实现 → 参数细节 → 限制与优化）
**未改动任何源码**

---

## User Intent

1. 用户要求：「`@nav/算法思路/` 结合代码 完善该项目的算法 架构、思路、核心算法、细节」。
2. 追加要求：「参考 `@nav/开源参考/` 中的算法文档的写法」——即按技术报告的叙述与数学风格写，
   而不是按 `docs/nav/<模块>/` 那套 9 文件工程契约写。
3. 仓库级约束（`AGENTS.md`）：不改 `src/`、不编造、最小改动、禁止未授权构建、禁止提交类操作。

---

## Scope

- 新增 `docs/nav/算法思路/README.md`（总览：架构、数据流、参数速查、与开源参考对照、阅读顺序）
- 新增 `docs/nav/算法思路/map/{01-概率占据建图与滑窗,02-地形与隧道语义,03-ESDF距离场与避障梯度}.md`
- 新增 `docs/nav/算法思路/planner/{01-导航框架与数据流,02-全局搜索JPS与A星,03-路径后处理与时间分配,04-轨迹优化MINCO与五次样条,05-MPC控制器,06-重规划策略}.md`
- 本记录

## Out of Scope

1. 不改任何源码/参数/launch/CMake。
2. 不把本目录纳入 `verify_docs.py` 的 9 文件契约（它面向工程接口，本目录面向算法叙述）。
3. 不重复 `docs/nav/<模块>/` 已有的接口/参数表，只引用它们。

---

## Explorer Findings

### Files inspected

- 风格样例：`docs/nav/开源参考/RM2026哈工大(威海)哨兵导航方案.md`（976 行，数学最全）、
  `RM2025中科大导航.md`（529 行）、`RM2026北京理工大学追梦战队哨兵导航.md`（465 行）
- 已审核的模块文档（作为权威事实来源）：`docs/nav/map/*.md`、`docs/nav/planner/*/*.md`、
  `docs/nav/ros2/*.md`、`docs/nav/utils/*.md`
- 参数真值：`docs/_evidence/repo/params.json`（map.yaml + planner.yaml 全键）
- 代码：`src/map/**`、`src/planner/**`、`src/ros2/**`、`src/ma_nav.cpp`

### Active logic path（写进 README 与 planner/01）

```
/lio/cloud_world + /lio/base_odom → map（概率占据 + 2D 栅格 + 地形/隧道语义）
  → planner 33 ms：plan_omni → replan FSM → JPS/A* → 后处理（5D 采样 + 时间分配 + 折叠降速）
  → MINCO/五次样条优化（两阶段 L-BFGS）
  → controller 10 ms：4 状态 MPC（OSQP + 热启动）→ 预测状态第 1 步速度 → world→body → /cmd_vel_chassis
  → 折叠事件 → /ma_nav/fold
```

### Risk notes（写文档时必须如实处理的“反直觉”点）

1. **单线程**：`spin_some`，三回调组只是分组声明（不能写成“并发”）。
2. **ESDF 的真相**：关闭的是 3D 概率图 ESDF 子地图；优化器实际用 **2D 栅格自带的有符号 EDT**
   （`GridMapESDF` 适配 2D `GridMap`，注入点 `src/planner/replan/src/fsm_replanner.cpp:92-93`）。
3. **MPC 输出未被使用**：调用方丢弃 `u_cmd`，改用 `predicted_states[1]` 的速度分量组 Twist。
4. **H≠2Q**：Hessian 只取对角，线性项用整矩阵；**Rd 无线性项 ⇒ 其非对角完全无效**。
5. **重规划标志残留**：`need_replan_` 初值为真、失败后不清 ⇒ 三条件都不成立也可能重规划。
6. **优化失败无兜底**：注释承诺与实现不一致。
7. **死代码**：`check_trajectory_collision`、`evaluate_duration/length`、`last_opt_path_`、
   `min_replan_interval`、`one_plan` 等（各篇按模块登记）。

### Recommended modification boundary

只新增 `docs/nav/算法思路/**`；所有数值必须来自 `params.json` 或 `config/*.yaml`；
所有结论必须能在代码或已审核模块文档中落地；推导标【推断】，冲突标【待确认】。

---

## Modifier Changes

### Files changed

| 文件 | 行数 | 主要内容 |
|---|---|---|
| `docs/nav/算法思路/README.md` | 113 | 系统架构、端到端数据流、全链路参数速查、与开源参考对照、阅读顺序、文档约定 |
| `map/01-概率占据建图与滑窗.md` | 320 | 对数几率贝叶斯更新与饱和、批量写回门控、**遗忘每 5 帧**、射线投射与候选累积、未知/膨胀、滑窗与原点滑动 |
| `map/02-地形与隧道语义.md` | 312 | 地形双判据（陡边 0.17、间隙硬编码 0.15）、可通行面、**隧道语义写入与折叠因果链**、区域 YAML 存取 |
| `map/03-ESDF距离场与避障梯度.md` | 190 | 两套 ESDF 的分工、2D 有符号 EDT、双线性+中心差分梯度、3D 局部盒实现、图外语义 |
| `planner/01-导航框架与数据流.md` | 222 | 单线程调度、节拍、坐标系约定、模块边界表、端到端时序 |
| `planner/02-全局搜索JPS与A星.md` | 317 | JPS 跳跃/强制邻居、A\* 差异、吸附与超时（实际 5000 ms）、复杂度与工程细节 |
| `planner/03-路径后处理与时间分配.md` | 298 | 剪枝、5D 采样、等效距离+加减速锥+平均速度积分、折叠降速、输出字段只写不读 |
| `planner/04-轨迹优化MINCO与五次样条.md` | 320 | 微分平坦、五次样条/最小 jerk 闭式、决策变量、两阶段 L-BFGS、采样代价、2D 下 yaw 置零 |
| `planner/05-MPC控制器.md` | 312 | 双积分模型与一阶欧拉、QP 标准型、**124 变量/164 约束**、H≠2Q、约束与求解、输出未被使用 |
| `planner/06-重规划策略.md` | 320 | 三类触发与形式化判据、标志残留、硬净空、起终点外推、20 s 语义、折叠事件、失败无兜底 |

### Key changes

1. **先读已审核模块文档，再读代码**：本目录不复述接口表，只讲动机/数学/因果，事实与模块文档对齐。
2. **把“反直觉点”写进正文**（而不是藏起来）：单线程、u_cmd 被丢弃、Rd 无效、标志残留、失败无兜底、
   `evaluate_*` 零调用点、`time_segments` 等字段只写不读。
3. **纠正一处我自己的错误**：README 初稿把 2D 栅格范围写成 `x∈[6,22]`，
   实际（`config/map.yaml:60-63` + `grid_map.cpp:6-7`）= `x∈[−6,+22]`、`y∈[−8,+8]`，28 m × 16 m、560×320 格。
4. **统一锚点口径**：把 `cost.hpp:81`、`jps.cpp:59` 这类**裸文件名锚点**全部展开为完整相对路径。

### Behavior preserved

- `src/`、`config/`、`launch/`、`CMakeLists.txt` 零改动（`git status --porcelain -- src config launch` 为空）。
- 未修改 `docs/nav/开源参考/`（只读参考）与既有 63 篇模块文档正文（仅本轮之外的历史修正）。

### Behavior intentionally adjusted

1. 本目录**不参与** 9 文件契约校验，篇幅与结构服从技术报告写法；但涉及行为/数值的结论都要求有模块文档或代码支撑。
2. README 的「与开源参考对照」表明确写出差异（例如参考文档假设 3D ESDF 梯度，我们实际是 2D 距离场 + 硬阈值）。

### Notes

- 写作分工：父代理写总览，5 路子代理各写 1–2 篇（互不重叠的文件范围）；父代理统一做机械校验与修正。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：7 个模块的调用链与节拍在 README 与 planner/01 中逐条对齐代码
- [x] 锚点机械校验：**842 条锚点全部通过**「文件存在 + 行号范围」检查（0 失败）
- [x] 裸锚点修正：9 组裸文件名锚点（21 处）展开为完整路径
- [x] 数值抽检：README 参数速查表逐条对 `params.json`（map.yaml / planner.yaml）
- [x] 关键反直觉结论回源复核（父代理亲自核对）：
      `GridMapESDF` 包 2D `GridMap` 并在 `fsm_replanner.cpp:92-93` 注入 ✓；
      `grid_map.cpp:200-269` 为**有符号** EDT（障碍内取负）✓；
      `maps/` 目录不存在 ⇒ `tunnel_regions_path` 自动保存当前必失败 ✓；
      MPC 124/164、H≠2Q、Rd 无线性项、u_cmd 被丢弃 ✓（与本轮早前的独立审核一致）
- [x] 用户允许范围内的测试或静态检查：仅只读检索与 Python 校验脚本
- [x] 如需构建，已取得用户明确许可：**未执行构建**
- [x] 提交类操作：**未执行**

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | README 初稿的 2D 栅格范围写错（`x∈[6,22]`，实为 `[−6,+22]`） | 已按 `config/map.yaml:60-63` 与 `grid_map.cpp:6-7` 修正为 28 m × 16 m / 560×320 格 |
| 2 | README 初稿把避障写成「依赖膨胀占据与安全距离判定」，与事实不符 | 已改为「3D ESDF 关闭；距离场来自 2D 栅格自带的有符号 EDT（`GridMapESDF`）」 |
| 3 | `map/03` 与 `planner/01` 中存在 9 组裸文件名锚点（21 处） | 已展开为完整相对路径，全族锚点校验 0 失败 |
| 4 | 子代理反馈「任务提示里的 `p_prev` 在代码中不存在」 | 已由该篇标【待确认】；`MpcParam` 只有 `u_prev`（写而不读） |
| 5 | 子代理发现 `ros2.tunnel_regions_path` 指向不存在的 `maps/` | 已写入 `map/02` 的限制表；父代理复核确认 `maps/` 不存在 |

### Final result

**PASS**

判据：

- 10 篇 / 2724 行，结构统一（1 方案概述 → 2 核心思想 → 3 数学建模 → 4 工程实现 → 5 关键参数 → 6 限制与优化），
  非 ASCII 中文技术报告风格与 `开源参考/` 一致；
- **842 条锚点全部可解析且行号合法**；
- 所有数值来自 `params.json`/`config/*.yaml`；所有反直觉结论都能贴出代码原文；
- 范围合规：源码零改动，未执行构建，未执行提交类操作。

---

## 未解决问题（留给后续）

1. 本目录**未纳入自动校验**：目前只有「锚点存在性/行号」这类机械校验，语义仍靠人工审核；
   若要把 10 篇也纳入 `verify_docs.py`，需要新的契约（本目录不是 9 文件结构）。
2. 各篇的【待确认】项汇总（供作者决策）：`kThreshold=0.5` 出处；
   `safe_threshold`(planner.yaml:24) 与 `safe_distance`(:47) 同值但独立、只改一个会口径错位；
   图外距离语义三处不一致；3D ESDF 关闭是否有意；20 s 重置语义；
   折叠补发指令被注释是否有意；`start_state`/`final_state` 命名（实为边界速度）。
3. 子代理提出的「优化方向」是**建议**，未改动代码：
   失败帧兜底、软约束加硬边界、H 与线性项口径统一、yaw 通道接线、
   多线程化后重估锁范围、`maps/` 目录与绝对路径的工程化处理。
