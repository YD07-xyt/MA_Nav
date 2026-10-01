# traj_optimize 模块文档（步骤 4 之二）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/planner/traj_optimize/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**
顺序：本轮为「按顺序完成 planner 三个模块」的第 1 个（traj_optimize → controller → replan）

---

## User Intent

1. 用户要求：**按顺序**完成 `nav/planner/{traj_optimize,controller,replan}` 的文档，
   **不要并行完成**（一个模块收尾后再开下一个）。
2. 约束沿用 `nav/example/README.md` 契约：只基于 Evidence Pack、9 个固定文件、
   每句事实带锚点、不确定项显式降级并进入 `待确认清单.md`。
3. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不改代码/参数、
   禁止未授权构建、禁止任何 git 提交类操作、禁止 writingplans。

---

## Scope

- `docs/nav/planner/traj_optimize/`：新增 9 篇文档
- `docs/_evidence/traj_optimize/inventory.json`：新增证据（17 文件 / 1872 符号 / 0 失败）
- `docs/_evidence/verify.traj_optimize.json`：校验报告
- `docs/ai_refactor_records/`：本记录

## Out of Scope

1. 不改任何被文档化的代码（`src/`、`config/`、`launch/`、`CMakeLists.txt`、参数默认值）。
2. 不写 `controller`、`replan` 的文档（按用户要求顺序后排）。
3. 不重新生成 `docs/_evidence/repo/*.json`（理由同 path_planning 记录：会改变 map 的缺口口径）。
4. 不清理模块内的停用副本（`spline_opt/**`、`gcopter/{minco,lbfgs,sdlp}.hpp`）——只登记。
5. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读，17 个文件 / 18228 行）：

- `src/planner/traj_optimize/CMakeLists.txt`
- 生效：`ma_spline_opt/optimizer_config.h`(155)、`ma_spline_opt/traj_optimizer.h`(481)、
  `ma_spline_opt/cost.hpp`(228)、`ma_spline_opt/SplineTrajectory/SplineOptimizer.hpp`(2586)、
  `ma_spline_opt/SplineTrajectory/SplineTrajectory.hpp`(3700)、`esdf_Interface.hpp`(41)、
  `grid_map_esdf.hpp`(50)、`gcopter/trajectory.hpp`(606)、`gcopter/root_finder.hpp`(1113)、
  `src/ma_spline_opt/traj_optimizer.cpp`（37 字节，仅一行注释）
- 停用：`spline_opt/**`（4 文件）、`gcopter/{minco,lbfgs,sdlp}.hpp`（3 文件）
- 作者自带说明：`opt.md`(4)、`ma_spline_opt/problem.md`(30)、
  `ma_spline_opt/SplineTrajectory/SplineOptimizerProtocols_zh.md`(311)

调用方与跨模块（只读）：

- `src/planner/replan/include/replan/fsm_replanner.h`、`src/planner/replan/src/fsm_replanner.cpp`
- `src/planner/controller/include/controller/traj_interface.hpp`
- `src/ros2/include/ros2/misc/visualizer.hpp`、`src/ros2/include/ros2/config.hpp`、`src/ros2/src/ros2_node.cpp`
- `src/utils/include/utils/{lbfgs.hpp,expected.hpp,logger.hpp}`
- `config/planner.yaml`、`CMakeLists.txt`、各下游 `CMakeLists.txt`

### Active logic path

```
replan: set_config(fsm_replanner.h:44) → set_esdf_interface(&GridMapESDF)(fsm_replanner.cpp:93)
  → ma_opt_.optimize(from_path_planning_trajectory(traj))(fsm_replanner.cpp:94,100)
      → MaSplineTrajectoryOptimizer::optimize 按 model 分派（traj_optimizer.h:165-193）
          → optimize_xy(2D)：时间点+航点+bc+mask → makeProblemFromTimePoints
              → optimize_2d_stage(stage1) → optimize_2d_stage(stage2, 以 stage1 解为初值)
              → synchronizeWorkingState → getWorkingSpline → extend_xy_with_zero_yaw
          → optimize_xy_yaw_joint(3D)：同上但用三维样条与 yaw 边界
  → 失败：MINCO_OPT_FIALED（fsm_replanner.cpp:101-104）
  → 成功：ros2 侧转成 controller 的轨迹接口（ros2_node.cpp:308-321）
```

### Data flow

- **参数**：`config/planner.yaml:45-89` → `ros2 config.hpp:188-203`（顶层 8 个 + 两个 stage）
  → `FsmReplan::set_param` → `set_config` → 阶段函数组装 `TimeCost` / `RobotIntegralCost`。
- **距离场**：`grid_map_esdf.hpp:27` 适配器把 2D 栅格地图的 ESDF 暴露成 CRTP 接口，
  由调用方以裸指针注入（`traj_optimizer.h:29-31`）。
- **输出**：三维五阶样条 + 时间段；下游只读 `success` 与 `trajectory`。
- **ROS 接口**：本模块零 ROS 接口（无 rclcpp、无 YAML 解析）。

### Risk notes

1. **两套同名实现**：`ma_spline_opt/**` 与 `spline_opt/**` 都有
   `SplineTrajectory.hpp` / `SplineOptimizer.hpp`，且**行号不同**
   （例如 `QuinticSplineND` 分别在 1303 与 1535 行）。作者 `opt.md:1-4` 明确取舍，
   但读者/AI 极易引用错副本 → 文档中每处都标注了生效/停用。
2. **gcopter 与作者说明不一致**：`opt.md:4` 说 gcopter 未使用，
   但 `src/ros2/include/ros2/misc/visualizer.hpp:7` 仍 include `gcopter/trajectory.hpp`，
   且该头把 `Piece` / `Trajectory` 定义在**全局命名空间**（`gcopter/trajectory.hpp:38`、`:348`）。
3. **空实现 .cpp**：`src/ma_spline_opt/traj_optimizer.cpp` 只有一行注释（37 字节、无换行），
   全部实现是模板化的头文件代码。
4. **成功判据宽松**：阶段函数只判「返回码非负 + 代价有限 + 决策向量有限」
   （`traj_optimizer.h:321`），未收敛也会返回成功。
5. **上下文与配置顺序敏感**：协议文档说明配置只影响之后新准备的上下文
   （`SplineOptimizerProtocols_zh.md:233-234`）。
6. **参数默认值齐全**：`StageOptimizerConfig` / `MaSplineOptimizerConfig` 每个字段都有初值
   （`optimizer_config.h:14-57`），缺键不会出现未初始化值——与 path_planning 的坑相反。
7. **下游只读两个字段**：`MAsplineOutput` 的 `time_segments` / `start_time` / `cost` 外部无消费者。
8. `src/` 下仍存在本轮之前的未提交改动（controller 的 mpc.cpp、本模块的 traj_optimizer.cpp、
   未跟踪的 opt.md）→ 文档声明 `worktree_dirty: true`。

### Recommended modification boundary

只新增 `docs/` 下文件与一个模块证据 JSON；停用副本只登记不删除；
不在文档里对停用副本给出实现细节（避免与生效副本混淆）。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/planner/traj_optimize/README.md` | 新增：背景、概述、边界、入口、**17 文件清单（含生效/停用标注）** |
| `docs/nav/planner/traj_optimize/模块设计说明.md` | 新增：需求约束（problem.md）、类协作、数据流、关键决策、线程与并发、9 条已知限制 |
| `docs/nav/planner/traj_optimize/接口文档.md` | 新增：文件表 + 核心类型语义表 + **234 行机械生成的符号索引** |
| `docs/nav/planner/traj_optimize/依赖关系.md` | 新增：CMake、外部库、内部 include 图、双向跨模块依赖、生效/停用分组、6 条风险 |
| `docs/nav/planner/traj_optimize/流程图.md` | 新增：主数据流、单阶段时序、代价构成、错误码状态图 |
| `docs/nav/planner/traj_optimize/配置说明.md` | 新增：机制 + 36 个叶子键三张表 + 11 处默认值差异 + 缺口 |
| `docs/nav/planner/traj_optimize/测试要点.md` | 新增：14 条风险点、6 条边界、可观测量、无法验证部分（全【推断】） |
| `docs/nav/planner/traj_optimize/常见问题FAQ.md` | 新增：Q1–Q8 + 参数类问题 + 排查手段 |
| `docs/nav/planner/traj_optimize/待确认清单.md` | 新增：3 条参数缺口、22 条上下文未知读取、11 条语义待确认、6 条其他 |
| `docs/_evidence/traj_optimize/inventory.json` | 新增（工具产物） |
| `docs/_evidence/verify.traj_optimize.json` | 新增（校验报告） |

### Key changes

1. **先判定「哪一套生效」再写文档**：依据 `opt.md:1-4` + 全仓库 include 检索，
   把 17 个文件分成「生效 10 / 停用 7」，并在 README、依赖关系、接口文档三处一致标注。
2. **符号索引改为机械生成**：1872 条符号不可能手写且必然出错，
   接口文档的「全量符号索引」由脚本从 `inventory.json` 生成
   （234 行，只列类/枚举/函数/变量，方法/字段在人工小节里），
   脚本不产出任何语义文字。
3. **每个参数表都给出 YAML 真值 + 代码默认值 + 读取行号**（36 个叶子键），
   并单列 11 处默认值差异（stage2 与默认值完全一致，stage1 有 6 处不同）。
4. **把「作者说明」与「代码现状」分开写**：`opt.md` 的取舍作为【事实】引用，
   gcopter 被 include 的现状作为【待确认】登记，不用一句话抹平矛盾。
5. **风险点全部指向可点开的代码行**：未收敛也算成功、上下文/配置顺序、图外不惩罚、
   首末点固定、全局命名空间污染、两套同名头等。

### Behavior preserved

- 被文档化代码零改动（`src/`、`config/`、`launch/`、`CMakeLists.txt`）。
- 未修改 `docs/tools/*`、`docs/nav/example/*`、`docs/nav/map/*`、path_planning 文档。
- 未删除任何停用副本文件。

### Behavior intentionally adjusted

1. **接口文档采用「人工语义 + 机械索引」混合结构**：
   与 map/path_planning 金样例的纯人工表不同，本模块符号量是 path_planning 的 9.7 倍，
   全人工列表既不现实也更容易编造；机械索引保证「每个符号都能点开」，
   语义只写在我真的读过的核心类型上。
2. **停用副本不给符号级文档**，只给规模与判定依据，避免把死代码写成可用接口。

### Notes

- 库存工具沿用上一轮已编译好的二进制（未触发新构建）。
- 新增模块证据会再次改变全局 `evidence_digest`（本轮为 `a1a05a6ce3e99b62`），
  三个模块全部完成后需要统一重基线所有模块文档的该字段（见「未解决问题」）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：门面 → 两类阶段函数 → 上下文/求值规格 → 代价结构逐条对源码核过
- [x] diff 检查：`git status --porcelain -- docs/` 核对，改动全部落在
      `docs/nav/planner/traj_optimize/`、`docs/_evidence/`、本记录
- [x] grep 检查：`rclcpp|yaml` 确认零 ROS/零 YAML；`withExecutor` 确认未启用并行执行器；
      `last_xy_spline_|has_last_traj_|check_trajectory_collision` 确认无使用点；
      `time_segments|start_time|cost` 确认下游只读 `success`/`trajectory`
- [x] XML / launch / yaml 检查：参数真值来自 `params.json` 的解析值（36 个叶子键）
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module traj_optimize` → **errors=0**，coverage **17/17**，
      warnings=9（全部为人工审核闸门 `review.pending`）
- [x] 注入测试：对 traj_optimize 副本注入假话题 `/cmd_vel`、越界行号
      `cost.hpp:99999`、编造符号 `ma_spline_opt::TimeCost::nonexistentMethod`，
      三类**全部**被识破，注入副本 `status=FAIL`
- [x] 独立审核 Agent（不同上下文）：结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**任何 git 提交类操作

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 首轮 17 个 error：`FsmReplan` / `gcopter` / `spline_opt` 等**目录名或跨模块类名**被当作符号要求出现在锚点附近 | 去掉反引号或改用文件锚点；未放宽校验器 |
| 2 | `待确认清单` 的参数表只给键路径、没给 YAML 值 → `param.value_mismatch` 4 处 | 表格补「YAML 值」列 |
| 3 | 两处中间节点（`planner_config.ma_spline_opt_params`）被加反引号当作 YAML 键 → `param.unknown_key` | 改为散文表述（键路径前缀用中文描述） |
| 4 | 「键路径与代码默认值不一致项」小节无锚点/标注 | 补一条【事实】收尾 |
| 5 | **自查发现的批替换事故**：修正停用副本行号时用了子串替换，误伤了 `ma_spline_opt/...`（活动副本）的 6 处锚点 | 立即发现并用反向替换修正，重跑校验器确认 errors=0 |
| 6 | 停用副本与活动副本**行号不同**（`QuinticSplineND` 1303 vs 1535，`SplineOptimizer` 408 vs 330） | 逐个文件按 inventory 精确取值，不再用文件名子串匹配 |
| 7 | **独立审核**：把「可视化重载无调用点」误写成「gcopter 类型未被使用」，横跨 4 个文件 | 核实后发现该重载确实以 gcopter 的 Trajectory 为形参并调用其接口，只是唯一调用处被注释（`src/ros2/src/ros2_node.cpp:306`）；4 处全部改写为「重载使用了该类型但无调用点、模板未实例化」 |
| 8 | **独立审核**：依赖关系的外部库锚点错位（Eigen3 写成 :16、utils 写成 :15） | 改为 :15 / :16（`src/planner/traj_optimize/CMakeLists.txt:14-17` 顺序） |
| 9 | **独立审核**：避障决策的锚点只覆盖平滑 L1，未覆盖正交投影 | 补 `cost.hpp:90-119` |
| 10 | **独立审核**：错误码结论的锚点与结论不对位 | 补 `traj_optimizer.h:41-45` |
| 11 | **独立审核**：「模块内单头最大 3700 行」不成立（停用副本 3976 行） | 改为「生效头最大 3700 行（停用副本另有 3976 行）」 |
| 12 | **独立审核**：「唯一可离线使用的正确性工具」过度概括 | 限定为「唯一可离线跑的数值对照工具」，并补编译期 `static_assert` 校验的锚点 |

### 独立审核 Agent（不同上下文）

按 `docs/nav/example/README.md` §4.1 起了一个不同上下文的只读审核 Agent，
它自己重新取证并额外用脚本机械比对了三张表。**复核通过**（它自己的结论）：

- 不使用 rclcpp / 不解析 YAML；
- **两阶段初值传递**成立（stage1 的解经 `x` 引用传入 stage2）；
- 36 个参数键的 YAML 值 / 读取行号 / 代码默认值三处一致，
  stage2 与默认值完全一致、stage1 恰好 6 处不同；
- 代价公式描述与 `cost.hpp` 一致（平滑 L1 阈值 1e-2、正交投影与小梯度增强、图外不惩罚、yaw 仅三维）；
- `MAsplineOutput` 的 `time_segments` / `start_time` / `cost` 全仓库无人读；
- `last_xy_spline_` / `has_last_traj_` 只有声明；`check_trajectory_collision` 无调用点；
- 未启用并行执行器（`withExecutor` 全仓库只有定义、零调用）；
- 停用判定成立，符号分组 930 / 798 / 144 与文档一致；
- **234 行符号索引与证据逐条匹配、0 处不符**；配置表 36 行与 47 行默认值列零不符。

审计结论：首轮 **NEEDS_FIX** —— 1 处语义级错误（第 7 条）+ 1 处锚点错位（第 8 条）
+ 4 处锚点/绝对化轻量缺陷，已逐条修复并重跑校验器（仍 `errors=0`）。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 **17 个源文件全部被文档提及**（含 7 个停用副本文件），缺失 0
- 注入测试：三类幻觉在 traj_optimize 副本上全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 6 条发现已**逐条修复并重新校验**，修后 `errors=0`
- 范围合规：`src/`、`config/`、`launch/`、`CMakeLists.txt` 零改动；未执行构建；未执行提交类操作

---

## 未解决问题（留给后续）

1. **三个模块完成后的统一重基线**：新增 traj_optimize 证据使全局 `evidence_digest`
   变为 `a1a05a6ce3e99b62`，`docs/nav/map/` 与 `docs/nav/planner/path_planning/` 的
   记录值随之过期（`evidence.stale` 警告）。计划在本轮三个模块全部完成后，
   一次性把各模块文档的 `evidence_digest` 更新到最终值。
2. **map 文档的 `worktree_dirty: false` 与实际不符**（`src/` 有未提交改动）——**已解决**，见文末「map 模块单独重新基线」。
3. 人工审核闸门：本模块 9 篇仍为 `status: draft` / `reviewer: 未审核`。
4. 模块内停用副本（`spline_opt/**`、`gcopter/{minco,lbfgs,sdlp}.hpp`）是否清理，
   以及 `visualizer.hpp` 对 `gcopter/trajectory.hpp` 的无用 include 是否删除，需作者决定。
5. 代码侧待办（未改代码，仅登记）：未收敛即成功、上下文/配置顺序、`evaluatePrepared`
   前置条件、图外不惩罚、首末点固定、全局命名空间污染。

---

## 收尾重基线（2026-10-02 追加）

用户要求的三个模块（traj_optimize → controller → replan）按顺序完成后，
本轮又补做了 path_planning，共 **4 个模块的证据包**进入 `docs/_evidence/`，
使全局 `evidence_digest` 依次变化；现已在收尾步骤统一为最终值 `7c453c4e8af72579`。

| 模块 | 文档数 | 重基线前摘要 | 重基线后 | 收尾校验结果 |
|---|---|---|---|---|
| path_planning | 9 | e92ac2dc4614a515 | 7c453c4e8af72579 | errors=0 / review.pending=9 / 覆盖 8/8 |
| traj_optimize | 9 | a1a05a6ce3e99b62 | 7c453c4e8af72579 | errors=0 / review.pending=9 / 覆盖 17/17 |
| controller | 9 | cc5179098b163a1c | 7c453c4e8af72579 | errors=0 / review.pending=9 / 覆盖 4/4 |
| replan | 9 | 7c453c4e8af72579（写入时即为最终值） | — | errors=0 / review.pending=9 / 覆盖 2/2 |

四个模块的 36 篇文档现在指向**同一份证据包状态**，非 `review.pending` 的警告为 0。

**map 模块随后单独重新基线（同日完成）**：见文末「map 模块单独重新基线」。

---

## map 模块单独重新基线（2026-10-02 追加）

用户随后要求**单独**重新基线 `docs/nav/map/`。本轮做的事：

1. **先验证证据可复现（非破坏性）**：按 `example/README.md` §9 的命令把 4 份证据重跑到临时目录再逐字段比对
   （忽略 `generated_at` / `git_rev` / `root` 三个易变字段）：
   `repo/ros2.json`、`repo/params.json`、`repo/launch.json` 与基线**逐字段一致**；
   map 符号表重跑为 22 文件 / 598 符号 / 0 失败，与基线数量、文件表完全一致。
2. **发现 1 条不可位级复现的符号并保留更准确的基线**：重跑把
   `rog_map::ProbMap::Ptr`（第 38 行，与源码 `typedef std::shared_ptr<ProbMap> Ptr;` 一致）
   报成 `ProbMap::shared_ptr`（第 37 行的宏行），属 clangd `documentSymbol` 在宏相邻别名上的偶发归属差异。
   据此**没有覆盖基线证据文件**，而是把该观察登记进 `docs/nav/map/待确认清单.md` 第 8 条。
3. **更新 9 篇 front-matter**：`evidence_digest: 1ab15719a7937f37 → 7c453c4e8af72579`、
   `worktree_dirty: false → true`（`git_rev` 仍为 `2eed7d3e884d`）。
   后者是因为 `src/` 下有 3 处**与本模块无关**的未提交改动（planner 侧两个 .cpp + 一个未跟踪的 opt.md），
   校验器要求声明与实际一致。
4. **更新 `docs/nav/map/待确认清单.md` 第 0 条**：由「已解决」改为「本轮已重新基线」，
   保留两次基线变更的完整经过。
5. **收尾校验**：`verify_docs.py --module map` → `errors=0`、`warnings=9`（全部为人工审核闸门）、覆盖率 22/22；
   `docs/tools/test_injection.py --root .` → **PASS**（其前置条件「map 原文档 errors=0」恢复成立）。

至此 **5 个模块**（map / path_planning / traj_optimize / controller / replan）的
**45 篇文档共用同一份证据摘要 `7c453c4e8af72579`**。
