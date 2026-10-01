# replan 模块文档（步骤 4 之四）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/planner/replan/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**
顺序：本轮为「按顺序完成 planner 三个模块」的第 3 个（traj_optimize → controller → **replan**），
也是用户指定顺序中的最后一个

---

## User Intent

1. 用户要求：**按顺序**完成 `nav/planner/{traj_optimize,controller,replan}` 的文档，不要并行。
2. 契约沿用 `nav/example/README.md`：只基于 Evidence Pack、9 个固定文件、锚点、
   不确定项显式降级。
3. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不改代码/参数、禁止未授权构建、
   禁止任何 git 提交类操作。

---

## Scope

- `docs/nav/planner/replan/`：新增 9 篇文档
- `docs/_evidence/replan/inventory.json`：新增证据（2 文件 / 66 符号 / 0 失败）
- `docs/_evidence/verify.replan.json`：校验报告
- `docs/ai_refactor_records/`：本记录
- 收尾：本轮三个 + 之前两个模块文档的 `evidence_digest` 统一重基线（见「未解决问题」1）

## Out of Scope

1. 不改任何被文档化的代码（`src/`、`config/`、`launch/`、`CMakeLists.txt`、参数默认值）。
2. 不清理死代码（`one_plan`、`check_point_equal`、投影结构、未使用成员）。
3. 不重新生成 `docs/_evidence/repo/*.json`。
4. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读）：

- `src/planner/replan/include/replan/fsm_replanner.h`（141 行）
- `src/planner/replan/src/fsm_replanner.cpp`（292 行）
- `src/planner/replan/CMakeLists.txt`（19 行）

调用方与跨模块（只读）：

- `src/ros2/include/ros2/ros2_node.h`、`src/ros2/src/ros2_node.cpp`、`src/ros2/include/ros2/config.hpp`
- `src/utils/include/utils/type_utils.hpp`（`RobotState` 有无初始化器）
- `config/planner.yaml`、顶层 `CMakeLists.txt`

### Active logic path

```
ros2: fsm_replanner.set_param(config.planner_config)（ros2_node.cpp:36）
每帧: fsm_replanner.plan(goal, current, map)（ros2_node.cpp:272）
  → 到达判定早退（fsm_replanner.cpp:21-25）
  → 三类触发判定：目标变化 / 旧轨迹碰撞 / 横向偏差（fsm_replanner.cpp:35-56）
  → 需要重规划：
       get_safe_pos（起终点外推）→ set_use_jps(true) → set_velocity → path_planning(...,5000)
       → GridMapESDF 适配器 → set_esdf_interface → from_path_planning_trajectory
       → model = OMNI_XY → ma_opt_.optimize
  → 成功：保存两条轨迹 + 新轨迹标志 + 成功状态（fsm_replanner.cpp:105-110）
  → 失败：返回 PLANNING_FAILED / MINCO_OPT_FIALED（:79 / :103）
```

### Data flow

- **参数**：`config/planner.yaml:93-103` → `load_replan_param`（`config.hpp:211-222`）
  → `PlannerConfig.replan_params` → `set_param` 分发到两个子模块。
- **地图**：`grid_map::GridMap` 同时喂给搜索（`set_map`）与优化（`GridMapESDF` 适配器）。
- **输出**：`ResultPath`（搜索轨迹 + 优化轨迹 + 状态 + 是否新轨迹），
  由 ros2 侧消费；新轨迹才下发给 controller。
- **ROS 接口**：本模块零 ROS 接口。

### Risk notes

1. **注释与实现冲突**：注释承诺「优化失败保持原始轨迹」，实现直接返回错误
   （`fsm_replanner.cpp:83` vs `:101-104`），调用方丢弃且不发布（`ros2_node.cpp:273-279`）。
2. **参数无初始化器**：`ReplanParam` 6 个字段、`old_goal_pose_` 都没有默认值，
   装载又是「存在才赋值」→ 缺键或首帧会读到不确定值（其中 3 个参数被 `plan()` 使用）。
3. **两个构造入口不一致**：`set_param` 分发优化配置，带参构造不分发（`h:42-46` vs `h:49-52`）。
4. **三个热启动参数只装载不使用**（有效期 / 连续性阈值 / 投影分辨率），
   另一个键（最小重规划间隔）连读取点都没有。
5. **死代码规模可观**：未调用的方法 2 个、未使用的结构 1 个（含 6 字段）、
   未使用的枚举 1 个（含 2 取值）、未使用的成员 2 个、从未返回的错误码 1 个。
6. **ESDF 裸指针有顺序依赖**：适配器是局部对象，优化器只存指针。
7. **硬净空比规划阈值松约 5 倍**（安全距离 × 0.2，下限 0.05）。
8. `src/` 下仍有本轮之前的未提交改动 → `worktree_dirty: true`。

### Recommended modification boundary

只新增 `docs/` 下文件与一个模块证据 JSON；死代码只登记不删除；
不改触发逻辑与阈值系数。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/planner/replan/README.md` | 新增：背景、编排层定位、三类触发、边界、入口、3 文件清单 |
| `docs/nav/planner/replan/模块设计说明.md` | 新增：目标约束、类协作、数据流、关键决策、线程、10 条已知限制 |
| `docs/nav/planner/replan/接口文档.md` | 新增：**全部 66 条符号**按结构分组并注语义 |
| `docs/nav/planner/replan/依赖关系.md` | 新增：CMake、外部库、内部与双向跨模块依赖、6 条风险 |
| `docs/nav/planner/replan/流程图.md` | 新增：主数据流、触发判定细化、重规划时序、失败分支 |
| `docs/nav/planner/replan/配置说明.md` | 新增：机制 + 6 行参数表 + 6 行默认值差异 + 缺口 + 未引用键 |
| `docs/nav/planner/replan/测试要点.md` | 新增：13 条风险点、7 条边界、可观测量、无法验证部分（全【推断】） |
| `docs/nav/planner/replan/常见问题FAQ.md` | 新增：Q1–Q9 + 参数类问题 + 排查手段 |
| `docs/nav/planner/replan/待确认清单.md` | 新增：4 条参数缺口、9 条上下文未知读取、13 条语义待确认、5 条其他 |
| `docs/_evidence/replan/inventory.json` | 新增（工具产物） |
| `docs/_evidence/verify.replan.json` | 新增（校验报告） |

### Key changes

1. **把「注释承诺」与「代码事实」分开写**：优化失败的兜底承诺单列一条风险，
   并给出注释行、失败分支行、调用方处理行三处证据。
2. **参数表同时给出「被读取」与「装载但未使用」两组**，
   并把「最小重规划间隔」单列为完全未被引用的键。
3. **死代码显式登记**：未调用方法、未使用结构/枚举/成员、从未返回的错误码
   都在接口文档里标注「无调用点/无使用」，避免后来者误以为它们在工作。
4. **构造入口不一致、ESDF 顺序依赖、硬净空口径**三条最容易踩的坑都写成风险项。
5. 全部 66 条符号都有人工语义。

### Behavior preserved

- 被文档化代码零改动；未修改 `docs/tools/*`、`example/*`、`map/*`、
  path_planning / traj_optimize / controller 的文档。
- 未删除任何死代码。

### Behavior intentionally adjusted

1. 配置说明把「3 个参数装载但未使用」写成【事实】而不是【待确认】，
   因为「无读取点」是可检索验证的结论；是否实现热启动才是待确认项。
2. 常见问题 FAQ 的 Q7 直接说明未调用实现里的日志时序问题，
   避免将来启用时被误导。

### Notes

- 库存工具沿用已编译好的二进制（未触发构建）。
- 本模块证据加入后，全局 `evidence_digest` 变为 `7c453c4e8af72579`（收尾重基线用值见下）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：早退 → 三条件触发 → 外推 → 搜索 → 优化 → 成功收尾逐行核对
- [x] diff 检查：改动全部落在 `docs/nav/planner/replan/`、`docs/_evidence/`、本记录
- [x] grep 检查：`rclcpp|yaml` 零命中；`one_plan|check_point_equal|TrajectoryProjectionResult|
      MincoError|has_valid_trajectory_|last_trajectory_time_|MAX_RETRIES` 均无使用点；
      三个热启动参数在 `src/planner/replan` 内无读取点
- [x] XML / launch / yaml 检查：参数真值取自 `params.json` 解析值（6 个被读取键）
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module replan` → **errors=0**，coverage **2/2**，warnings=9
- [x] 注入测试：假话题 `/cmd_vel`、越界行号 `fsm_replanner.cpp:99999`、
      编造符号 `replan::FsmReplan::nonexistentMethod` 三类全部被识破
- [x] 独立审核 Agent（不同上下文）：结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 首轮 4 个 error：`tl::expected` 与 `//` 被反引号包住（前者非本模块符号、后者被当成话题）；`replan_param` 字段锚点用了结构声明行；`MAX_RETRIES` 落在错误锚点窗口外 | 逐条去掉反引号或改用正确行号；未放宽校验器 |
| 2 | 依赖关系最初按经验写 CMake 行号 | 用 `cat -n` 逐行核对后确认 3/5/7/9/11/13 与链接项 14-18 正确 |
| 3 | **独立审核**：FAQ 写成「没有调用**已装载的**最小重规划间隔参数」，但该键从未被装载（`config.hpp:211-222` 无此键、`ReplanParam` 无此字段） | 改为「既没有装载、也没有字段与读取点」 |
| 4 | **独立审核**：「类型成员也没有初始化器」过度概括（该类型有 2 个成员是有初始化的） | 精确列出未初始化的成员（位置/速度/加速度/加加速度、yaw、时间戳、四元数） |
| 5 | **独立审核**：`planner_config_` 锚到结构声明行而非成员定义行 | 锚点改为 `fsm_replanner.h:39` |
| 6 | **独立审核**：「触发条件是或关系」锚点过窄（只覆盖碰撞分支） | 锚点扩为 `fsm_replanner.cpp:35-56` |
| 7 | **独立审核**：「优化器只保存裸指针」是跨模块断言但缺上游锚点 | 三处补 `traj_optimizer.h:29-31` |
| 8 | **独立审核**：「唯一使用者是 ros2」的证据误用了本模块的链接方向 | 改引三处 include 锚点（`ros2_node.h:13`、`ros2/include/ros2/config.hpp:3`、`ros2_node.cpp:5`） |
| 9 | **独立审核**：「yaw 不参与优化」措辞易误读且锚点不全 | 改为「yaw 不作为优化变量（沿用参考航向）」并补模型枚举锚点 |
| 10 | **独立审核**：日志通道说法不准（触发条件其实不打日志；耗时走另一个通道） | 拆分说明并补计时通道锚点（`utils/logger.hpp:125`） |
| 11 | **独立审核**：`set_param`「一次分发三组参数」表述含糊 | 改为「保存三组、下发其中两组」 |

### 独立审核 Agent（不同上下文）

只读审核 Agent 逐条重取证，**13 条高风险断言在符号/行号/参数值层面全部为真**：

- 无 rclcpp / 无 YAML；
- 三类触发条件、早退与标志清除时机；
- `get_safe_pos` 的返回条件、上限（200 次 / 1.5 m）与阈值来源；
- 碰撞复检的两个阈值口径、双路径检查与逐段采样；
- `lateral_deviation` 算法与短路径返回值；
- 「注释承诺兜底但实现直接报错」；
- 死代码清单（含未调用实现里的日志时序问题）；
- 两个构造入口不一致；
- 参数与旧目标成员缺初始化器；
- 配置说明 6 个被读取键 / 3 个装载未使用 / 1 个完全未读取；
- CMake 全部行号与「链接 controller 但零引用」。

审计结论：首轮 **NEEDS_FIX** —— 1 处事实错误 + 1 处过度概括 + 4 处锚点/证据问题
+ 3 处表述不准，已逐条修复，并把它建议的 3 条（模型写死 2D、触发无日志、
同名旧目标成员陷阱）补进待确认清单；修后校验器仍 `errors=0`。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 2 个源文件全部被文档提及，缺失 0
- 注入测试：假话题、越界行号、编造符号三类全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 11 条发现已逐条修复，修后 `errors=0`
- 范围合规：`src/`、`config/`、`launch/`、`CMakeLists.txt` 零改动；未执行构建；未执行提交类操作

---

## 未解决问题（留给后续）

1. **收尾重基线**：本轮新增 4 个模块证据（path_planning / traj_optimize / controller / replan）
   使全局 `evidence_digest` 依次变化，早先的模块文档记录值会过期。
   最终值 `7c453c4e8af72579` 已用于 replan；
   path_planning 与 traj_optimize 的 9 篇各自需要更新该字段（**本记录所在的收尾步骤一并处理**）。
2. `docs/nav/map/` 的摘要与 `worktree_dirty` 问题**已在同日的「map 模块单独重新基线」中解决**。
3. 人工审核闸门未过（本模块 9 篇 `status: draft`）。
4. 代码侧待办（未改代码，仅登记）：注释兜底未实现、参数无初始化器、
   构造入口不一致、三个热启动参数未接线、死代码清理、ESDF 顺序依赖、
   硬净空口径、失败无退避、20 s 超时策略在调用方。

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
