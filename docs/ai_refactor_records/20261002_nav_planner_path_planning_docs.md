# path_planning 模块文档（步骤 4 之一）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/planner/path_planning/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**

---

## User Intent

用户原始目标与关键约束：

1. 「参考 `nav/example/` 这个例子，完成 `nav/planner/path_planning/` 的文档」。
2. `nav/example/README.md` 是**强制契约**：输入只能是 Evidence Pack，
   输出是固定 9 个文件 + front-matter，每句事实必须带 `文件:行号` 锚点，
   不确定项必须显式降级并进入 `待确认清单.md`。
3. 金样例为 `docs/nav/map/`（单一 owner），风格与粒度必须对齐。
4. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不擅自改代码/参数、
   禁止未授权构建、禁止任何 git 提交类操作、禁止使用 writingplans。

---

## Scope

- `docs/nav/planner/path_planning/`：新增 9 篇文档，删除 1 个 0 字节占位文件（用户确认）
- `docs/_evidence/path_planning/inventory.json`：新增证据（脚本产物）
- `docs/_evidence/verify.path_planning.json`：校验报告（脚本产物）
- `docs/ai_refactor_records/`：本记录

## Out of Scope

1. **不改任何被文档化的代码**：`src/`、`config/`、`launch/`、任何 `CMakeLists.txt`、参数默认值。
2. 不写 `nav/planner/{controller,replan,traj_optimize}`、`nav/ros2`、`nav/utils`、`nav/map` 的文档。
3. 不重新生成 `docs/_evidence/repo/{ros2,params,launch}.json`（理由见「Notes」1）。
4. 不修复 `docs/nav/map/` 文档的过期状态（理由见「未解决问题」1）。
5. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读）：

- `src/planner/path_planning/CMakeLists.txt`
- `src/planner/path_planning/include/path_planning/path_planning.hpp`（39 行）
- `src/planner/path_planning/src/path_planning.cpp`（101 行）
- `src/planner/path_planning/include/path_planning/post_processing.h`（165 行）
- `src/planner/path_planning/src/post_processing.cpp`（465 行）
- `src/planner/path_planning/include/path_planning/search/astar.h`（85 行）
- `src/planner/path_planning/src/search/astar.cpp`（119 行）
- `src/planner/path_planning/include/path_planning/search/jps.h`（113 行）
- `src/planner/path_planning/src/search/jps.cpp`（399 行）

调用方与跨模块（只读，用于判定边界与数据流）：

- `src/planner/replan/include/replan/fsm_replanner.h`、`src/planner/replan/src/fsm_replanner.cpp`
- `src/ros2/include/ros2/config.hpp`（参数装载）、`src/ros2/src/ros2_node.cpp`（参数传入与可视化）
- `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h`（下游消费点）
- `src/map/include/map/grid_map.hpp`（`is_tunnel`、`getDistance`、`posToIndex` 等）
- `src/utils/include/utils/logger.hpp`、`src/utils/include/utils/scope_timer.hpp`
- `config/planner.yaml`、`CMakeLists.txt`、`src/*/CMakeLists.txt`

证据与规范：

- `docs/nav/example/README.md`、`docs/nav/example/模板/*.md`、`docs/nav/map/*.md`（金样例）
- `docs/tools/verify_docs.py`（496 行，逐条读其判据）
- `docs/_evidence/repo/{ros2,params,launch}.json`、`docs/_evidence/map/inventory.json`
- `docs/ai_refactor_records/20261002_module_doc_anti_hallucination_pipeline.md`（上一轮记录）

### Active logic path

```
FsmReplan::plan / one_plan
  → path_planning.set_map(*grid_map)                 （每次规划前，值拷贝）
  → path_planning.set_velocity(current_vel, 0)        （标量 + 向量）
  → path_planning.set_use_jps(true)                   （当前链路固定 JPS）
  → PathPlanning::path_planning(start, goal, yaw0, yaw1, 5000)
      → PathPlanningImpl::path_planning
          1) use_jps_ ? JPS::jps_search : AStar::original_astar_search   → raw_path
          2) PathPostProcessing::optimize_path                            → optimized_path
          3) sample_path_states                                           → path_states
          4) assign_trajectory_timing                                     → timed_trajectory / total_time
          5) fill_additional_trajectory_info  + 有效性闸门                → Trajectory / nullopt
  → ma_spline_opt::from_path_planning_trajectory(traj)   （下游 MINCO 优化）
```

### Data flow

- **参数**：`config/planner.yaml:23` 的 `planner_config.path_planning_params` →
  `src/ros2/include/ros2/config.hpp:227-250` 装载 → `ros2_node.cpp:36` →
  `FsmReplan::set_param`（`fsm_replanner.h:45`）→ `PathPlanning::set_param`
  → A*/JPS 取安全阈值 + 后处理取整包参数。
- **地图**：planner 侧只有 `grid_map::GridMap`（2D + ESDF 距离场 + 隧道语义）。
- **ROS 接口**：本模块**零** ROS 接口（无 rclcpp include、无 pub/sub/timer）。
- **输出消费者**：traj_optimize 只读起终点位姿、`timed_trajectory`、`total_time`；
  replan 另读 `raw_path` / `optimized_path` 做二次碰撞检查与可视化。

### Risk notes

1. `PathPostProcessingParams` 有 8 个字段**声明处无初始化器**
   （`safe_threshold`、`max_vel`、`max_acc`、`time_resolution`、`min_traj_num`、
   `traj_cut_length`、`distance_weight`、`yaw_weight`），
   而装载函数用 `if (node["..."])` 守卫（`config.hpp:236-243`）→ 删键即读不确定值。
2. `AStar::safe_threshold_` 同样无初值（`astar.h:71`），而 `JPS` 有 `= 0.0`（`jps.h:50`），
   两个搜索器不对称。
3. A* 与 JPS 的**起点容错策略不同**：A* 只查离散点，JPS 要求起点附近 1.5 m 内有
   可通行栅格并会吸附（`jps.cpp:221-231`）。
4. `distance_weight` / `yaw_weight` 由 YAML 装载进结构体，但**本模块内无任何使用点**；
   `evaluate_duration` / `evaluate_length` 有定义无调用；
   `if_cut` / `UnOccupied_positions` / `UnOccupied_initT` / `weighted_length` 只写不读。
5. `fill_additional_trajectory_info` 先用 `timed_trajectory.size()` 作除数
   （`post_processing.cpp:331`），而空轨迹判定在其后（`path_planning.cpp:58-61`）。
6. 折叠降速完全依赖地图隧道语义（`post_processing.cpp:438`），语义缺失时**静默失效**。
7. `params.json` 生成时只扫了 `src/map/include` 与 `src/ros2/include`，
   **planner 侧代码绑定不在证据里**；本模块 16 条参数在证据中登记为
   `context_unknown: true` 的子节点读取（后缀候选与本文写法一一对应）。
8. `src/` 下存在本轮之前的未提交改动（controller 的 `mpc.cpp`、
   traj_optimize 的 `traj_optimizer.cpp`，以及未跟踪的 `opt.md`），
   因此本文档描述的是**工作区**状态。

### Recommended modification boundary

只新增 `docs/` 下文件（+ 一个模块证据 JSON）；所有事实来自证据与源码行号；
两个空/占位文件（`interface.md`）需要用户裁决后才能处置；不改任何 `src/`。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/planner/path_planning/README.md` | 新增（背景、模块概述、边界、入口与调用关系、源文件清单） |
| `docs/nav/planner/path_planning/模块设计说明.md` | 新增（目标约束、类协作、数据流、关键决策、线程、已知限制） |
| `docs/nav/planner/path_planning/接口文档.md` | 新增（文件/类/方法/成员/数据结构表 + Doxygen 规范指回模板） |
| `docs/nav/planner/path_planning/依赖关系.md` | 新增（CMake、外部库、模块内 include 图、双向跨模块依赖、依赖风险） |
| `docs/nav/planner/path_planning/流程图.md` | 新增（主数据流、时序图、时间分配子流程、隧道区间状态机） |
| `docs/nav/planner/path_planning/配置说明.md` | 新增（读取机制、16 条参数表、默认值不一致项、缺口、可移植性） |
| `docs/nav/planner/path_planning/测试要点.md` | 新增（10 条风险点、9 条边界场景、可观测量、无法验证部分，全部【推断】） |
| `docs/nav/planner/path_planning/常见问题FAQ.md` | 新增（Q1–Q6 + 参数类问题 + 排查手段） |
| `docs/nav/planner/path_planning/待确认清单.md` | 新增（参数缺口、16 条上下文未知读取、6 条语义待确认、8 条其他） |
| `docs/nav/planner/path_planning/interface.md` | **删除**：0 字节占位（git 跟踪），与「9 文件固定名」契约冲突；已获用户明确确认 |
| `docs/_evidence/path_planning/inventory.json` | 新增（工具产物：8 文件 / 192 符号 / 0 解析失败） |
| `docs/_evidence/verify.path_planning.json` | 新增（校验报告） |

### Key changes

1. **证据先行**：先用既有 C++ 工具（`docs/tools/cpp/inventory/build/ma_nav_inventory`，
   已编译产物，未触发新构建）对本模块 8 个源文件提取符号表，文档中的每个符号名都来自该表。
2. **每个 H2 小节都有锚点或显式降级**：空公式化小节会被 `annotation.unsupported_section`
   拦下，因此「测试前置条件」「附：Doxygen 规范」等小节都补了锚点或 `【待确认】`。
3. **跨模块符号不带反引号限定名**：`grid_map::GridMap`、`utils::TimeConsuming`、
   `logger::planning` 等不在本模块 inventory 内，按 `example/README.md` §9.1
   改用「文件锚点 + 裸类名」表述，避免 `symbol.unknown` 误报。
4. **把「没验证过」写成结论**：`测试要点.md` 全部标注【推断】，
   明确写出「本仓库 `src/` 下无测试」，不出现任何「已验证/已测试」类断言。
5. **缺口如实登记**：`待确认清单.md` 给出 16 条上下文未知读取（与
   `params.json` 的 `subnode_reads` 一一对应）、28 条未被引用键的归属、
   以及本模块参数绑定未进证据的扫描范围缺口。
6. **机械一致性**：front-matter 的 `git_rev=2eed7d3e884d`、
   `evidence_digest=e92ac2dc4614a515`、`worktree_dirty=true` 与当前工作区交叉核对一致。

### Behavior preserved

- 被文档化代码**零改动**：`src/`、`config/`、`launch/`、所有 `CMakeLists.txt` 未修改
  （`git status` 可证；`src/` 下的 3 个条目为本轮之前既有）。
- 未修改 `docs/tools/*`、`docs/nav/example/*`、`docs/nav/map/*`、`docs/nav/AGENTS.md`。
- 未重新生成 `docs/_evidence/repo/*.json`，因此 map 模块的证据内容与缺口计数
  （4 / 28 / 70）保持不变。

### Behavior intentionally adjusted

1. **删除 `docs/nav/planner/path_planning/interface.md`**：0 字节、git 跟踪的占位文件。
   保留它会让校验器永远报 `structure.front_matter`（空文件无 front-matter），
   也无法满足「模块 9 个文件、接口文档名为 `接口文档.md`」的契约。
   删除前已用 `ask_user_question` 取得用户明确选择（选项之一即「删除该 0 字节占位文件」），
   文件可从 git 历史恢复。
2. **本模块的「配置说明」以「模块不读 YAML」为核心结论**：
   与 map 模块（自己用 `LoadParam` 读键）形成对照，参数链写成
   「ros2 装载 → replan 注入 → 本模块使用」，避免读者按 map 的机制去找键。
3. **`接口文档.md` 的 Doxygen 附录不复制原文，改为指回模板 + 【待确认】**：
   与金样例 `docs/nav/map/接口文档.md` 的处理方式一致（原文保存于
   `docs/nav/example/模板/接口文档.md`，避免双份漂移）。

### Notes

1. **为什么不重新生成 `repo/params.json`**：该文件是仓库级共享证据，
   其内容参与**全局** `evidence_digest`，且 `docs/nav/map/待确认清单.md`
   逐条引用了其中的计数（4 / 28 / 70）。若以 `--module path_planning --scan src/planner`
   重跑，会同时改变 map 文档的缺口口径，属越界。本模块参数的读取链因此
   以「ros2 装载函数 + 调用点」的锚点表述，并在 `待确认清单.md` 记录该证据缺口。
2. **新增模块证据的副作用**：`docs/_evidence/path_planning/inventory.json` 会被计入
   `evidence_common.evidence_digest()`（该函数遍历证据目录下所有 `*.json`），
   因此全局摘要由 `1ab15719a7937f37` 变为 `e92ac2dc4614a515`；
   path_planning 的 9 篇文档统一使用新值。
3. **未执行任何构建**：库存工具使用的是上一轮已编译好的二进制
   （`docs/tools/cpp/inventory/build/ma_nav_inventory`），本轮没有触发编译/链接。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：`PathPlanning → Impl → {AStar|JPS} → PathPostProcessing` 五步流水线
      与 6 个输出字段逐条对着源码行核过
- [x] diff 检查：`git status --porcelain -- docs/` 核对，改动全部落在
      `docs/nav/planner/path_planning/`、`docs/_evidence/`、本记录
- [x] grep 检查：`rclcpp|yaml|LoadParam` 确认本模块零 ROS 接口、零 YAML 解析；
      16 个参数逐个在本模块内 grep 使用点，确认 4 个无使用点
- [x] XML / launch / yaml 检查：参数真值取自 `params.json` 的 YAML 解析值（带行号），
      未使用 `config/planner.yaml` 的注释文本
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module path_planning` → **errors=0**，coverage 8/8，
      warnings=9（全部为设计上的人工审核闸门 `review.pending`）
- [x] 注入测试：对 path_planning 副本注入假话题 `/cmd_vel`、越界行号
      `jps.cpp:9999`、编造符号 `path_planning::JPS::nonexistentMethod`，
      三类**全部**被识破（`ros2.unknown_topic` / `anchor.line_out_of_range` /
      `symbol.unknown`），注入副本 `status=FAIL`
- [x] 独立审核 Agent（不同上下文）：按 `example/README.md` §4.1 要求另行执行，
      结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**任何 `git add/commit/push/tag/merge`，
      只用了只读的 `git status` / `git rev-parse`

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | `interface.md`（0 字节，git 跟踪）导致 `structure.front_matter`，且违反 9 文件契约 | 询问用户后按用户选择删除；记入「有意调整」 |
| 2 | 首轮 16 个 error：8 个「带 `::` 的符号 + 多锚点」行、4 处把 `planner_config.path_planning_params` 这类**中间节点**加反引号当作 YAML 键、1 处 `//` 被当成话题、`FsmReplan::set_param` 等跨模块限定名 | 全部按「跨模块符号不带限定名 + 一个锚点配一个符号 + 中间节点不加反引号」逐条修正，未放宽校验器规则 |
| 3 | `repo/params.json` 未覆盖 `src/planner`，本模块参数绑定不在证据内 | 不重跑（越界且有副作用），改在 `配置说明.md` 与 `待确认清单.md` 显式登记缺口 |
| 4 | 证据摘要因新增模块证据而全局变化，`docs/nav/map/` 9 篇文档记录值过期 | 不修改 map 文档（越界），记录为待办（见「未解决问题」1） |
| 5 | `src/` 下存在本轮之前的未提交改动，map 文档的 `worktree_dirty: false` 与实际不符 | 仅如实声明本模块 `worktree_dirty: true`；map 侧不在本轮范围 |
| 6 | **独立审核**：`模块设计说明.md` 与 `常见问题FAQ.md` 把「优化失败保留原始轨迹」写成【事实】 | 生效路径在优化失败时直接报错、调用方丢弃结果（`src/planner/replan/src/fsm_replanner.cpp:101-104`、`src/ros2/src/ros2_node.cpp:273-279`），符合注释的那份实现无调用点。已降级为【待确认】并写清两侧证据；FAQ Q6 改写为「检查上一轮轨迹」（`src/planner/replan/src/fsm_replanner.cpp:41-42`） |
| 7 | **独立审核**：`接口文档.md` 与 `依赖关系.md` 漏写下游读取的 `start_state` / `final_state`（起终速度与 yaw 角速度） | 补齐 Trajectory 表并给出 `src/planner/traj_optimize/include/traj_optimize/ma_spline_opt/optimizer_config.h:142-148` 锚点 |
| 8 | **独立审核**：Trajectory 表漏列 6 个字段，两处「见清单」是悬空引用；`evaluate_duration` / `evaluate_length` 为无调用点函数却未登记 | 表补全为 15 行并标明「只写不读」；`待确认清单.md` 第三节新增第 8 条 |
| 9 | **独立审核**：点级「图外无碰撞」与 JPS 栅格级「越界即占用」被混写；JPS 代际复位被绝对化 | 限定为「点级校验」并补栅格级语义（`模块设计说明.md` 限制 3、`测试要点.md` R5）；R3 限定为「`set_map` 内不得重置」；新增 R11 |
| 10 | **独立审核**：`distance_weight` / `yaw_weight` 全仓库零消费者（可判定结论）被写成开放问题 | 改判为【事实】死参数，YAML 注释过期（`config/planner.yaml:32-33`）一并登记 |

### 独立审核 Agent（不同上下文）

按 `docs/nav/example/README.md` §4.1 的要求，用 `subagent_fork` 另起一个**不同上下文**的
审核 Agent，只拿 Evidence Pack + 9 篇文档 + 源码，任务是「找出无法被证据支撑的句子」。
它**没有修改任何文件**。

复核通过的项（它自己重新取证）：

- 本模块不使用 rclcpp、不解析 YAML（include 列表与 `grep` 双证）；
- planner/controller 只链接不 include，且经 traj_optimize 头链间接取得类型；
- 16 条参数的 YAML 值 / 代码默认值 / 读取行号与三处源码逐条一致；
- `distance_weight` / `yaw_weight` / `unfold_time` / `fold_margin` / `evaluate_duration` /
  `evaluate_length` 的「无使用点」结论成立（`unfold_time` / `fold_margin` 确由
  `src/ros2/src/ros2_node.cpp:315-321` 的云台折叠事件读取）；
- A*/JPS 关键行为（点级图外宽容、1.5 m 吸附、代际不得在 `set_map` 重置、超时返回空）；
- 入口链锚点全对（全仓库仅 replan 持有门面）。

审计结论：首轮 **NEEDS_FIX** —— 符号/行号/参数值层全对，但查出 9 条
**语义级**缺陷（兜底行为被当事实、下游消费字段漏写、清单悬空引用与死字段遗漏、
越界语义跨文档冲突、动机归因错误、结论绝对化）。这些正是
`verify_docs.py` 明确声明「抓不到」的部分
（`docs/nav/example/README.md` §4.1）。
上表第 6–10 条即为其发现，已逐条定向修复并重新校验。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 8 个源文件全部被文档提及，缺失 0
- 注入测试：三类幻觉在 path_planning 副本上全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 9 条语义级发现已**逐条修复并重新校验**，
  修后仍 `errors=0`
- 范围合规：`src/`、`config/`、`launch/`、`CMakeLists.txt` 零改动；
  未执行任何构建；未执行任何 git 提交类操作

---

## 未解决问题（留给后续）

1. **`docs/nav/map/` 文档已过期**：全局 `evidence_digest` 因新增模块证据而改变
   （`1ab15719a7937f37` → `e92ac2dc4614a515`），9 篇文档均报 `evidence.stale`；
   同时 `src/` 出现新的未提交改动，其 `worktree_dirty: false` 与实际不符，
   9 篇各报 `evidence.worktree_mismatch`（这是本轮之前就存在的状态）。
   修复方式是「重新基线」：更新 9 篇的 `evidence_digest` 与 `worktree_dirty`，
   或在提交后再重跑提取器。本轮未处理。
2. **人工审核闸门**：本模块 9 篇仍为 `status: draft` / `reviewer: 未审核`，
   需按 `docs/nav/AGENTS.md` 逐篇确认后改为 `reviewed`。
3. **`subnode_reads` 跨函数调用点推导未实现**：本模块 16 条参数读取的绝对键路径
   目前靠人工按调用点确认（候选与证据一致）。
4. **`docs/tools/test_injection.py` 的默认前置条件（map 原文档 errors=0）当前不成立**，
   脚本整体报 FAIL；其三类注入检查本身仍然全部命中。
5. 本模块自身的事务性缺口（8 个参数字段无初值、`AStar::safe_threshold_` 无初值、
   JPS 1.5 m 吸附常量、4 个字段/2 个函数无使用点）已在
   `待确认清单.md` 与 `测试要点.md` 登记，**未改代码**。

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
