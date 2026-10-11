# ros2 模块文档（步骤 5 之一）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/ros2/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**
顺序：本轮为「按顺序完成 ros2 → utils」的第 1 个

---

## User Intent

1. 用户要求：**按顺序**完成 `nav/ros2/` 与 `nav/utils/` 的文档，不要并行。
2. 契约沿用 `nav/example/README.md`：只基于 Evidence Pack、9 个固定文件、锚点、
   不确定项显式降级。
3. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不改代码/参数、禁止未授权构建、
   禁止任何 git 提交类操作。

---

## Scope

- `docs/nav/ros2/`：新增 9 篇文档
- `docs/_evidence/ros2/inventory.json`：新增证据（5 文件 / 189 符号 / 0 失败）
- `docs/_evidence/verify.ros2.json`：校验报告
- 顺带：因用户在本轮期间提交了 `f150fbcca0c7`（工作区变干净），
  把已有 5 个模块（map + planner 四个）的 `git_rev` / `worktree_dirty` / `evidence_digest`
  重新基线（见「Notes」1）
- `docs/ai_refactor_records/`：本记录

## Out of Scope

1. 不改任何被文档化的代码（`src/`、`config/`、`launch/`、`CMakeLists.txt`、参数默认值）。
2. 不写 utils 的文档（按用户要求顺序排后）。
3. 不清理死成员/硬编码话题名/重复加载等已登记问题。
4. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读，5 文件 / 2952 行）：

- `src/ros2/include/ros2/color_msg_utils.hpp`（64）
- `src/ros2/include/ros2/ros2_node.h`（170）
- `src/ros2/include/ros2/config.hpp`（348）
- `src/ros2/src/ros2_node.cpp`（977）
- `src/ros2/include/ros2/misc/visualizer.hpp`（1396）

入口与构建（只读）：

- `src/ma_nav.cpp`（23 行，进程入口）
- `launch/run.launch.py`（49 行）
- `src/ros2/CMakeLists.txt`、顶层 `CMakeLists.txt`
- 证据：`docs/_evidence/repo/ros2.json`（34 接口 + 58 配置绑定）、`params.json`

### Active logic path

```
main（src/ma_nav.cpp:4）→ 声明/读取 planner_params_path（:9-11）→ GlobalPlanner2d（:13）
  → 构造（ros2_node.cpp:27）：Config(params_path) → MaMap(map_params_path) → mpc_ → 重规划器 set_param
  → 三个互斥回调组（:38-40）→ 4 订阅 + 28 发布 + 2 服务 + 3 定时器（:51-120）
  → 地图组：map_callback（:429）/odom_callback（:394）
  → 规划组（33 ms）：plan_omni（:248）→ FsmReplan::plan（:272）→ 可视化 + 折叠事件（:298-366）
  → 控制组（10 ms）：controller_callback（:130）→ Mpc::solve（:179）→ 折叠指令（:197-216）
     → 世界系→车体系（:222-226）→ ±20/NaN 保护（:228-240）→ /cmd_vel_chassis（:242）
```

### Data flow

- **参数**：ROS 参数只传文件路径，其余全部由 `Config` 从 YAML 读取
  （`src/ma_nav.cpp:9-11`、`src/ros2/include/ros2/config.hpp:316-345`）。
- **地图**：点云与里程计都写地图，因此被放进**同一个互斥回调组**避免并发写
  （`src/ros2/src/ros2_node.cpp:43-46`）。
- **规划结果**：新轨迹同时更新 MPC 目标、折叠事件与跟踪游标
  （`src/ros2/src/ros2_node.cpp:308-344`）。
- **折叠**：轨迹 + 地图隧道语义 → 折叠/展开事件 → 按跟踪时间下发
  （`src/ros2/src/ros2_node.cpp:915-976`、`src/ros2/src/ros2_node.cpp:197-216`）。

### Risk notes

1. **锁的注释与用法不一致**：注释称保护地图，实际只在点选区域两处加锁
   （`ros2_node.h:68` vs `ros2_node.cpp:514`、`:793`）。
2. **20 s 计时的隐式前置条件**：`plan_omni` 第一处使用就无条件取可选值
   （`ros2_node.cpp:271`），赋值点只有目标回调（`:481`）与内部（`:275/287/297`）。
3. **构造期重复加载地图**（`ros2_node.cpp:122-128`）。
4. **硬编码**：首帧限速 ±3（`:166`）、发布保护 ±20（`:229-236`）、三个话题名（`:99-101`）。
5. **死成员/死常量**：`minco_trajectory_`、`t_now_`、`is_print`、`plotter_`、三个折叠常量
   （`ros2_node.h:109/118-121/142-143`）；`is_minco` 装载后无人使用（`config.hpp:331`）。
6. **配置字段无初始化器** + 存在才赋值 ⇒ 缺段时布尔量不确定（`config.hpp:296-311`、`:319-332`）。
7. 可视化器在**全局命名空间**且单头 1396 行（`visualizer.hpp:27`）。

### Recommended modification boundary

只新增 `docs/` 下文件；死代码、硬编码、重复加载只登记不修改；
不把 `Visualizer` 移入命名空间。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/ros2/README.md` | 新增：背景、四块构成、接口计数、边界、入口、5 文件清单 |
| `docs/nav/ros2/模块设计说明.md` | 新增：目标约束、类协作、数据流、关键决策、线程与并发、10 条已知限制 |
| `docs/nav/ros2/接口文档.md` | 新增：**34 个 ROS 接口全表**（订阅/发布/服务/参数）+ 5 个类的接口表 |
| `docs/nav/ros2/依赖关系.md` | 新增：ament 构建、ROS 依赖、内部与双向跨模块依赖、6 条风险 |
| `docs/nav/ros2/流程图.md` | 新增：系统数据流、回调组分工、规划状态机、控制时序、折叠链路 |
| `docs/nav/ros2/配置说明.md` | 新增：读取机制 + 12 键顶层表 + 其它段 + 缺口 + 4 处绝对路径 |
| `docs/nav/ros2/测试要点.md` | 新增：14 条风险点、11 条边界、可观测量、无法验证部分（全【推断】） |
| `docs/nav/ros2/常见问题FAQ.md` | 新增：Q1–Q9 + 参数类问题 + 排查手段 |
| `docs/nav/ros2/待确认清单.md` | 新增：4 条参数缺口、8 行代表样本、12 条语义待确认、6 条其他 |
| `docs/_evidence/ros2/inventory.json` | 新增（工具产物） |
| `docs/_evidence/verify.ros2.json` | 新增（校验报告） |

### Key changes

1. **接口面先于正文**：先用 `repo/ros2.json` 的 34 条接口建立「订阅/发布/服务/参数」
   总表，再回到代码补 QoS、回调组与行号，避免凭印象写话题名。
2. **把回调组与锁讲清楚**：明确「组内串行、组间并发」，并指出锁的注释与用法不一致，
   列为待确认而不是替作者下结论。
3. **折叠链路单独成图**：从隧道检测 → 区间合并 → 事件生成 → 排序去重 → 控制侧消费，
   每步给出锚点。
4. **硬编码与死代码显式登记**，不擅自清理。
5. **顺带修正跨模块结论**：发现 `ros2_node.h:143` 声明了 gcopter 的
   `Trajectory<5, 2>` 成员，因此**模板已被实例化**，据此修正了 traj_optimize 文档里
   「从未实例化」的表述（4 篇文档）。

### Behavior preserved

- 被文档化代码零改动；未修改 `docs/tools/*`、`example/*`、其它模块正文结论
  （除上述 traj_optimize 的实例化修正）。

### Behavior intentionally adjusted

1. 修正 traj_optimize 的 4 处表述（gcopter 模板实例化），因为它是**跨模块**事实错误，
   与本模块证据直接相关。
2. 顺带把 5 个既有模块的 front-matter 重新基线到新提交（见 Notes）。

### Notes

1. **期间发生的仓库状态变化**：用户把 planner 阶段的改动与文档提交为
   `f150fbcca0c7`，`src/`、`config/`、`launch/` 变干净。因此：
   - 5 个既有模块的 `git_rev` 更新为 `f150fbcca0c7`、`worktree_dirty` 改为 `false`；
   - 5 个待确认清单的「工作区不干净」条目改写为「已解决/已重新基线」；
   - 新增 ros2 证据后全局摘要变化，5 个模块的 `evidence_digest` 先更新为当时的
     `937d4a38ea08d42c`（utils 完成后还会再统一一次）。
2. 库存工具沿用已编译好的二进制（未触发构建）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：入口 → 构造 → 三组回调 → 规划/控制/可视化节拍逐行核对
- [x] diff 检查：改动全部落在 `docs/` 下
- [x] grep 检查：`rclcpp` 全仓库唯一依赖；死成员逐条 grep；`is_minco` 仅两处；
  `plan_start_time_` 赋值点；`map_mutex_` 两处使用
- [x] XML / launch / yaml 检查：launch 用 ast 解析结果（1 节点 / 1 声明参数）核对；
  参数真值取自 `params.json`
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module ros2` → **errors=0**，coverage **5/5**，warnings=9
- [x] 注入测试：假话题 `/cmd_vel`、越界行号 `ros2_node.cpp:99999`、
      编造符号 `planner::GlobalPlanner2d::nonexistentMethod` 三类全部被识破
- [x] 独立审核 Agent（不同上下文）：结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 首轮 3 个 error：`logger::ros2` 被反引号包住（非本模块符号）、`top` 被当作需支撑的符号、中间节点 `planner_config.replan_params` 出现在正文反引号里 | 逐条改写措辞；未放宽校验器 |
| 2 | 初稿把接口计数写成「24 发布 / 6 订阅 / 2 服务」 | 按 `repo/ros2.json` 与源码重数为 **28 发布（24+4）/ 4 订阅 / 2 服务**，并写明划分 |
| 3 | 「color_msg_utils.hpp 不被任何文件 include」写错 | 核实后被可视化头 include（`visualizer.hpp:9`），改为「只被可视化头 include」 |
| 4 | 依赖关系中 ROS 链接段的锚点写成 `:25-37`（25 行是空行） | 改为 `:26-37` |
| 5 | 交叉发现：traj_optimize 文档声称 gcopter 模板「从未实例化」 | 本模块证据显示 `ros2_node.h:143` 有 `Trajectory<5, 2>` 成员 ⇒ 已实例化；修正 4 篇 traj_optimize 文档 |
| 6 | **独立审核**：`Visualizer` 与节点命名空间共存的锚点指向 `.cpp` 且声称有 `using` | 实际证据是节点头的成员声明（`ros2_node.h:96`），且代码中没有 `using`；已改写 |
| 7 | **独立审核**：「28 条未被引用键全是序列元素」漏掉 1 条真实缺口 | 实测 27 条序列元素 + 1 条死配置（`planner.yaml:97` 的 `min_replan_interval`）；已在配置说明与待确认清单中拆开 |
| 8 | **独立审核**：「58 条配置绑定佐证」与证据口径不符 | 58 条绑定里只有 13 条是 `config` 根键链（覆盖本段与 `mapping_model`）；已改为 13 |
| 9 | **独立审核**：4 处锚点精度问题（死成员、PUBLIC 链接、地图缺失、隧道区域缺失、点选闸门） | 逐条改用精确行号（含 `h:109`/`h:118-121`/`:560-563`/`:780-789`/`:504-506`） |
| 10 | **独立审核**：锁的结论偏窄、两处口径不一致 | 补充「其中一处加锁同时读一次栅格地图」，待确认条目补 `:793` |
| 11 | **独立审核**：「唯一直接读取参数文件」措辞偏宽 | 限定为「`planner.yaml` 的唯一解析者」 |
| 12 | **独立审核**：新增 3 条建议 | 已补进待确认清单：隧道检测有效采样 0.002 s（声明默认 0.02 s 不生效）、两个可视化重载均无调用点、锁范围与将来并发路径 |

### 独立审核 Agent（不同上下文）

只读审核 Agent 逐条重取证，**11 条高风险断言在符号/行号/计数层面全部成立**：

- 接口计数与方向（24 个可视化发布者 + 4 个节点发布者 + 4 订阅 + 2 服务 = 34）；
- 「唯一依赖 rclcpp」；回调组划分与三个定时器周期；点选订阅落在默认组；
- 锁的两处位置与「地图路径不加锁」的结论；
- `plan_start_time_` 的赋值点与无条件取值；
- 构造期重复加载地图；硬编码 ±3/±20/三话题名/P5-only；
- 死成员与死常量逐条；折叠链路每一步（含末尾仍在隧道内的补区间）；
- 12 行参数表与 `params.json` 逐条一致；入口与 launch 描述；
- 99 条符号行号抽查全部与符号表一致。

审计结论：首轮 **NEEDS_FIX** —— 3 处事实/锚点错误 + 8 处精度/措辞问题，已逐条修复并补记 3 条待确认；修后校验器仍 `errors=0`。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 5 个源文件全部被文档提及，缺失 0
- 注入测试：假话题、越界行号、编造符号三类全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 12 条发现已逐条修复，修后 `errors=0`
- 范围合规：本轮未改动 `src/`、`config/`、`launch/`、任何 `CMakeLists.txt`（**注**：工作区中根 `CMakeLists.txt` 存在未提交改动，那是用户自己引入的，不属于本轮改动）；未执行构建；未执行提交类操作

---

## 未解决问题（留给后续）

1. **utils 完成后需要再次统一摘要**：本模块文档现用 `937d4a38ea08d42c`，
   utils 证据加入后全局摘要会再变，届时 7 个模块一并更新（收尾步骤）。
2. 人工审核闸门未过（本模块 9 篇 `status: draft`）。
3. 代码侧待办（未改代码，仅登记）：锁注释与用法不一致、20 s 计时的隐式前置条件、
   构造期重复加载地图、硬编码阈值与话题名、死成员/死常量、`is_minco` 死配置、
   地图格式 P5-only、4 处绝对路径、可视化器全局命名空间与体积。

---

## 期间的上游改动与第二次重新基线（2026-10-02 追加）

本模块文档完成后、utils 文档进行期间，用户提交了 `8f98daee5bc7`
（`feat(ros2):优化可视化 ， 日志`），改动了本模块的两个文件：

| 改动 | 位置 | 对文档的影响 |
|---|---|---|
| `visualizer.hpp` +91 行：新增两个 `PubOptPath` 重载（五次样条版与多项式样条版） | `src/ros2/include/ros2/misc/visualizer.hpp:517`、`src/ros2/include/ros2/misc/visualizer.hpp:559` | 符号 189 → **191**；`PubGlobalPath` 之后的所有方法行号位移（515→606、635→726、…、1217→1308），接口文档 13 个锚点与 4 个区间行全部更新；「1396 行 / 23 方法」更新为「1487 行 / 24 方法」 |
| 优化路径可视化改传样条轨迹 | `src/ros2/src/ros2_node.cpp:299` | 原表述「优化航点路径」修正为「直接传样条轨迹给五次样条重载」 |
| RUNNING 状态日志被注释 | `src/ros2/src/ros2_node.cpp:381` | 文档未引用该日志，无需改动 |

处理方式：**重跑 ros2 证据** → 修正上述锚点与表述 → 重跑校验器回到 `errors=0`；
同时把 6 个模块（map + planner 四个 + ros2）的 `git_rev` 更新为 `8f98daee5bc7`、
摘要统一为 `be346b5265f1410d`。

**结论**：本模块文档在上游改动后已重新对齐源码；未发现该改动与其它文档结论冲突
（顺带确认 traj_optimize 文档引用的 `visualizer.hpp` 锚点都位于位移点之前，无需改动）。
