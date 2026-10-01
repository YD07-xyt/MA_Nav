# controller 模块文档（步骤 4 之三）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/planner/controller/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**
顺序：本轮为「按顺序完成 planner 三个模块」的第 2 个（traj_optimize → **controller** → replan）

---

## User Intent

1. 用户要求：**按顺序**完成 `nav/planner/{traj_optimize,controller,replan}` 的文档，不要并行。
2. 契约沿用 `nav/example/README.md`：只基于 Evidence Pack、9 个固定文件、锚点、
   不确定项显式降级。
3. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不改代码/参数、禁止未授权构建、
   禁止任何 git 提交类操作。

---

## Scope

- `docs/nav/planner/controller/`：新增 9 篇文档
- `docs/_evidence/controller/inventory.json`：新增证据（4 文件 / 80 符号 / 0 失败）
- `docs/_evidence/verify.controller.json`：校验报告
- `docs/ai_refactor_records/`：本记录

## Out of Scope

1. 不改任何被文档化的代码（`src/`、`config/`、`launch/`、`CMakeLists.txt`、参数默认值）。
2. 不写 replan 的文档（按用户要求顺序排后）。
3. 不重新生成 `docs/_evidence/repo/*.json`。
4. 不实现 `YawController::control`（只登记）。
5. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读）：

- `src/planner/controller/CMakeLists.txt`（29 行）
- `src/planner/controller/include/controller/mpc.h`（530 行，MPC 全部实现）
- `src/planner/controller/include/controller/traj_interface.hpp`（141 行）
- `src/planner/controller/include/controller/yaw_plan.hpp`（23 行）
- `src/planner/controller/src/mpc.cpp`（37 字节，仅一行注释）

调用方与跨模块（只读）：

- `src/ros2/include/ros2/ros2_node.h`、`src/ros2/src/ros2_node.cpp`、`src/ros2/include/ros2/config.hpp`
- `src/planner/replan/CMakeLists.txt`（只为确认「链接但不用」）
- `src/utils/include/utils/logger.hpp`、`config/planner.yaml`、顶层 `CMakeLists.txt`

### Active logic path

```
ros2 构造：Mpc(config.mpc_params)（ros2_node.cpp:34）
  → ctor: discretize()（mpc.h:91-93）→ init_solver()（mpc.h:95-96）
新轨迹：mpc_.set_trajectory(ma_traj_interface_)（ros2_node.cpp:313）
每帧：mpc_.solve(x0, t_now, u_cmd, predicted_states, predicted_inputs)（ros2_node.cpp:179）
  → build_problem: traj_->valid() → sample_sequence(t_now, dt, N) → g_ / lb_ / ub_
  → updateGradient/updateBounds → solveProblem → 状态判定 → 提取 u_cmd 与预测序列
  → 调用方用 predicted_states[1] 的速度分量作为速度指令（ros2_node.cpp:186-189）
```

### Data flow

- **参数**：`config/planner.yaml:109-131` 的 `mpc_param` → `load_mpc_param`（`config.hpp:272-289`）
  → `Config.mpc_params`（`config.hpp:314`）→ `Mpc` 构造函数（`ros2_node.cpp:34`）；
  构造后不再更新，因此 Hessian 与约束矩阵可以做成常数。
- **参考轨迹**：经 `TrajectoryInterface` 抽象进入，控制器只认识
  `ReferencePoint`（时间/状态/控制）。
- **ROS 接口**：本模块零 ROS 接口。

### Risk notes

1. 权重矩阵**只取对角线**（`mpc.h:185`、`:194`、`:202-205`），非对角项静默失效。
2. 参数里的「上一帧控制量」**写而不读**（`mpc.h:34` 声明、`mpc.h:525` 赋值），
   跨帧控制平滑实际未生效。
3. 权重默认值是**零矩阵**（`mpc.h:20-25`）→ YAML 删键不报错但代价退化。
4. 状态硬约束**只覆盖 vx/vy 且从 k=1 开始**（`mpc.h:363-369`），x/y 限幅是无效配置。
5. `YawController::control` 只有声明、无定义、无调用点（`yaw_plan.hpp:20`）。
6. 日志是**隐式 include** 进来的（`mpc.h:6` 只 include 了接口头）。
7. 求解接受**不精确解**（`mpc.h:464`）。
8. 轨迹短于预测时域时参考被钳制到终点（`traj_interface.hpp:79`），不报错。
9. `libqpOASES.a` 被链接但源码零引用（`CMakeLists.txt:23`）；replan 链接本模块但不 include
   （`replan/CMakeLists.txt:18`）。
10. `src/` 下仍有本轮之前的未提交改动 → `worktree_dirty: true`。

### Recommended modification boundary

只新增 `docs/` 下文件与一个模块证据 JSON；不实现 yaw 控制器、不清理无效链接、
不改 CMake 路径。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/planner/controller/README.md` | 新增：背景、三部分构成、边界、入口、4 文件清单 |
| `docs/nav/planner/controller/模块设计说明.md` | 新增：设计目标、类协作、数据流、关键决策、线程、9 条已知限制 |
| `docs/nav/planner/controller/接口文档.md` | 新增：**全部 80 条符号**按文件/类分组并注语义 |
| `docs/nav/planner/controller/依赖关系.md` | 新增：CMake、外部库、内部依赖、双向跨模块依赖、6 条风险 |
| `docs/nav/planner/controller/流程图.md` | 新增：主数据流、单帧时序、QP 结构、失败分支状态图 |
| `docs/nav/planner/controller/配置说明.md` | 新增：机制 + 10 行参数表 + 6 行差异项 + 缺口 + 可移植性 |
| `docs/nav/planner/controller/测试要点.md` | 新增：14 条风险点、9 条边界、可观测量、无法验证部分（全【推断】） |
| `docs/nav/planner/controller/常见问题FAQ.md` | 新增：Q1–Q9 + 参数类问题 + 排查手段 |
| `docs/nav/planner/controller/待确认清单.md` | 新增：3 条参数缺口、10 条上下文未知读取、10 条语义待确认、5 条其他 |
| `docs/_evidence/controller/inventory.json` | 新增（工具产物） |
| `docs/_evidence/verify.controller.json` | 新增（校验报告） |

### Key changes

1. **把 QP 结构写成可核对的算术**：变量数、约束数、各约束段的行数与来源
   全部对应到 `init_solver`/`build_problem` 的具体行，读者可按 N=20 复算 124/164。
2. **区分「参数存在」与「参数生效」**：`u_prev`、`x_min/x_max` 的 x/y 分量、
   权重非对角项都被标为「存在但不生效」，并给出全仓库检索结论。
3. **把调用方的行为写清楚**：速度指令来自预测状态第 1 步
   （`ros2_node.cpp:186-189`），失败时本帧不下发（`ros2_node.cpp:179-183`）。
4. **不夸大占位代码**：yaw 部分只描述现状（枚举 + 无定义声明 + 无调用点），
   并进入待确认清单。
5. 全部 80 条符号都有人工语义（本模块符号量小，不需要机械索引）。

### Behavior preserved

- 被文档化代码零改动；未修改 `docs/tools/*`、`example/*`、`map/*`、
  path_planning 与 traj_optimize 文档。
- 未删除 yaw 占位代码、未清理未使用的 qpOASES 链接。

### Behavior intentionally adjusted

1. 配置说明把「x/y 限幅无效」写成【事实】并给出约束构建锚点，
   而不是照抄 YAML 注释；这样读者能区分「配置有意保留」与「真的生效」。
2. FAQ 用「真实状态」而不是「实测状态」表述（本仓库无测试，
   避免出现无依据的验证类措辞）。

### Notes

- 库存工具沿用已编译好的二进制（未触发构建）。
- 新增模块证据再次改变全局 `evidence_digest`（本轮为 `cc5179098b163a1c`），
  三个模块完成后统一重基线。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：构造期（离散化 + 求解器初始化）与每帧路径（构建/更新/求解/提取）逐行核对
- [x] diff 检查：改动全部落在 `docs/nav/planner/controller/`、`docs/_evidence/`、本记录
- [x] grep 检查：`rclcpp|yaml` 零命中；`u_prev` 仅声明+赋值；
      `YawController|YawControlIntput` 全仓库零引用；`qpOASES` 仅 CMake；
      `controller/` 的 include 者只有 ros2
- [x] XML / launch / yaml 检查：参数真值取自 `params.json` 解析值（10 个键）
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module controller` → **errors=0**，coverage **4/4**，
      warnings=9（全部为人工审核闸门）
- [x] 注入测试：假话题 `/cmd_vel`、越界行号 `mpc.h:99999`、编造符号
      `control::Mpc::nonexistentMethod` 三类全部被识破
- [x] 独立审核 Agent（不同上下文）：结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 首轮 5 个 error：`unread_keys` / `static_assert` / `solve` 等**非本模块符号**被反引号包住并落在带锚点的行上；`//` 被当成话题；FAQ 出现「实测」 | 逐条去掉反引号或改写措辞；未放宽校验器 |
| 2 | CMake 行号：README 与依赖关系最初把 project/GLOB/add_library/target_sources/include 写成 3/9/13/15/17 | 用 `sed` 逐行核对后改为 3/8/12/14/16（链接段 20-29 正确） |
| 3 | **独立审核**：把「Hessian 只取对角」写成「非对角权重被忽略/不生效」，重复出现在 6 处 | 核实后确认：Hessian 只用对角（`mpc.h:185`）但**线性项用整矩阵**（`mpc.h:327`、`mpc.h:333`），非对角项会经线性项生效，且与 Hessian 不自洽（H≠2Q）。6 处全部改写，并新增待确认条目 |
| 4 | **独立审核**：5 处把维度常量锚到 `traj_interface.hpp:17-18`（实际常量在 15-16 行） | 全部改为 `:15-16` |
| 5 | **独立审核**：README「唯一消费者是 ros2」与依赖关系中「replan 只链接」口径冲突 | 限定为「源码中唯一**使用**本模块的是 ros2」 |
| 6 | **独立审核**：依赖图多画了一条不存在的 `mpc.cpp → mpc.h` 边（该 .cpp 只有注释、无 include） | 删除该边，改为独立节点 |
| 7 | **独立审核**：设计目标第 6 条把注释声称写成「已支持」 | 改写为「注释声称…实际未接线」 |
| 8 | **独立审核**：24 条 unread_keys 只锚到 `planner.yaml:114` 一行 | 放宽为 `:114-131` |

### 独立审核 Agent（不同上下文）

只读审核 Agent 自己重新取证，**复核通过**（10 项高风险断言中的 9 项）：

- 不使用 rclcpp / 不解析 YAML；
- 维度公式与 N=20 时的 124 变量 / 164 约束（并逐一核对了三元组建行数）；
- Hessian 与线性约束矩阵只在构造期构建一次，每帧只更新线性项与边界；
- `u_prev` 写而不读；
- 状态硬约束只覆盖 vx/vy 且从 k=1 开始，x/y 限幅确实无效；
- yaw 控制器未实现、无调用点（且没有任何文件 include 它）；
- 日志确实是隐式 include（它给出了完整 include 链）；
- 配置说明 10 行逐条正确，24 条 unread 确实是经助手函数读取的序列元素；
- qpOASES 零引用、replan 只链接不 include、CMake 硬编码 `/usr/local` 三项均成立。

审计结论：首轮 **NEEDS_FIX** —— 1 处语义级错误（横跨 6 处）+ 5 处锚点/口径/图边缺陷，
已逐条修复并重跑校验器（仍 `errors=0`）。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 4 个源文件全部被文档提及，缺失 0
- 注入测试：假话题、越界行号、编造符号三类全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 6 类发现已逐条修复并重新校验，修后 `errors=0`
- 范围合规：`src/`、`config/`、`launch/`、`CMakeLists.txt` 零改动；未执行构建；未执行提交类操作

---

## 未解决问题（留给后续）

1. 三个模块完成后统一重基线 `evidence_digest` —— **已完成**（统一为 `7c453c4e8af72579`，map 见文末单独重新基线）。
2. 人工审核闸门未过（9 篇 `status: draft`）。
3. 代码侧待办（未改代码，仅登记）：权重只取对角线、`u_prev` 未参与惩罚、
   权重默认零矩阵、x/y 限幅无效、yaw 控制器未实现、不精确解被接受、
   参考超时域被静默钳制、日志隐式 include、qpOASES 冗余链接、replan 悬空链接、
   CMake 硬编码系统路径。

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
