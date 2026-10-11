# utils 模块文档（步骤 5 之二）改造记录

记录日期：2026-10-02
任务范围：`docs/nav/utils/`（新增 9 篇模块文档 + 证据包），**不改动被文档化的代码**
（唯一的例外见「Behavior intentionally adjusted」1，已获用户明确授权）
顺序：本轮为「按顺序完成 ros2 → utils」的第 2 个（ros2 已先完成）

---

## User Intent

1. 用户要求：**按顺序**完成 `nav/ros2/` 与 `nav/utils/` 的文档，不要并行。
2. 契约沿用 `nav/example/README.md`：只基于 Evidence Pack、9 个固定文件、锚点、
   不确定项显式降级。
3. 仓库级约束（`AGENTS.md`）：最小改动、不编造、不改代码/参数、禁止未授权构建、
   禁止任何 git 提交类操作。
4. 针对 0 字节 `src/utils/src/utils.cpp` 造成的校验器边界，用户明确选择
   **「1+2」**：既最小修补校验器，也给该文件补一行说明注释（授权改代码）。

---

## Scope

- `docs/nav/utils/`：新增 9 篇文档
- `docs/_evidence/utils/inventory.json`：新增证据（11 文件 / 1010 符号 / 失败 1：空文件）
- `docs/_evidence/verify.utils.json`：校验报告
- 用户授权的两处非文档改动：
  1. `docs/tools/verify_docs.py`：允许对「存在但 0 行」的文件使用 `:1` 锚点
  2. `src/utils/src/utils.cpp`：补一行说明注释（0 字节 → 1 行）
- `docs/ai_refactor_records/`：本记录

## Out of Scope

1. 不改其它源码；不清理 matchit/plotter 等闲置资产。
2. 不改 CMake 的依赖可见性（只在文档里登记）。
3. 不做任何 git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读，11 文件 / 6279 行）：

- `src/utils/include/utils/logger.hpp`（130）
- `src/utils/include/utils/scope_timer.hpp`（124）
- `src/utils/include/utils/eigen_alias.hpp`（140）
- `src/utils/include/utils/type_utils.hpp`（84）
- `src/utils/include/utils/yaml_loader.hpp`（153）
- `src/utils/include/utils/fmt_eigen.hpp`（105）
- `src/utils/include/utils/plotter.hpp`（60）
- `src/utils/include/utils/lbfgs.hpp`（807，第三方单头）
- `src/utils/include/utils/expected.hpp`（2475，第三方单头）
- `src/utils/include/utils/matchit.h`（2204，第三方单头）
- `src/utils/src/utils.cpp`（原本 0 字节）

消费方抽样（只读）：map 的 `3d_occ_map/config.hpp`、ros2 的 `ros2_node.h`、
traj_optimize 的 `traj_optimizer.h`、replan 的 `fsm_replanner.h`、controller 的 `mpc.h`。

### Active logic path

```
业务模块 include 本模块头
  ├─ 日志：logger::info/warn/error/debug（构造即写，自动带文件:行）
  ├─ 计时：utils::TimeConsuming 构造开始 → 析构按量级上报 logger::timer
  ├─ 类型：utils::decimal_t → Vecf/Matf/... → RobotState
  ├─ 参数：yaml_loader::YamlLoader → 每次 LoadParam 重新 LoadFile → 取值/校验/回落
  └─ 算法：traj_optimize 直接用 lbfgs 单头；traj_optimize/replan 用 tl::expected
```

### Data flow

- 参数装载是**每次调用读一次文件**（`yaml_loader.hpp:49`、`:126`）。
- 类型别名以 `decimal_t = double` 为根（`eigen_alias.hpp:22`）。
- 日志通道是头文件内 inline 变量，进程一份（`logger.hpp:101-125`）。

### Risk notes

1. **依赖可见性**：CMake 把 spdlog/yaml-cpp 设为 PRIVATE，但公共头直接 include 它们
   （`CMakeLists.txt:13`、`logger.hpp:5-8`、`yaml_loader.hpp:12`）。
2. **文件不存在未处理**：只捕获类型转换异常，`YAML::LoadFile` 的读取异常直接上抛
   （`yaml_loader.hpp:49`）。
3. **每次装载重读文件**（无缓存）。
4. **闲置资产**：`matchit.h` 全仓库零引用；`tools::Plotter` 唯一引用是 ros2 的未使用成员，
   且该头依赖未声明的 nlohmann/json、套接字不校验、析构不关闭。
5. **重复宏**：`DEBUG_FILE_DIR`/`PCD_FILE_DIR`/`SIGN` 在两处定义。
6. **风格不一致**：`lbfgs.hpp` 用 `eigen3/` 前缀；`type_utils.hpp` 用相对路径 include。
7. **空白实现文件**：唯一的 .cpp 原本 0 字节，导致校验器无法对它给出锚点（见下）。

### Recommended modification boundary

只新增 `docs/` 下文件；闲置资产、依赖可见性只登记；
0 字节文件与校验器边界按用户选择处理。

---

## Modifier Changes

### Files changed

| 文件 | 变更 |
|---|---|
| `docs/nav/utils/README.md` | 新增：背景、五类基础设施、边界、消费方、11 文件清单 |
| `docs/nav/utils/模块设计说明.md` | 新增：目标约束、组件协作、数据流、关键决策、线程、10 条限制 |
| `docs/nav/utils/接口文档.md` | 新增：按 7 个文件分组的接口表 + 3 个第三方单头的主要接口（1010 条符号） |
| `docs/nav/utils/依赖关系.md` | 新增：CMake、外部库、内部依赖图、6 个消费者、6 条风险 |
| `docs/nav/utils/流程图.md` | 新增：依赖图、日志链、参数装载流程、计时时序、别名传播 |
| `docs/nav/utils/配置说明.md` | 新增：本模块不产生参数证据 + 装载语义表 + 编译期宏 + 参数文档入口 |
| `docs/nav/utils/测试要点.md` | 新增：12 条风险点、9 条边界、可观测量、无法验证部分（全【推断】） |
| `docs/nav/utils/常见问题FAQ.md` | 新增：Q1–Q9 + 排查手段 |
| `docs/nav/utils/待确认清单.md` | 新增：4 条参数缺口、2 条读取说明、12 条语义待确认、5 条其他 |
| `docs/_evidence/utils/inventory.json` | 新增（工具产物） |
| `docs/_evidence/verify.utils.json` | 新增（校验报告） |
| `docs/tools/verify_docs.py` | **用户授权**：0 行文件的 `:1` 锚点放行（一行改动 + 一行注释） |
| `src/utils/src/utils.cpp` | **用户授权**：0 字节 → 补一行「纯头文件库」说明注释 |

### Key changes

1. **先确认「本模块不产生参数证据」**：`params.json` 里没有任何 `src/utils/` 记录，
   并解释原因是装载用变量作键；避免把别模块的参数缺口误记到本模块。
2. **把装载语义写成一张可核对的表**：命中/类型不匹配/缺键/必填/文件不存在/读取时机，
   每条一行锚点，其中「文件不存在未处理」是审计价值最高的一条。
3. **区分「机制提供方」与「参数拥有方」**：本模块给机制，键归各业务模块。
4. **闲置资产显式登记**（matchit、plotter），不擅自删除。
5. CLI 边界处理：0 字节文件既无法给锚点、又必须被覆盖率覆盖，
   按用户选择同时做「校验器最小修补」与「补一行注释」。

### Behavior preserved

- 除用户授权的那一行注释与校验器修补外，`src/`、`config/`、`launch/` 零改动。
- 未清理任何闲置资产；未改 CMake 依赖可见性。

### Behavior intentionally adjusted

1. `src/utils/src/utils.cpp` 由 0 字节变为 1 行注释（用户授权）；副作用是
   **工作区再次不干净**，因此 7 个模块共 63 篇文档的 `worktree_dirty` 统一改为 `true`。
2. `docs/tools/verify_docs.py` 的行号检查由 `last > total`
   改为 `last > max(total, 1)`，只影响 0 行文件；对其它模块行为不变，
   并在 `example/README.md` 的已知边界一节登记。

### Notes

1. **本轮期间仓库发生两次提交**，随之做了两轮重新基线：
   - `f150fbcca0c7`（docs 提交）：把 6 个模块的 `git_rev`/`worktree_dirty`/摘要统一；
   - `8f98daee5bc7`（ros2 可视化与日志改动）：`visualizer.hpp` +91 行、
     `ros2_node.cpp` 两处行为变化（优化路径可视化改传样条轨迹、RUNNING 日志被注释），
     因此**重跑 ros2 证据**（189 → 191 符号）、修正 ros2 文档的 13 个位移锚点与
     「1396 行/23 方法」等计数，并把 6 个模块的 `git_rev` 与摘要再次统一。
   - 期间确认 traj_optimize 文档引用的 `visualizer.hpp` 锚点全部 < 位移点，无需改动。
2. 本轮所有模块当前共用摘要 `be346b5265f1410d`、`git_rev: 8f98daee5bc7`。
3. 库存工具沿用已编译好的二进制（未触发构建）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：日志/计时/类型/装载/单头五条路径逐条核对
- [x] diff 检查：改动全部落在 `docs/` 与授权的一行源码注释
- [x] grep 检查：`matchit` 零引用、`map_mutex_`/`ROOT_DIR`/宏重复、`LoadFile` 两处
- [x] XML / launch / yaml 检查：`params.json` 过滤确认无 `src/utils` 记录
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py --module utils` → **errors=0**、coverage **11/11**、warnings=9
- [x] 注入测试：假话题 `/cmd_vel`、越界行号 `logger.hpp:99999`、
      编造符号 `utils::nonexistentMethod` 三类全部被识破
- [x] 独立审核 Agent（不同上下文）：结论见下
- [x] 如需构建，已取得用户明确许可：**本轮无需构建**，也未执行构建
- [x] 提交类操作：**未执行**

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 首轮 5 个 error：`YAML::LoadFile` 与 `inline` 被反引号包住（前者非本模块符号、后者支撑不住锚点）；两个小节既无锚点也无降级标注 | 去掉反引号、给小节补锚点与【待确认】标注；未放宽校验器 |
| 2 | `lbfgs_optimize` 锚点误写成匿名枚举行 | 改为实际函数行（`lbfgs.hpp:438`） |
| 3 | 0 字节文件导致校验器无解（覆盖率要锚点、锚点要行号） | 按用户选择「1+2」处理，并在 `example/README.md` 登记该边界 |
| 4 | 本轮期间仓库两次提交，ros2 源码改动使 13 个锚点位移 | 重跑 ros2 证据并逐条修正，详见 Notes 1 |
| 5 | **独立审核**：「本模块是唯一不依赖 ROS 的库」是事实错误 | map 与 planner 四个模块同样不依赖 ROS（只有 ros2 用 ament/rclcpp）；已改为「库目标不依赖 ROS」并补上「顶层工程强依赖 ROS」的条件 |
| 6 | **独立审核**：`RobotState` 未初始化字段数写 6 | 实际 9 个字段（位置/速度/加速度/加加速度各一个）中只有 2 个有初始化器 → 未初始化 **7** 个；已更正 |
| 7 | **独立审核**：README/测试要点仍称 `utils.cpp` 为「0 字节空文件」 | 与本轮补注释后的事实矛盾；已统一改为「占位实现文件（本轮补一行说明注释）」 |
| 8 | **独立审核**：依赖口径不准（Eigen「由下游各自提供」、静态库因果） | 顶层有统一 `find_package`（`CMakeLists.txt:52-55`），且顶层 nlohmann 声明被注释（`CMakeLists.txt:60`）；已按证据改写三处 |
| 9 | **独立审核**：6 处锚点精度问题（lbfgs/tl 命名空间锚到符号行、绘图器资源管理只锚后半句、跨模块断言缺上游锚点、基础通道过度概括、自引用出处、表头错位） | 逐条修正（`lbfgs.hpp:8`、`expected.hpp:141`、补 `plotter.hpp:19` 与 `ros2_node.h:109`、通道分工改写、出处改指、表头改「锚点」） |
| 10 | **独立审核**：新增 4 条建议 | 已补进待确认清单：独立构建前提、nlohmann 声明去留、顶层统一 find_package 的约定、空文件措辞与 `worktree_dirty` 复核 |

### 独立审核 Agent（不同上下文）

只读审核 Agent 逐条重取证，**11 条高风险断言基本全部成立、无编造**：

- 纯头文件库（11 文件 / 1010 符号，唯一 .cpp 为 1 行注释）；
- 4 个日志包装与 15 个通道（逐个通道名核对，含 `ros2` 通道的 spdlog 名 `plan_ros2_node`）、
  `std::source_location` 捕获点、彩色格式串、被注释的文件通道；
- 计时器三种构造、单位换算、源码拼写 `set_enbale`、三个控制成员；
- `decimal_t` 与别名族锚点逐行核对、`RET_CODE` 12 值、`GridType` 6 值；
- **装载器四条结论**：路径键/平键两分支、类型不匹配回落、仅 required 抛异常、
  文件不存在异常未捕获、每次装载重读文件（`LoadFile` 恰好 2 处）；
- CMake 的 PRIVATE 与公共头不匹配、utils 无 `find_package`、构建顺序、`ROOT_DIR` 注入；
- `params.json` 中 `src/utils` 出现 0 次及其原因（变量键）；
- 闲置资产（matchit 零引用、Plotter 唯一引用链）与第三方来源/许可；
- 消费方集合（含 controller 的隐式链路逐跳验证）。

审计结论：首轮 **NEEDS_FIX** —— 3 处必须修正（不依赖 ROS 的唯一性、字段计数、
空文件措辞陈旧）+ 11 处精度/口径问题 + 4 条补记，已逐条修复；修后校验器仍 `errors=0`。

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 11 个源文件全部被文档提及，缺失 0
- 注入测试：假话题、越界行号、编造符号三类全部被识破
- 独立审核：首轮 `NEEDS_FIX` 的 14 条发现已逐条修复，修后 `errors=0`
- 范围合规：除用户授权的「校验器一行修补 + 空 .cpp 一行注释」外，
  `src/`、`config/`、`launch/` 零改动；未执行构建；未执行提交类操作

---

## 未解决问题（留给后续）

1. 人工审核闸门未过（本模块 9 篇 `status: draft`）。
2. ~~`worktree_dirty: true` 是当前真实状态（那一行注释未提交）~~ → **已回落**，见文末「后续」。
3. 代码侧待办（未改代码，仅登记）：依赖可见性 PRIVATE 与公共头不匹配、
   装载无缓存与文件不存在异常、重复宏、`eigen3/` 前缀与相对 include 风格、
   matchit 闲置、plotter 的资源管理与未声明依赖、第三方单头无版本记录。

---

## 后续（同日追加）：撤回那一行注释，`worktree_dirty` 回落

用户随后要求把 `worktree_dirty` 回落。当时 `src/utils/src/utils.cpp` 那一行注释**尚未提交**，
因此「回落」只有两条路：提交它，或撤回它。用户选择**撤回**。

处理内容：

1. `src/utils/src/utils.cpp` 恢复为 **0 字节**（`git status --porcelain -- src config launch` 重新为空）。
2. 7 个模块共 **63 篇**文档的 `worktree_dirty` 由 `true` 回落为 `false`
   （`git_rev` 仍为 `8f98daee5bc7`，`evidence_digest` 不变，因为证据文件不包含源码内容）。
3. utils 文档中「占位实现文件 / 本轮补一行说明注释」的措辞全部改回
   **「空文件（0 字节）」**，并在 FAQ Q8 说明：校验器已允许对「存在但 0 行」的文件使用 `:1` 锚点，
   因此空文件同样可以被文档锚定。
4. 6 个既有模块的 `待确认清单` 第 0 条统一改写为「已解决（本轮回落）」，保留历次基线变更的轨迹。
5. 校验器的一行修补**保留**（它是 0 行文件可被锚定的前提，也是本模块能同时满足
   「覆盖率」与「锚点行号」两条约束的原因）；`example/README.md` 的已知边界登记同样保留。

**最终状态**：7 个模块 `errors=0`、非 `review.pending` 警告 0、覆盖率全满、
`worktree_dirty: false`、`git_rev: 8f98daee5bc7`、`evidence_digest: be346b5265f1410d`。
