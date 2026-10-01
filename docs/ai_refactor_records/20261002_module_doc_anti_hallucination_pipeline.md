# 模块文档防幻觉流水线（步骤 1–3）改造记录

记录日期：2026-10-02
任务范围：`docs/`（文档系统与工具），**不改动被文档化的代码**

---

## User Intent

用户原始目标与关键约束：

1. 为 MA_Nav 构建「文档系统 → AI 知识库 → Agent」链路的第一步：
   给 AI 一个 **example**，让它能按规范读项目并完成各模块文档。
2. 第一批文档必须**高质量、不能有幻觉**。
3. 输出要求：简洁、高信息压缩。
4. 用户明确要求：**先不要写文档**，先给解决方案。
5. 用户选定范围 **A（步骤 1–3）**：契约 + 工具 + `map` 金样例闭环 + 注入测试。
6. 用户选定工具语言 **混合**：inventory 用 C++，其余用 Python，统一 JSON 契约。
7. 用户就 C++ inventory 路径选定 **A 方案**并明确许可：
   在 `docs/tools/cpp/` 下独立构建，且工具运行时调用
   `clang++ -fsyntax-only -Xclang -ast-dump=json`。
8. 无法确定的问题优先询问用户。

---

## Scope

本次允许修改的范围：

- `docs/nav/example/`：规范与模板重构
- `docs/nav/map/`：金样例文档（9 篇）
- `docs/tools/`：证据提取器、校验器、注入测试
- `docs/_evidence/`：证据包与校验报告（脚本产物）
- `docs/ai_refactor_records/`：本记录

---

## Out of Scope

本次明确不处理的内容：

1. **不改任何被文档化的代码**：`src/`、`config/`、`launch/`、`CMakeLists.txt`、
   `package.xml`、任何参数默认值。
2. 不写 `map` 以外模块的文档（`planner`、`ros2`、`utils` 留到步骤 4）。
3. 不做 MkDocs 站点、知识库分块、RAG Agent（步骤 5，且需单独许可生成类命令）。
4. 不实现 `subnode_reads` 的跨函数调用点推导（列为已知边界）。
5. 不做任何 Git 提交类操作。

---

## Explorer Findings

### Files inspected

被文档化侧（只读）：

- `src/map/CMakeLists.txt`、`src/map/include/map/**`、`src/map/src/*.cpp`
- `src/map/include/map/3d_occ_map/config.hpp`（443 行，参数读取主战场）
- `src/map/src/terrain_analysis.cpp`（含被注释的历史实现）
- `src/ros2/src/ros2_node.cpp`、`src/ros2/include/ros2/config.hpp`
- `src/utils/include/utils/yaml_loader.hpp`
- `config/map.yaml`、`config/planner.yaml`、`launch/run.launch.py`
- `docs/nav/AGENTS.md`、`docs/nav/example/*`、`docs/interface.md`
- `build/compile_commands.json`（19 条）

### Active logic path

```
ros2_node.cpp:33 构造 MaMap
  → MaMap::update_odom / update_cloud（缓存 + 里程计新鲜度过滤）
  → MaMap::update_map
      → ROGMap::updateProbMap
          → ProbMap::raycastProcess → insertUpdateCandidate
          → ProbMap::probabilisticMapFromCache → applyForgetting
      → MaMap::update_terrain → TerrainAnalyzer::analyze
      → dynamic_2d_occ_ 融合 global_2d_occ_ → GridMap::setMap
```

### Data flow

- 参数：`config/planner.yaml:10 → ros2.map_params_path → config/map.yaml`
  → `rog_map::Config(cfg_path, name_space="rog_map")`
  → `YamlLoader::LoadParam("rog_map/...", member, default)`
- 消息：ros2 层手动喂点云/里程计；`map` 模块**不注册任何 ROS2 接口**
- 地图查询：planner 只 include `grid_map.hpp` 与 `ma_map.hpp`

### Risk notes

1. **两种参数读取机制并存**：`map` 用 `LoadParam("前缀/路径", ...)`，
   `ros2` 用 yaml-cpp 键链（含作用在局部子节点上的 `node["key"]`）。
   只覆盖一种会把「读不到」误报成「没被读」。
2. **注释里的旧值**：`config/planner.yaml:7` 的
   `cmd_vel_name: "/cmd_vel_chassis" #"/cmd_vel"`。
   纯文本匹配会把注释当真值 → 必须使用**解析后的值**。
3. **`traj_optimize` 下有两套同名 spline 头**（`ma_spline_opt/` 与
   `spline_opt/`），步骤 4 必须先判定「当前生效」再写文档。
4. **`src/` 存在本会话之前的未提交改动**（23:05–23:20，本会话 00:11 开始），
   其中 `src/ros2/include/ros2/config.hpp`、`src/ros2/src/ros2_node.cpp`
   直接构成本轮证据 → 文档描述的是工作区状态，不是 HEAD 提交状态。
5. **仓库无任何测试**（`src/` 下 0 个 test 文件）→ 「测试要点」只能是【推断】。
6. `src/map/src/terrain_analysis.cpp` 保留约 140 行被注释的历史实现。
7. `config.hpp:180` 与 `:181` 是两条完全相同的 `LoadParam(p_free)` 语句。

### Recommended modification boundary

只新增 `docs/` 下的文件；证据提取与校验全部走脚本，人工不参与事实填写；
金样例只做 `map` 一个模块，验证通过后再复制到其余模块。

---

## Modifier Changes

### Files changed

**规范与模板（`docs/nav/example/`）**

| 文件 | 变更 |
|---|---|
| `README.md` | 由「输出格式清单」重写为**强制契约**：输入契约（Evidence Pack）、输出契约（9 文件 + front-matter）、证据契约（三段式 + 锚点）、校验契约（三层检查表）、禁止项、验收标准、注入测试、重新生成命令、覆盖边界。**原 `【任务】/【要求】` 原文保留在附录** |
| `模板/README.md` | 新增（模块概述骨架） |
| `模板/模块设计说明.md` | 新增 |
| `模板/接口文档.md` | 新增；**原 `example/接口文档.md` 的 Doxygen 规范全文保留** |
| `模板/依赖关系.md` | 新增（原文件为 0 字节） |
| `模板/流程图.md` | 新增；保留原三条「优先完成」要求 |
| `模板/配置说明.md` | 新增 |
| `模板/测试要点.md` | 新增；保留原「优先完成…输出测试点」要求 |
| `模板/常见问题FAQ.md` | 新增（原文件仅标题） |
| `模板/待确认清单.md` | 新增；保留原「AI 不确定的功能,问题」要求 |
| 原 8 个平铺 `.md` | 删除（内容全部迁入 `模板/` 对应文件，无信息丢失） |

> **审计提示**：上述 8 个文件里，`流程图.md` 在删除前**尚未被 Git 跟踪**
> （`git status` 只显示 7 个 `D` 条目）。它的三段「优先完成」要求已完整保留在
> `模板/流程图.md`，此处显式记录，避免日后无法从 Git 历史追溯该文件曾存在。

**工具目录卫生**

| 文件 | 作用 |
|---|---|
| `docs/tools/.gitignore` | 忽略工具运行产物（`__pycache__/`、`_injection_tmp/`、`cpp/inventory/build/`）。根 `.gitignore` 的 `build` 规则已覆盖构建目录，但 Python 缓存与注入测试临时目录需要单独忽略 |

**工具（`docs/tools/`）**

| 文件 | 作用 |
|---|---|
| `cpp/inventory/CMakeLists.txt` | 独立 CMake 工程（不进 colcon、不改 `src/CMakeLists.txt`） |
| `cpp/inventory/src/main.cpp` | LSP 客户端，调 clangd `textDocument/documentSymbol` 提取符号表 |
| `evidence_common.py` | 统一契约、git_rev、YAML 带行号解析、顶层逗号切分、证据摘要 |
| `extract_ros2.py` | pub/sub/srv/timer/ROS 参数；**把 `config.X` 反查成 YAML 解析值** |
| `extract_params.py` | 两种读取机制、代码默认值、缺口（未解析路径/未被引用键/上下文未知读取） |
| `extract_launch.py` | 用 Python `ast` 真解析 launch（注释掉的节点不会误判为已启动） |
| `verify_docs.py` | 三层确定性校验 + 结构/时效/工作区一致性 |
| `test_injection.py` | 注入三类幻觉，验证校验器能 FAIL |

**金样例（`docs/nav/map/`）**：9 篇文档 + `docs/_evidence/` 证据包

### Key changes

1. **把「不编造」从口号变成可执行约束**：每句事实必须带 `文件:行号` 锚点，
   锚点附近必须真的出现被引用的符号，否则 `anchor.unsupported`。
2. **真值只取解析结果**：话题名经「代码变量 → `config["a"]["b"]` 绑定 → YAML 值」
   反查；参数值取自 YAML 解析（带行号），不接受注释文本。
3. **两侧对不上就如实登记**：`unresolved_paths`（代码读、YAML 无）4 条、
   `unread_keys` 28 条（全在 planner.yaml）、`subnode_reads` 70 条，
   全部进入 `待确认清单.md`，不允许 AI 猜。
4. **人工审核做成机器闸门**：`status: draft` / `reviewer` 未填即 `review.pending`，
   AI 无法自行绕过得到 `PASS`。
5. **文档时效可检测**：`evidence_digest` + `git_rev` + `worktree_dirty`
   三项写入 front-matter；`worktree_dirty` 与只读 `git status` 交叉核对，
   声明 false 而实际有改动即 FAIL。
6. **校验器自证**：注入测试证明三类幻觉必被拦下。

### Behavior preserved

- 被文档化的代码**零改动**（`git status` 可证；`src/` 下的 M 条目为会话前既有）。
- 原 `example/` 的全部要求文本保留（README 附录 + 各模板的「生成要求」块）。
- 原有 Doxygen 注释规范与 8 条代码注释规范原文保留。
- `docs/nav/AGENTS.md`（脱敏、溯源、人工审核）未被修改，且在新规范中被引用。
- 根 `.gitignore` 的 `build` 规则已能覆盖新工具构建目录，未改 `.gitignore`。

### Behavior intentionally adjusted

1. 文件名 `常见问题 FAQ.md`（含空格）→ 统一为 `常见问题FAQ.md`，规范中已注明。
2. `example/` 由「平铺 9 个文件」变为「规范 + `模板/`」；
   金样例不入 `example/`，以 `docs/nav/map/` 为**单一 owner**（避免副本漂移）。
3. 验收口径由「校验器 PASS」改为「`errors == 0`；人工审核前为
   `NEEDS_FIX(review.pending)`」——这是设计要求，不是降级。

### Notes

- C++ 工具最初打算用 libclang C API；实测系统**未安装 libclang-14-dev**，
  且 `-ast-dump=json` 单文件输出达 **2.6 GB**（PCL/ROS 头全被拉入）。
  改为 clangd LSP `documentSymbol`：等价 AST 精度、内存友好、
  22 文件 598 符号 0 失败。该方案变更经用户确认（选项 A）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：`MaMap → ROGMap → ProbMap → InfMap/FreeCntMap/ESDFMap/RayCaster`
      与 `GridMap`/`TerrainAnalyzer` 的符号与行号逐条对着源码核过
- [x] diff 检查：`git status --porcelain` 全量核对，逐个确认改动都在 `docs/` 下
- [x] grep 检查：`map` 模块 include 图、跨模块依赖（谁 include `map/`）逐一取证
- [x] XML / launch / yaml 检查：`launch/run.launch.py` 用 `ast` 解析；
      `config/*.yaml` 用 `yaml.compose` 解析（保留行号）
- [x] 用户允许范围内的测试或静态检查：
      `verify_docs.py`（errors=0，coverage 22/22）、`test_injection.py`（PASS）
- [x] 如需构建，已取得用户明确许可：
      已获许可构建 `docs/tools/cpp/inventory`；**未构建 `ma_nav` 本体**
- [x] 提交类操作：**未执行**任何 `git add/commit/push/tag/merge`，
      只用了只读的 `git status` / `git rev-parse`

### Issues found

| # | 问题 | 处置 |
|---|---|---|
| 1 | 校验器首轮报 21 个 error | 逐条判定：13 处为**文档真错**（锚点与符号不匹配、跨模块限定名、越界行号引用、小节无锚点），8 处为**校验器过严**（文件名被当成 YAML 键路径、外部命名空间符号被要求出现在锚点附近、代码片段被当成符号、代码默认话题被当成编造）。**两类分开修**，未用放宽规则掩盖文档错误 |
| 2 | 校验报告写在 `docs/_evidence/` 内，被 `evidence_digest` 自我引用，导致全部文档瞬间 stale | 摘要计算排除 `verify.*.json` |
| 3 | `p_free` 在 `config.hpp:180` 与 `:181` 重复读取 | 不修代码（越界），登记为 `测试要点.md` R6 与 `待确认清单.md` 第 6 条 |
| 4 | `terrain_analysis.hpp:32` 头注释与实现矛盾 | 不修代码，登记为 `模块设计说明.md` 已知限制 #1、`测试要点.md` R1、`待确认清单.md` 第 1 条 |
| 5 | 序列类参数（如 `visualization.range`）被展平成 `[i]`，误报为「键不存在」 | 修提取器：序列本身也产出一条记录 |
| 6 | `map.yaml` 的「未被引用键」全是序列元素下标 | 修提取器：父键被引用时元素不算缺口 |
| 7 | `subnode_reads` 因同一行出现两次而重复计数（112 → 70） | 修提取器去重键 |
| 8 | `src/` 存在会话前的未提交改动，`git_rev` 会误导读者 | 新增 `worktree_dirty` 字段 + 只读 `git status` 交叉校验；9 篇文档声明 `true`；`待确认清单.md` 第 0 条 |
| 9 | 跨模块符号（`utils::yaml_loader`）无法被本模块 inventory 验证 | 文档改用文件锚点表述；`example/README.md` §9.1 登记为已知覆盖边界 |

### Final result

**PASS**

判据：

- `verify_docs.py`：`status=NEEDS_FIX`，**errors = 0**，warnings = 9
  （全部为设计上的人工审核闸门 `review.pending`）
- 覆盖率：模块 22 个源文件全部被文档提及，缺失 0
- `test_injection.py`：**PASS** —— 假话题 `/cmd_vel`、越界行号
  `rog_map.cpp:9999`、编造符号 `rog_map::ROGMap::nonexistentMethod`
  三类幻觉全部被报错，注入副本 `status=FAIL`
- 范围合规：`src/`、`config/`、`launch/`、`CMakeLists.txt` **零改动**
- **未执行 `ma_nav` 本体构建**：AGENTS.md 禁止未授权构建

---

## 提交后重新基线（2026-10-02 追加）

用户将首轮提取时 `src/` 下那批未提交改动提交为 `2eed7d3e884d`
（`refactor:重构轨迹优化，控制器的接口`）。据此做了一次重新基线，结论如下。

### 1. 证据层与新提交等价（这是本轮最重要的核对）

| 项 | 提交前（工作区） | 提交后（HEAD） | 结论 |
|---|---|---|---|
| `src`/`config`/`launch` 未提交改动 | 13 个文件 | **0 个文件** | 提交内容 == 首轮读取的工作区内容 |
| 证据摘要 `evidence_digest` | `1ab15719a7937f37` | `1ab15719a7937f37` | **完全一致** |
| 符号数 / 接口数 / 参数绑定 | 598 / 34 / 61 | 598 / 34 / 61 | 不变 |
| 参数缺口 / 未被引用键 / 上下文未知读取 | 4 / 28 / 70 | 4 / 28 / 70 | 不变 |

因此：**`docs/nav/map/` 的 9 篇文档描述的就是 `2eed7d3e884d`，无需重新生成。**
（`evidence_digest` 的计算刻意排除了 `git_rev` / `generated_at` / `root` 这三个
易变字段，所以「提交」本身不会让文档失效，只有**内容变化**才会。）

### 2. 时效字段已更新

| 文件 | 变更 |
|---|---|
| `docs/nav/map/*.md`（9 篇） | `git_rev: 5cecf8f84447` → `2eed7d3e884d`；`worktree_dirty: true` → `false` |
| `docs/nav/example/README.md` | front-matter 示例同步更新（否则在教一个过期的例子） |
| `docs/nav/map/待确认清单.md` 第 0 条 | 由「源码工作区不干净（最重要）」改为「~~源码工作区不干净~~ **已解决**」，保留原始经过与等价性核对结论 |

`worktree_dirty` 的交叉校验这次真正起了作用：若只改 `git_rev` 而不改
`worktree_dirty`，校验器会直接判 `evidence.worktree_mismatch`（FAIL）。

### 3. 首轮遗留疑问已解答：两个 0 字节的 `.cpp` 不是误清空

首轮记录把 `src/planner/controller/src/mpc.cpp` 与
`src/planner/traj_optimize/src/ma_spline_opt/traj_optimizer.cpp` 为 0 字节
列为「需要确认是预期状态还是误清空」。本次提交的统计给出了答案：

```
src/planner/controller/src/mpc.cpp                            | 447 ---------
src/planner/controller/include/controller/mpc.h               | 526 +++++++++--
src/planner/traj_optimize/src/ma_spline_opt/traj_optimizer.cpp| 365 -------
src/planner/traj_optimize/include/traj_optimize/.../traj_optimizer.h | 372 +++++--
```

即：**实现被有意搬进了头文件（header-only 化），对应的 `.cpp` 被清空**，
属设计变更而非误操作。

对步骤 4 的直接影响（必须写进 planner 文档计划）：

1. planner 模块的符号证据**必须以头文件为主要来源**；
   `documentSymbol` 扫 `src/planner/**/include` 才能拿到实现；
   两个空 `.cpp` 只能作为「已清空，历史实现见 git 历史」的记录项。
2. 本仓库 `file(GLOB_RECURSE "src/*.cpp")` 的构建方式对空 `.cpp` 无害，
   但文档的「依赖关系 / 文件表」必须显式说明这一点，否则读者会以为丢了代码。
3. `traj_optimize` 下同时存在 `ma_spline_opt/` 与 `spline_opt/` 两套同名头，
   本次提交只动了 `ma_spline_opt/` 一侧 → 步骤 4 必须先判定哪一套是当前生效，
   不能两套都写。

### 4. 基线重跑结果

| 检查 | 结果 |
|---|---|
| `verify_docs.py` | `status=NEEDS_FIX`，**errors = 0**，warnings = 9（仍全部为人工审核闸门 `review.pending`）；无 `evidence.stale`、无 `evidence.worktree_mismatch` |
| `test_injection.py` | **PASS** —— 假话题 `/cmd_vel`、越界行号 `rog_map.cpp:9999`、编造符号 `rog_map::ROGMap::nonexistentMethod` 三类仍全部被识破 |

### 5. 修订后的剩余事项

1. 人工审核仍未完成（9 篇 `status: draft`）——这是**设计中的闸门**，需你逐篇确认后改 `reviewed`。
2. `nav/map/待确认清单.md` 第三节的 7 条语义待确认项仍然有效。
3. `subnode_reads` 的跨函数调用点推导仍未实现（70 条，属 planner 范围）。
4. 两个空 `.cpp` 的疑问已关闭；`traj_optimize` 两套同名头的取舍仍是步骤 4 的前置问题。

---

## map 模块单独重新基线（2026-10-02 追加，planner 文档阶段之后）

planner 四个模块（path_planning / traj_optimize / controller / replan）的 36 篇文档完成后，
用户要求**单独**重新基线 `docs/nav/map/`。本轮做的事：

1. **先验证证据可复现（非破坏性）**：按 `example/README.md` §9 的命令把 4 份证据重跑到临时目录后逐字段比对
   （忽略 `generated_at` / `git_rev` / `root`）：
   `repo/ros2.json`（34 接口 / 70 扫描文件）、`repo/params.json`（173 叶子 / 61 绑定 / 70 上下文未知 / 4 缺口 / 28 未被引用）、
   `repo/launch.json`（1 节点 / 1 声明参数）与基线**逐字段一致**；map 符号表重跑为 22 文件 / 598 符号 / 0 失败。
2. **发现 1 条不可位级复现的符号，并保留更准确的基线**：重跑把
   `rog_map::ProbMap::Ptr`（`src/map/include/map/3d_occ_map/prob_map.h:38`，与源码
   `typedef std::shared_ptr<ProbMap> Ptr;` 一致）报成 `ProbMap::shared_ptr`（第 37 行的宏行），
   属 clangd `documentSymbol` 在宏相邻别名声明上的偶发归属差异。
   因此**没有覆盖基线证据文件**，而是把该观察登记为 `docs/nav/map/待确认清单.md` 第 8 条。
3. **更新 9 篇 front-matter**：`evidence_digest: 1ab15719a7937f37 → 7c453c4e8af72579`、
   `worktree_dirty: false → true`（`git_rev` 仍为 `2eed7d3e884d`）。
   后者是因为 `src/` 下有 3 处与本模块无关的未提交改动（planner 侧两个 .cpp 与一个未跟踪的 opt.md），
   校验器会交叉核对这一字段。
4. **更新 `docs/nav/map/待确认清单.md` 第 0 条**：由「已解决」改为「本轮已重新基线」，
   把两次基线变更（提交后重基线、planner 阶段后重基线）的经过都保留下来。
5. **收尾校验**：
   - `verify_docs.py --module map` → `status=NEEDS_FIX`，**errors=0**，warnings=9（全部为人工审核闸门），覆盖率 22/22；
   - `docs/tools/test_injection.py --root .` → **PASS**（前置条件「map 原文档 errors=0」恢复成立，
     三类注入幻觉仍全部被识破）。

**当前一致性**：map / path_planning / traj_optimize / controller / replan 五个模块共 **45 篇文档**
共用同一份证据摘要 `7c453c4e8af72579`，全部 `errors=0`，非 `review.pending` 警告为 0。
仍未完成的只有人工审核闸门（45 篇均为 `status: draft`）。
