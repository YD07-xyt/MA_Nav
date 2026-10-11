# nav 文档第二轮审核修复 + 文档系统加固 改造记录

记录日期：2026-10-03
任务性质：**审核后的修复**（文档内容残留 + 文档系统授权加固）
审核链条：
- 第 1 轮审核：`docs/ai_refactor_records/audit/20261002_nav_docs_audit_round1.md`（第三方只读，NEEDS_FIX）
- 他人修复与自证：`docs/ai_refactor_records/audit/20261002_nav_docs_audit_round1_verification.md`
- 本轮：第 2 轮复审（发现回归与残留）→ 用户授权后执行修复

基线：`git_rev = 8f98daee5bc7ea2384fc390aa02c4f8f4ecf1852`（全程未变）

---

## User Intent

1. 对另一 AI 的修复做**再次审查**（第 2 轮）。
2. 审查后用户明确指令：
   - **「直接修掉」**第 2 轮发现的问题（表格结构回归 + 计数未修完 + 残留）；
   - **「第一节列的『需授权』四项（模板、校验器加固、契约示例）一并授权」**。

---

## Scope

本次允许修改的范围：

- `docs/nav/**`：修复文档内容残留与 3 处表格结构回归
- `docs/nav/example/模板/*.md`（9 个）：补 `worktree_dirty` 字段
- `docs/nav/example/README.md`：契约的示例基线与检查项描述
- `docs/tools/verify_docs.py`：授权范围内的校验器加固
- `docs/_evidence/verify.*.json`：校验器重跑产物
- 本记录

## Out of Scope

1. **`src/`、`config/`、`launch/` 零改动**（已用 `git status -- src config launch` 为空核验）。
2. **根 `CMakeLists.txt` 的未提交改动不由本轮处置**——那是用户的改动（mtime 22:12），
   提交/还原/保留由用户决定。本轮只保证文档锚点相对声明的 `git_rev` 正确。
3. 不改 63 篇的 `status` / `reviewer`：人工审核闸门仍交由用户完成。
4. 不执行构建。

---

## Explorer Findings：第 2 轮复审结论

### 他人修复的正确部分（已逐条回源确认）

| 项 | 结论 |
|---|---|
| 9 条内容层 HIGH（M-1~M-4、S-1、S-2、T-1、R-1、U-1） | **修复正确**，语义与代码一致 |
| 机器层 | 7/7 `errors=0`；front-matter 63/63 一致；覆盖率全满 |
| 注入自检 | `test_injection.py --root .` **独立复跑 PASS**（原文档 errors=0，注入副本 FAIL 3 类） |
| 抽查 MEDIUM/LOW | M-5(70)、M-9、M-12、M-14(**25** 实测 25)、M-15/16/17 锚点、T-2/T-3/T-4、U-2/U-3/U-4/U-10、P-1/P-2/P-4/P-6、C-1/C-2 均正确 |
| 记录层 | D-1 措辞改为「工作区根 CMakeLists 是用户改动」；D-2 收窄为「不改 status 拿不到 PASS」并披露旁路；D-4 加「最新基线」指引 |
| 一处优于第一轮审核 | `subnode_reads` 构造函数内条数：对方 **6 条**（319/334/338/339/342/343）正确，第一轮审核写 5 条有误 |

### 本轮发现的问题（均已修）

**A. 他人本轮新引入的表格结构回归（3 处，`verify_docs.py` 结构性看不见）**

| 位置 | 问题 | HEAD 对照 |
|---|---|---|
| `map/测试要点.md:76` | 表头 2 列、该行 3 列，句尾残留碎片 `\|注释里 \|` | HEAD 该行 2 列 → 新引入 |
| `map/测试要点.md:110` | 表头 5 列、该行 6 列，残留旧锚点碎片 `\|/include/...sliding_map.h:68\`` | HEAD 该行 5 列 → 新引入 |
| `map/依赖关系.md:123` | 5 列表格该行仅 4 列（扩写 PCL 使用点时丢单元格） | HEAD 对应行 5 列 → 新引入 |

**B. 计数未修完（1 处）**：`map/配置说明.md:42` 写 `config.hpp:81-209`，主参数表实际覆盖 **81–212**
（`grid_map.y_left/y_right/resolution` 在 `config.hpp:210/211/212`）。

**C. 残留（含本轮复审新发现的同类漂移，共 8 处）**

| # | 位置 | 问题 |
|---|---|---|
| 1 | `map/待确认清单.md` §二 尾 | 仍写「下一步做 planner 文档时，必须先实现调用点推导」——planner 文档已完成且已按调用点人工确认 |
| 2 | `map/待确认清单.md` §三 item 2 | 仍称 `unfinished_frame_cnt` 有数据竞争风险——`模块设计说明.md` 已判定**不存在**竞争 |
| 3 | `map/待确认清单.md` §三 item 4 | 称「代码未按 `rog_map.esdf.enable` 分支」——代码**确实分支**（`prob_map.cpp:55`），与已修 FAQ 矛盾 |
| 4 | `map/待确认清单.md` §四 item 5/6 | 「下一步实现推导」「planner 模块文档阶段处理」均为过期计划 |
| 5 | `map/测试要点.md` R7 现象 | 称「概率地图仍持有 ESDF 子地图并可经访问器取用」——与已修 FAQ/边界表**直接矛盾** |
| 6 | `replan/模块设计说明.md:25` | 结论已改对，但锚点 `optimizer_config.h:65`（`enum OptModel`）**支撑不了「yaw 被置零」** |
| 7 | `utils/接口文档.md:24` | `logger::info` 行未写出**必填首参**（日志通道），照表写会编译失败 |
| 8 | `path_planning/模块设计说明.md:85` | 单元格内裸写 `× \|Δθ\|` 未转义 → 3 列表格渲染成 5 列（历史遗留，非本轮引入） |

**D. 授权加固四项**：模板缺 `worktree_dirty`；校验器 5 项盲区；契约示例为旧基线；契约检查项表未含新检查。

### Risk notes

- 3 处表格回归都在 `map`，属「替换原句时删了前半、漏删后半」的同类事故。
- 残留集中在 `map` 的 `待确认清单.md` 与 `测试要点.md`：他人修了 FAQ / 设计说明，**未同步清单与测试要点**，形成跨文档自相矛盾。
- 校验器此前**不检查 Markdown 结构**，故上述回归全部 `errors=0` 通过。

---

## Modifier Changes

### Files changed

**文档内容（4 个文件）**

| 文件 | 变更 |
|---|---|
| `docs/nav/map/测试要点.md` | 修 2 处表格回归；R7 现象改为「不创建 ESDF 子地图（被 `cfg_.esdf_en` 门控）」；门控证据移到「证据」行 |
| `docs/nav/map/依赖关系.md` | 风险 3 行补回第 5 列（`【事实】`） |
| `docs/nav/map/配置说明.md` | `81-209` → `81-212` |
| `docs/nav/map/待确认清单.md` | §二尾、§三 item 2/4、§四 item 5/6 全部按已完成的 planner 文档与 ros2 侧结论改写 |
| `docs/nav/planner/replan/模块设计说明.md` | R1 行补锚点 `traj_optimizer.h:120-134`、`:253`（零化证据） |
| `docs/nav/utils/接口文档.md` | §一 增加「首参是必填日志通道」说明 |
| `docs/nav/planner/path_planning/模块设计说明.md` | `\|Δθ\|` 转义 |

**文档系统（授权项）**

| 文件 | 变更 |
|---|---|
| `docs/nav/example/模板/*.md`（9 个） | 在 `evidence_digest` 后补 `worktree_dirty: <true\|false>`，字段顺序与契约 §2.1 一致 |
| `docs/nav/example/README.md` | §2.1 示例刷新为当前基线（`8f98daee5bc7` / `be346b5265f1410d`）；§4 检查项表补 4 个新 ID 并分列 WARN/ERROR；§6.1 增加闸门强制方式说明 |
| `docs/tools/verify_docs.py` | 5 项加固，见下 |

### Key changes：校验器加固（`docs/tools/verify_docs.py`）

| # | 变更 | 作用 |
|---|---|---|
| 1 | 新增 `REVIEWER_PLACEHOLDERS`（含 `未审核`/`待审核`）+ `review.bypass`（**ERROR**） | 堵住「只把 `status` 改成 `reviewed` 即得 PASS」的自我认证旁路；`reviewer` 填真人且 `status: reviewed` 才不告警 |
| 2 | `FRONT_MATTER_REQUIRED` 补 `generated_by` / `generated_at` | 契约要求但此前不校验 |
| 3 | 新增 `structure.doc_mismatch` | `doc:` 必须等于文件名 |
| 4 | 新增 `anchor.range_invalid` | 区间锚点不得倒置（`end >= start`） |
| 5 | 新增 `evidence.rev_drift`（WARN）+ `anchor.worktree_drift`（WARN） | 前者比对**证据包自身**的 `git_rev` 与声明；后者只标记**真正引用了脏文件**的锚点 |
| 6 | 抽出 `WORKTREE_DIRTY_PATHS` 常量 | 便于把 `*CMakeLists.txt` 纳入 `worktree_dirty`（见「有意调整」） |

新增辅助函数：`worktree_changed()`（全仓库改动，排除 `docs/`）、`evidence_revs()`。

### Behavior preserved

- 7 个模块的 `errors == 0`，覆盖率与结构检查口径不变。
- 63 篇 front-matter 未改动（仍 `8f98daee5bc7` / `be346b5265f1410d` / `false` / `draft` / `未审核`）。
- 注入测试的三类幻觉识别能力不变（加固后重跑仍 PASS）。
- 契约 §3 的「行号相对声明的 `git_rev`」原则不变——**未按工作区行号改任何锚点**。

### Behavior intentionally adjusted

1. **`status: reviewed` + 占位 `reviewer` 由 WARN 升级为 ERROR**。这是本轮唯一会改变判定结果的行为变化：
   此前该组合可得 PASS，现为 FAIL。63 篇当前均为 `draft`，因此现状不受影响。
2. **未把根 `CMakeLists.txt` 折进 `worktree_dirty` 布尔位**（这是对授权项的一处**有理由的偏离**）：
   - 若直接纳入，则因该文件当前为脏，63 篇必须全部翻转为 `worktree_dirty: true`；
     而该文件是**用户尚未决定**的改动（提交/还原/保留），会让 63 篇 front-matter 与
     记录里的「最新基线」随之抖动；
   - 改为新增 `anchor.worktree_drift`：**精确**告警「引用了脏文件的那些锚点」（实测 6/7 模块各 1 条，
     `map` 因只引用模块内 `src/map/CMakeLists.txt` 而正确地**不告警**），信息量更大且零抖动；
   - 能力仍保留：把 `WORKTREE_DIRTY_PATHS` 追加 `"*CMakeLists.txt"` 即可切换到全局布尔口径（代码内已注明）。
3. `reviewer` 占位 + `status: draft` 时不再额外报「reviewer 未填写」，避免与 `review.pending` 重复计数
   （warning 数保持每篇一条）。

### Notes

- 新增的 `evidence.rev_drift` 会对每个模块报 2 条（证据包分别生成于 `2eed7d3e884d` 与 `f150fbcca0c7`），
  这是**真实情况**而非误报：`evidence_digest` 不含源码，源码漂移此前无任何机制可发现。
- 因此各模块 warnings 由 9 变为 11–12（9 × `review.pending` + 2 × `evidence.rev_drift` + 0~1 × `anchor.worktree_drift`）。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：本轮所有修改逐条回源（`prob_map.cpp:55-56`、`config.hpp:210-212`、`traj_optimizer.h:120/127/253`、`config.hpp:46`）
- [x] diff 检查：`git status --porcelain` 分类；`git status -- src config launch` **为空**
- [x] 校验器 7 模块重跑：`errors=0`，warnings 11–12，`verify.*.json` 已重新生成
- [x] **新检查自证**：在临时副本上实测
  - `status: reviewed` + `reviewer: 未审核` → `review.bypass` **ERROR** ✅
  - `status: reviewed` + `reviewer: 张三` → 该篇无 `review.pending` ✅
  - `doc:` 与文件名不符 → `structure.doc_mismatch` ✅
  - `:120-5` 区间倒置 → `anchor.range_invalid` ✅
  - 临时目录已删除（`_selftest` / `_injection_tmp` 均不存在）
- [x] 表格结构全库扫描：**0 处**列数不一致（含 `\|` 转义处理）
- [x] front-matter 一致性：63/63 同一组合
- [x] 模板字段：9/9 含 `worktree_dirty`
- [x] 注入测试：加固后重跑 **PASS**
- [x] XML/launch/yaml：本轮未涉及
- [x] 如需构建：**未执行**，AGENTS.md 禁止未授权构建

### Issues found

1. 本轮修复过程中，**加固后的校验器立即报出我自己的一处编辑错误**：
   `map/测试要点.md:96` 的 `config/map.yaml:32` 锚点不支撑新加的 `ProbMap`/`esdf_map_` 符号
   （`anchor.unsupported`）。已改为把门控证据放到「证据」行并附 `esdf_map_` 符号 —— 这条恰好证明新检查有效。
2. `map` 之外 6 个模块出现 `anchor.worktree_drift`：它们引用了当前工作区为脏的根 `CMakeLists.txt`。
   属**正确告警**（锚点在声明的 `git_rev` 下仍然正确），不是文档缺陷。
3. 第 2 轮复审未能覆盖「21 条对方新增发现」的逐条复核（对方记录只列了分类，未列逐条内容）；
   本轮改为按**类别**回归验证（HIGH 逐条 + MEDIUM/LOW 抽查 + 全库结构/计数/一致性扫描）。

### Final result

**PASS**

判据：

- 7 个模块 `verify_docs.py` → `errors = 0`；覆盖率 22/22、8/8、17/17、4/4、2/2、5/5、11/11
- 第 2 轮发现的问题**全部修复**：3 处表格回归 = 0；计数 1 处已修（81–212）；8 处残留已修；4 项授权项已完成
- 表格结构、front-matter、模板字段、注入自检 全部复核通过
- 范围合规：`src/`、`config/`、`launch/` 零改动；未执行构建；未执行任何提交类操作
- **未执行构建**：AGENTS.md 禁止在未获用户明确许可前运行构建命令

> 注：`status` 仍为 `NEEDS_FIX` 是正确的——那是**人工审核闸门**（63 篇 `draft`），
> 不是本轮修复失败。按契约 §6.1，人工逐篇确认后填 `reviewer` 并改 `status: reviewed` 方可得到 `PASS`。

---

## 遗留（需用户处置）

1. **根 `CMakeLists.txt` 未提交改动**：提交 / 还原 / 保留。在决定前，所有 `CMakeLists.txt:NN`
   锚点在**工作区**下偏一行；文档锚点按 `git_rev` 均正确，**不要按工作区改锚点**。
2. **人工审核**：63 篇 `status: draft`；闸门已加固（`review.bypass`）。
3. **`WORKTREE_DIRTY_PATHS` 口径**：是否把 `*CMakeLists.txt` 纳入全局 `worktree_dirty`（会翻转 63 篇）。
4. **标注覆盖口径（第 1 轮 D-5）**：契约是否需要「每个锚点行都带标记」的更强要求。
