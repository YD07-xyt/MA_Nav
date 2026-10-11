# nav 文档第 1 次严格审核 —— 核实与修复记录

记录日期：2026-10-02
被核实对象：`docs/ai_refactor_records/audit/20261002_nav_docs_audit_round1.md`（第三方只读审核，NEEDS_FIX）
核实方式：**7 路独立只读子代理**（每路一个模块，不同上下文）+ 父代理自行核实系统层条目
核实基线：`git_rev = 8f98daee5bc7`（HEAD）；根 `CMakeLists.txt` 按 `git show HEAD:` 核对
结论：**审核报告的实质发现几乎全部成立**；已按其结论修正文档，7 个模块回到 `errors=0`

---

## 一、核实方法与总量

| 项 | 值 |
|---|---|
| 被核实的文档内容层条目 | **71 条**（map 20、path_planning 6、traj_optimize 7、controller 5、replan 7、ros2 10、utils 16） |
| 判定 **成立** | **64 条** |
| 判定 **部分成立** | **7 条**（P-4、P-6、T-6、S-7、U-10、U-11、U-13） |
| 判定 **不成立** | **0 条** |
| 子代理另行发现、审核未列的文档错误 | **21 条**（map 2、path_planning 4、controller 3、replan 3、ros2 4、utils 5） |
| 系统层条目（§2、§4）自行核实 | 13 条（S-1~S-4、D-1~D-9） |

核实要求每位子代理「回到源码贴出原文片段」才算成立；审核自己剔除的 8 条误报（其 §5）
经复核**确认其剔除正确**（根因是拿脏工作区的行号比文档，而契约定义行号相对声明的 `git_rev`）。

---

## 二、逐模块核实结果与修复

| 模块 | 审核条目 | 成立 | 部分 | 追加发现 | 已修复 |
|---|---|---|---|---|---|
| map | 20 | 20 | 0 | 2 | M-1~M-20（M-5 在核实前已修）+ MAP-X1/X2 |
| path_planning | 6 | 4 | 2 | 4 | P-1~P-6 + PP-X1~X4 |
| traj_optimize | 7 | 6 | 1 | 0 | T-1~T-7 |
| controller | 5 | 5 | 0 | 3 | C-1~C-5 + CT-X1~X3 |
| replan | 7 | 7 | 0 | 3 | R-1~R-7 + RP-X1~X3 |
| ros2 | 10 | 9 | 1 | 4 | S-1~S-10 + RS-X1~X4 |
| utils | 16 | 13 | 3 | 5 | U-1~U-16 + UT-X1~X5 |

### 最严重的几条（会误导读者理解真实行为）

| 模块 | 原文档说法 | 代码事实 | 处理 |
|---|---|---|---|
| utils | `get_current_dir_name` 返回「当前工作目录名」 | 返回 **Unix 纪元秒**（`scope_timer.hpp:36`），地图模块当时间戳用 | 改写并标注「名字有误导性」 |
| ros2 | 节点是**多线程**、组间并发 | `rclcpp::spin_some(nh)` 单线程（`src/ma_nav.cpp:17`），全仓库无多线程执行器 | 重写「线程与并发」整节 + 连带 6 处 |
| ros2 | 下发新轨迹会**补发**折叠状态 | 补发那行**被注释**（`ros2_node.cpp:334-337`），只更新记账变量 | 改为风险项 |
| map | `getMap` 取「占据矩阵」 | 取 **ESDF 距离缓冲**（`grid_map.cpp:161`） | 改写 |
| map | planner 经 `get_rog_map` 查询地图 | `src/planner/**` 对 `MaMap` **零引用**；查询接口无调用点 | 改写「查询链路」与调用方列 |
| map | `ProbMap` 始终持有 ESDF 子地图 | 只在 ESDF 开启时创建，当前配置下为空指针 | 改写 |
| traj_optimize | `safe_distance`/yaw 参数用于「输出后校验」 | 后校验函数**零调用点**且无 yaw 形参 | 删该表述 |
| replan | 2D 模式「沿用参考航向」 | yaw 被**置零**（`traj_optimizer.h:127`） | 改写并与 traj_optimize 对齐 |
| controller | `Rd` 非对角项经线性项生效 | 梯度只由 Q/QN 与 R 构成，**Rd 无线性项** ⇒ 其非对角完全无效 | 限定该结论 |
| controller | README 称模块「输出底盘指令」 | 调用方丢弃 `u_cmd`，改用预测速度组指令（`ros2_node.cpp:219-225`） | 改写 README |
| utils | `traj_optimize` 依赖本模块计时 | traj_optimize 零引用；真正用的是 **replan** | 改依赖表 |

其余为计数过期（ros2 189→191、map 112→70）、锚点指向注释/相邻重载/声明行、
「三条件都不成立就不会重规划」等逻辑表述、以及根 `CMakeLists.txt` 在 HEAD 与工作区差一行导致的精度问题。

---

## 三、系统层条目核实

| ID | 审核结论 | 我的核实 | 处理 |
|---|---|---|---|
| S-1 | 工作区根 `CMakeLists.txt` 未提交改动使 30 个锚点位移，但按 HEAD 全部正确 | **成立**，且确认这是工作区问题、不是文档缺陷 | 不改锚点（按契约行号相对 `git_rev`） |
| S-2 | `evidence_digest` 不含源码；`worktree_dirty` 范围不含根 `CMakeLists.txt` | **成立**（`verify_docs.py` 只探测 `src config launch`） | 工具改进建议（需授权） |
| S-3 | 重基线只刷 front-matter、正文数字过期 | **成立**：找到 2 处（ros2「189 条」、map「112 条」） | 已修正为 191 / 70 |
| S-4 | 校验器 9 类盲区 | **逐条读代码确认成立**（无符号行不查支撑、末段子串匹配、参数只比对表行、话题只查反引号、`claim.unverified_test` 仅 5 短语、`doc:` 不比对文件名、不要求 `generated_by/at`、不校验 `end >= start`、不读证据包 `git_rev`） | 记录；建议项需授权 |
| D-1 | 记录中存在被证伪的「`CMakeLists.txt` 零改动」声明 | **成立**（根 `CMakeLists.txt` 自 22:12 起为用户改动） | 已改写 ros2 记录措辞（说明那是用户引入的改动，非本轮改动） |
| D-2 | 「人工审核闸门」可被 `status: reviewed` 绕过 | **成立**：`reviewer` 拒绝名单不含「未审核」，且只按 `status` 告警 | 已修正流水线记录里过强的表述；**收紧校验器需授权** |
| D-3 | 9 个模板缺 `worktree_dirty` 字段（而它是必填 error） | **成立**（`grep` 9 个模板全无该字段） | **需授权**后补齐模板 |
| D-4 | 5 篇记录 + 1 篇模块文档仍写已不存在的「当前一致性」 | **成立** | 已更新为最新基线（7 模块/63 篇/`be346b5265f1410d`/`false`）并在 4 份 planner 记录加「最新基线」指引 |
| D-5 | 标注契约只兑现约 10% | **部分成立**：实测 301/3007 锚点行 = 10.0% 属实，但契约（`example/README.md:97-99`）只要求「事实必带锚点、推断/待确认必须显式」，未要求每行都带标记 | 记录；可按需在契约里明确口径 |
| D-6 | traj_optimize 记录「18228 行」不可复现 | **成立**：17 个代码文件为 **17558** 行（18228 是含 4 个 .md 旁注的口径） | 已改为「17 个代码文件 / 17558 行；另有 4 个 .md 旁注」 |
| D-7 | 记录把校验器写成 496 行 | **成立**（写入时为 495，本轮补丁后为 497） | 记录属历史快照，未追溯改写；本记录留档 |
| D-8 | `subnode_reads` 57 vs 70 对账不平 | **不成立（表述冲突而非数字错误）**：70 条是**物理位置**（全在 ros2 的 `config.hpp`），planner 文档的 57 条是**语义归属**（按调用点判定） | 已把 ros2 的分布写准（64 条装载函数 + 6 条构造函数）；两处口径在后文互为解释 |
| D-9 | 契约自身的 front-matter 示例过期 | **成立**（示例仍为 `2eed7d3e884d` / `1ab15719a7937f37`） | 示例可视为「格式示例」；是否刷新**需授权** |

---

## 四、修复后的状态

| 模块 | status | errors | 非 review.pending 警告 | 覆盖率 |
|---|---|---|---|---|
| map | NEEDS_FIX | 0 | 0 | 22/22 |
| path_planning | NEEDS_FIX | 0 | 0 | 8/8 |
| traj_optimize | NEEDS_FIX | 0 | 0 | 17/17 |
| controller | NEEDS_FIX | 0 | 0 | 4/4 |
| replan | NEEDS_FIX | 0 | 0 | 2/2 |
| ros2 | NEEDS_FIX | 0 | 0 | 5/5 |
| utils | NEEDS_FIX | 0 | 0 | 11/11 |

- `docs/tools/test_injection.py --root .` → **PASS**
- 63 篇 front-matter：`git_rev: 8f98daee5bc7`、`evidence_digest: be346b5265f1410d`、`worktree_dirty: false`
- 本轮改动**只在 `docs/`**：37 个文档文件被修正 + 1 份改造记录 + 1 份工具（0 行文件锚点规则，用户先前授权）

---

## 五、未处理项（需用户决策 / 授权）

| # | 项 | 影响 | 需要的授权 |
|---|---|---|---|
| 1 | 根 `CMakeLists.txt` 的未提交改动（编译器 flag 与 LTO 段） | 当前不影响校验器（判定范围不含它），但所有 `CMakeLists.txt:NN` 锚点在**工作区**下偏一行；契约按 HEAD 判定，故文档是对的 | 提交 / 还原 / 保留（审核 C.1） |
| 2 | 校验器把根 `CMakeLists.txt` 纳入 `worktree_dirty` 探测 | 一旦纳入，7 个模块会因该文件脏而全部需要 `worktree_dirty: true` | 是否修改 `docs/tools/` |
| 3 | `reviewer` 改为白名单（拒绝「未审核」） | 堵住 D-2 的机械绕过 | 是否修改 `docs/tools/` |
| 4 | 模板补 `worktree_dirty`（9 个）+ `FRONT_MATTER_REQUIRED` 补 `generated_by/at` | 模板现在照抄必失败 | 是否修改 `docs/nav/example/` |
| 5 | 校验器补 `end >= start`、`doc:` 与文件名比对、读证据包 `git_rev` | 提升闭环强度 | 是否修改 `docs/tools/` |
| 6 | 契约示例 front-matter 刷新为当前基线 | 避免读者困惑 | 是否修改 `docs/nav/example/` |
| 7 | 标注覆盖口径（D-5） | 明确「是否要求每个锚点行都带标记」 | 契约措辞由用户定 |

## 六、结论

审核报告的 **64 条成立 + 7 条部分成立、0 条被推翻**，说明它是可信的；
其中 9 条 HIGH 会直接误导读者对真实行为的理解，均已修正。
另有 **21 条**是子代理在核实过程中额外发现的同类错误（说明审核本身也不完整，
两轮独立审核互补）。修复后 7 个模块 `errors=0`、覆盖率全满、全仓库注入自检 PASS。
文档内容层的错误已清掉，剩余的是**工具/契约层**的加固项，需要用户授权后执行。
