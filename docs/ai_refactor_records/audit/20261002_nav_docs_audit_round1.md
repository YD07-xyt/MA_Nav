# nav 模块文档第 1 次严格审核记录（结合代码与实际情况）

记录日期：2026-10-02
审核对象：`docs/nav/` 全部 7 个模块文档（63 篇，约 8000 行）
审核性质：**只读审核**，未修改任何被审文件
审核基线：`git_rev = 8f98daee5bc7ea2384fc390aa02c4f8f4ecf1852`（`git rev-parse HEAD`）

---

## User Intent

用户原始目标与关键约束：

1. `docs/nav/` 下的模块文档由 AI 生成，**尚未审核**。
2. 要求**结合代码以及实际情况进行严格审核，保证文档的正确性**。
3. 隐含约束（来自 `AGENTS.md` 与 `docs/nav/AGENTS.md`）：
   - 不编造、必须可溯源、人工审核闭环；
   - 最小改动、不越界修改 `src/`、`config/`、`launch/`、任何 `CMakeLists.txt`；
   - 未获明确许可不得执行构建。

---

## Scope

本次审核覆盖的范围：

- `docs/nav/map/`（9 篇）
- `docs/nav/planner/path_planning/`（9 篇）
- `docs/nav/planner/traj_optimize/`（9 篇）
- `docs/nav/planner/controller/`（9 篇）
- `docs/nav/planner/replan/`（9 篇）
- `docs/nav/ros2/`（9 篇）
- `docs/nav/utils/`（9 篇）
- 契约与工具：`docs/nav/example/README.md`、`docs/nav/example/模板/*`、`docs/tools/*.py`
- 证据包：`docs/_evidence/**`
- 改造记录：`docs/ai_refactor_records/*.md`

---

## Out of Scope

本次明确不处理的内容：

1. **不修改任何文件**（只读审核）。所有问题只登记、不修复。
2. 不修改 `src/`、`config/`、`launch/`、任何 `CMakeLists.txt`。
3. 不执行构建、不执行注入测试写盘、不执行任何 Git 提交类操作。
4. 不重构文档系统与校验器（相关建议仅作为修复项列出）。

---

## 审核方法

为避免「生成文档的同一上下文自我确认」，采用三路交叉：

1. **机器层复跑（确定性）**：
   ```bash
   python3 docs/tools/verify_docs.py --root . --module <module> \
       --docs docs/nav/<module> --evidence docs/_evidence
   ```
   7 个模块全部执行；`--report` 省略以免写盘。

2. **语义层审计（独立上下文，只读）**：7 路并行子代理，每路只读一个模块的 9 篇文档 + 该模块源码 + 证据包，任务是「找出无法被证据支撑的句子」，输出必须带 `文件:行号` 与源码原文片段作为反证。

3. **回源复核（本次审核的关键环节）**：父代理对**每一条**上报发现回到源码重新取证，**确认成立才登记、不成立即剔除**。同时父代理独立复核了继承关系、行号锚点、符号计数、覆盖率、front-matter、证据摘要等横向项。

> 第 3 步是必要的：本次审核共剔除 **8 条误报**（见第 6 节），若不做回源复核，会误伤正确文档。

---

## Explorer Findings

### Files inspected

被文档化侧（只读，7 个模块共 69 个源文件）：

- `src/map/**`（22）、`src/ros2/**`（5）、`src/utils/**`（11）
- `src/planner/path_planning/**`（8）、`src/planner/traj_optimize/**`（17）、
  `src/planner/controller/**`（4）、`src/planner/replan/**`（2）
- `config/map.yaml`、`config/planner.yaml`、`launch/run.launch.py`、根 `CMakeLists.txt`、
  各模块 `CMakeLists.txt`、`src/ma_nav.cpp`
- 作者旁注（关键真值）：`src/planner/traj_optimize/include/traj_optimize/opt.md`

文档与工具侧：

- `docs/nav/**`（63 篇 + 契约 + 9 个模板）
- `docs/tools/verify_docs.py`（497 行）、`evidence_common.py`、`extract_*.py`、`test_injection.py`
- `docs/_evidence/{map,ros2,utils,path_planning,traj_optimize,controller,replan}/inventory.json`、
  `docs/_evidence/repo/{ros2,params,launch}.json`

### Active logic path（用于判断「活代码 / 死代码」）

```
src/ma_nav.cpp:13 构造 planner::GlobalPlanner2d（src/ros2/src/ros2_node.cpp:33 构造 MaMap）
  → 单线程 rclcpp::spin_some(ma_nav.cpp:17)
      → map_callback    : update_cloud → update_map → ROGMap::updateProbMap
      → planner_callback: fsm_replanner.plan → path_planning(JPS) → traj_optimize(ma_spline_opt)
      → controller_callback: mpc_.solve → 由 predicted_states[1] 速度组 Twist → /cmd_vel_chassis
      → pub_callback    : Visualizer 发布 /ma_nav/*
```

### Risk notes

1. **工作区不干净且与锚点强相关**：根 `CMakeLists.txt` 有未提交改动（LTO 注释化，行号 +1）。
2. **证据包 revision 与文档声明不一致**：`docs/_evidence/repo/*.json` 与多数 inventory 生成于
   `2eed7d3e884d`，而 63 篇文档声明 `8f98daee5bc7`。
3. **两套同名 spline 头**：`ma_spline_opt/`（生效）与 `spline_opt/`（后续用）、`gcopter/`（未使用），
   作者已在 `opt.md:1-5` 明确。
4. **两个 .cpp 被有意清空**（header-only 化），一处 0 字节 `src/utils/src/utils.cpp` 仍在。
5. 仓库 `src/` 下**无任何测试** → 各模块「测试要点」只能全部为【推断】。

---

## 第 1 节：验证通过项（机器层与横向项）

以下项经复核**未发现问题**，是本次审核的「干净面」：

| 项 | 结果 |
|---|---|
| 契约规定的 9 个文件 | 7 模块 × 9 = 63 篇，文件名全部合规 |
| front-matter | 63/63 含契约 9 字段；`doc:` 与文件名一致、`module:` 与目录一致 |
| `verify_docs.py` | 7 模块全部 `status=NEEDS_FIX errors=0 warnings=9`，9 条警告**全部**为 `review.pending`（设计闸门） |
| 覆盖率 | map 22/22、path_planning 8/8、traj_optimize 17/17、controller 4/4、replan 2/2、ros2 5/5、utils 11/11，缺失 0 |
| 三级标注 | `【事实】/【推断】/【待确认】` 全模块在用（275 / 227 / 56） |
| 逐文件符号计数 | 与 `inventory.json` **精确一致**（map 598/22；traj_optimize 1872/17，其中 spline_opt 798 + gcopter 144；utils 1010/11；path_planning 192/8；ros2 191/5；controller 80/4；replan 66/2） |
| 活/死代码判定 | `ma_spline_opt` 为唯一生效实现，`spline_opt`/`gcopter` 无 include 引用，与作者 `opt.md` 一致；JPS 为实际生效搜索器（`fsm_replanner.cpp:69/:124` 显式开启），A* 保留为对照 |
| 空文件诚实性 | `src/planner/controller/src/mpc.cpp` 与 `.../ma_spline_opt/traj_optimizer.cpp` 的 header-only 化、`src/utils/src/utils.cpp` 的 0 字节，均被如实记录 |
| 注释旧值陷阱 | 话题一律使用解析真值 `/cmd_vel_chassis`，**从未**出现注释里的 `/cmd_vel` |
| ros2 接口面 | 34 个主题/服务（28 pub + 4 sub + 2 srv）名称、类型、方向全部正确；QoS、定时器（33/10/10 ms）、回调组绑定正确 |
| 根 CMakeLists 锚点 | 31 个中 30 个在当前工作区偏移，但**按声明的 `git_rev` 全部正确**（见第 5 节误报说明） |
| 证据缺口对齐 | `subnode_reads=70`、`unresolved_paths=4`、`unread_keys=28`、`bindings=61` 与各模块待确认清单对齐 |
| 时效字段 | 现算 `evidence_digest=be346b5265f1410d` 与 63 篇一致；63 篇 `git_rev` = HEAD；`worktree_dirty: false` 与 `git status -- src config launch` 为空一致 |

---

## 第 2 节：系统性发现（最高杠杆，先看这三条）

### S-1 工作区陷阱：根 `CMakeLists.txt` 未提交改动使 30 个锚点错位

- 事实：`git status --porcelain` → ` M CMakeLists.txt`；该文件 LTO 段被注释化，
  并在 `set(CMAKE_CXX_FLAGS_RELEASE ...)` 处**净增 1 行**，其后所有行号 +1。
- 后果：`CMakeLists.txt:NN` 形式的顶层锚点在**当前工作区**下全部偏一行。
- **但**：`git show HEAD:CMakeLists.txt` 下这些锚点**全部与文档声称一致**，例如
  `ros2` 在 HEAD:94（工作区 95）、`controller` 在 HEAD:91（工作区 92）、
  `ament_auto_add_executable` 在 HEAD:76（工作区 77）。
- 定性：这是**工作区/越界缺陷，不是文档内容缺陷**。
- **护栏：不要按当前工作区去改这些锚点。**

### S-2 `evidence_digest` 不含源码，`worktree_dirty` 不覆盖根 `CMakeLists.txt`

- `evidence_common.py:99-114` 只对证据目录下 `*.json` 取摘要，并主动剔除
  `git_rev / generated_at / root`；因此源码变化、正文数字变化均无法触发 `evidence.stale`。
- `verify_docs.py:116` 的探测范围是 `git status --porcelain -- src config launch`，
  根 `CMakeLists.txt` 不在覆盖内。
- 实证：`docs/_evidence/repo/ros2.json` 自身记为 `git_rev: 2eed7d3e884d`，
  而 ros2 文档声明 `8f98daee5bc7`；本次核对接口面**未漂移**（源码实测 34 = 证据 34），
  但机制上不会报警。
- 说明：该局限在 `20261002_nav_utils_docs.md` 中已被作者显式承认。

### S-3 重基线只刷新 front-matter、未刷新正文数字

63 篇于同一时刻被统一改写 `evidence_digest`，但正文中依赖证据包的硬编码数字未同步：

- `docs/nav/ros2/接口文档.md:15`、`docs/nav/ros2/README.md:17` 写「189 条符号」，
  而 `docs/_evidence/ros2/inventory.json` 现为 **191**；
- `docs/nav/map/配置说明.md:163` 写「112 条 `subnode_reads`」，证据实测 **70**
  （本模块 `待确认清单.md:31` 写的是 70）。

### S-4 校验器的具体盲区（决定 `errors=0` 的分量）

| 盲区 | 位置 | 后果 |
|---|---|---|
| 锚点行无被引号包裹的符号时不查支撑性 | `verify_docs.py:314-321` | `CMakeLists.txt:NN` 这类锚点只查存在与行号范围 |
| 符号只按 `::` 末段做 ±6 行**子串**匹配 | `verify_docs.py:156-166` | 注释 `// update() is DEPRECATED` 也能满足 `MaMap::update` |
| 参数只在**表行**内按子串比对 | `verify_docs.py:373,386` | 参数「含义」写错、值写对照样通过 |
| 话题只在反引号 token 内检查 | `verify_docs.py:334` | 正文里裸写 `/cmd_vel` 不会被发现 |
| `claim.unverified_test` 仅 5 短语且被任意标记豁免 | `verify_docs.py:76,366` | 「已跑通测试」不匹配 |
| `doc:` 从不与文件名比对 | 全文无 `fm.get("doc")` | 本次 63 篇恰好都对 |
| `generated_by` / `generated_at` 既不要求也不读取 | `verify_docs.py:49-50` | 契约要求但无校验 |
| 不校验 `end >= start` | `verify_docs.py:308` | `file:100-5` 视为合法区间 |
| 从不读取证据包自身的 `git_rev` | `verify_docs.py:242-272` | 见 S-2 |

---

## 第 3 节：逐模块发现（文档内容层）

> 下列每条均已回源码取证。severity 判据：HIGH = 会误导读者对**真实行为**的理解；
> MEDIUM = 事实性错误或自相矛盾；LOW = 锚点/数字/措辞精度问题。

### 3.1 map（金样例）

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| M-1 | HIGH | `docs/nav/map/接口文档.md:317` | `getMap` = 取**占据矩阵** | 实为 **ESDF 距离缓冲**：`map(x,y)=esdf_buffer_[toAddress(...)]`（`src/map/src/grid_map.cpp:157-165`）；唯一调用者自己命名为 `esdf`（`src/ros2/include/ros2/misc/visualizer.hpp:880`） |
| M-2 | HIGH | `docs/nav/map/流程图.md:123-142` | planner 经 `get_rog_map` → `isLineFree`/`getNearestCellIs` 查询 | `ROGMap::isLineFree`（`rog_map.h:59/66/74`）、`getNearestCellIs/Not`、`getNearestInfCellIs/Not`（`:81/90/99/108`）**全仓库 0 调用点**；`get_rog_map` 唯一调用者为 `visualizer.hpp:1309` |
| M-3 | HIGH | `docs/nav/map/测试要点.md:115`（另 `常见问题FAQ.md:117-119`） | 地图话题发布点在 `src/ros2/src/ros2_node.cpp:71-83` | publisher 全在 `src/ros2/include/ros2/misc/visualizer.hpp:71-83`；`ros2_node.cpp:72` 是注释「// 4. 定时器」，`:73-83` 是三个 `create_wall_timer` |
| M-4 | HIGH | `docs/nav/map/README.md:66-67` | 调用方为「planner」：`get_rog_map` / `get_grid_map` | `src/planner/**` 对 `MaMap` **零引用**；`get_grid_map()` 调用点全在 ros2 层（`ros2_node.cpp:434/515/670/718/794`，并在 `:272/:317` 把栅格作为实参交给 replan） |
| M-5 | MED | `docs/nav/map/配置说明.md:163` | 「同时还有 **112 条** `subnode_reads`」 | `params.json` 实测 **70**；本模块 `待确认清单.md:31/:82` 与 ros2/utils/planner 各文档均写 70 |
| M-6 | MED | `docs/nav/map/测试要点.md:76-77`（R5） | `esdf.resolution >= resolution`「只写在 YAML 注释里」 | 代码**抛异常强制**：`if (prob_map_resolution > temp_counter_map_resolution) throw std::invalid_argument(...)`（`src/map/src/counter_map.cpp:46-50`） |
| M-7 | MED | `docs/nav/map/模块设计说明.md:71-82`、`流程图.md:66` | 遗忘衰减是每帧 `updateProbMap` 的固定末步 | 实为**每 5 次**一次：`if (++forget_skip_counter >= 5) { ...; applyForgetting(); }`（`src/map/src/prob_map.cpp:348-353`）；批量写回亦受 `batch_update_counter >= cfg_.batch_update_size` 门控（`:332-339`） |
| M-8 | MED | `docs/nav/map/测试要点.md:120/123/124/129` | 4 个话题可作为「地图正常更新」的观测量 | 当前配置下**永不发布**：`inf_unk` 受 `unk_inflation_en`（`visualizer.hpp:1327`；`config/map.yaml:6=false`）、`esdf` 受 `esdf_en`（`:1361`；`map.yaml:32=false`）、`frontier` 受 `frontier_extraction_en`（`:1335`；`map.yaml:11=false`）、`surf_map` 唯一发布点所在 `visualizeSurfMap`（`visualizer.hpp:905`）全仓库无调用者 |
| M-9 | MED | `docs/nav/map/接口文档.md:310` | `set_semantics` = 建语义索引 | `grid_map.hpp:65` **仅声明无定义**（`GridMap::set_semantics` 0 命中），调用即链接失败；带参重载在 `:103` |
| M-10 | MED | `docs/nav/map/模块设计说明.md:20` | 「概率值不直接暴露给查询方」 | 存在公开 `double getMapValue(const utils::Vec3f&) const`（`prob_map.h:73`，实现 `prob_map.cpp:414-419` 直返 `occupancy_buffer_`）；同模块 `接口文档.md:129` 自认「取原始概率值」 |
| M-11 | MED | `docs/nav/map/配置说明.md:119`、`常见问题FAQ.md:49-51/61-62` | `ros_callback.cloud_topic/odom_topic` 影响订阅与新鲜度过滤 | 两者只被 `config.hpp:115-116` 读入成员（声明 `:337`），**全仓库无消费者**；新鲜度过滤只用 `odom_timeout`（`ma_map.hpp:109`）；真实话题来自 `ros2.map_topic_name/odom_topic_name`（`config.hpp:320/323` → `ros2_node.cpp:51/58`） |
| M-12 | MED | `docs/nav/map/依赖关系.md:40-42` | PCL 使用点仅为 `Config` 的两个别名 | 还出现在 `rog_map.h:35-36`、`esdf_map.h:70-74`、`esdf_map.cpp:255/273/288/306`、`rog_map.cpp:65`（`pcl::io::loadPCDFile`） |
| M-13 | LOW | `docs/nav/map/配置说明.md:42` | 「本节所有参数读取点都在 `config.hpp:137-212`」 | 同表 esdf/ros_callback/visualization 等行行号为 `81-126`，实际区间 `81-212`，自相矛盾 |
| M-14 | LOW | `docs/nav/map/待确认清单.md:68` | 「代码默认值与 YAML 值共 **30 处**不一致」 | 其引用的配置说明「不一致项」表只有 **25** 行 |
| M-15 | LOW | `docs/nav/map/模块设计说明.md:115` | 讲 `resetCell` 却锚 `sliding_map.h:91` | `:91` 是 `virtual void resetLocalMap() = 0;`，`resetCell` 在 `:101` |
| M-16 | LOW | `docs/nav/map/测试要点.md:110` | 「注释警告内存泄漏」锚 `sliding_map.h:68` | 该警告在 `prob_map.h:170-171` |
| M-17 | LOW | `docs/nav/map/常见问题FAQ.md:62` | 核对 `cloud_topic` 锚 `config/map.yaml:1` | `map.yaml:1` 是 `rog_map:`，`cloud_topic` 在 `:23` |
| M-18 | LOW | `docs/nav/map/测试要点.md:142`、`配置说明.md:122` | `visualization.time_rate` 为 0 时「代码未校验」 | 有显式分支 `if (viz_rate > 0) {...}`（`ros2_node.cpp:442-450`），0 表示不节流 |
| M-19 | LOW | `docs/nav/map/模块设计说明.md:134-138`、`测试要点.md:42-50` | `unfinished_frame_cnt` 疑似数据竞争、【待确认】 | 可判定**无竞争**：两者同处 `map_callback`（`ros2_node.cpp:429-433`），且 map/odom 订阅共享 MutuallyExclusive 的 `map_cb_group_`；真正跨组无锁的是 `grid_map_ptr_`（写 `ma_map.hpp:95`，读 `ros2_node.cpp:272/:317/:452-455`） |
| M-20 | LOW | `docs/nav/map/模块设计说明.md:148`、`依赖关系.md:122` | 「原点由 `fix_map_origin` 固定」 | `map_sliding.enable=true` 时原点随里程计滑动：`if (batch==0 && map_sliding_en && (map_empty_ \|\| (pos-local_map_origin_d_).norm() > map_sliding_thresh)) slideAllMap(pos);`（`prob_map.cpp:321-325`）；`fix_map_origin` 只初始化（`sliding_map.cpp:71`、`rog_map.cpp:36`） |

### 3.2 planner/path_planning

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| P-1 | HIGH | `docs/nav/planner/path_planning/配置说明.md:110-112` | 28 条 `unread_keys`「并非死配置，只因生成证据时没扫 `src/planner`…等纳入扫描后判定」 | 因果错误：这 28 条全部在**已扫描**的 `src/ros2/include/ros2/config.hpp` 中被读取（`:213`、`:278-288`），真因是提取器不跟进矩阵/向量助手函数；扫 `src/planner` 一条也解决不了。且其中 `planner_config.replan_params.min_replan_interval`（`config/planner.yaml:97`）**全仓库无读取点**，是真死配置——与 `docs/nav/ros2/配置说明.md:86-91` 的正确分析矛盾 |
| P-2 | MED | `docs/nav/planner/path_planning/接口文档.md:110` | 「8 邻域栅格 A*，返回**栅格中心**路径」 | A* 以原始点为格心并原样返回：`new Node {start, 0, heuristic(start, goal)}`（`astar.cpp:37`）、`current->position + Vector2d(dx,dy)*resolution`（`:63`）、`reconstruct_path` 直返（`:102-110`）；发栅格中心的是 JPS（`jps.cpp:183-185`） |
| P-3 | LOW | `docs/nav/planner/path_planning/模块设计说明.md:55-56/91-92` | 「调用方依次注入地图、参数、速度」 | `plan()` 只调 `set_map(:60)`、`set_velocity(:72)`，**从不调 `set_param`**；参数经 `fsm_replanner.h:45` 由 `ros2_node.cpp:36` 一次性注入（该模块 `README.md:65` 写的是对的） |
| P-4 | LOW | `docs/nav/planner/path_planning/常见问题FAQ.md:59-61` | Q4 的失败原因只引 A* | 生效路径是 JPS，对应 `jps.cpp:202-209`（起终点碰撞）、`:277-281`（超时）、`:345-354`（无路径） |
| P-5 | LOW | `docs/nav/planner/path_planning/配置说明.md:49` | `fold_time` 是「折叠准备距离公式的第一项」 | 它是第一项的**因子**（`post_processing.cpp:406`）；且漏记其 ros2 消费方（`ros2_node.cpp:318-320`） |
| P-6 | LOW | `docs/nav/planner/path_planning/接口文档.md:192` 等 | — | 13 个 inventory 符号未文档化：`DenseSample`/`TunnelInterval`/`PathState`/`TimedTrajectoryPoint` 的全部字段、`AStar::Node::operator>`（`astar.h:53`）；AStar/PathPostProcessing 段缺模板基类行 |

### 3.3 planner/traj_optimize

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| T-1 | HIGH | `docs/nav/planner/traj_optimize/配置说明.md:44` | `safe_distance` 与四个 yaw 参数用于「代价评估**与输出后校验**」 | 唯一后校验 `check_trajectory_collision` **无任何调用点**（`traj_optimizer.h:38` 声明、`:145` 定义即全部出现处），且**不含 yaw 形参**；`safe_distance` 运行时只进积分代价（`:295`、`:357`）。本模块 `接口文档.md:128`、`常见问题FAQ.md:90`、`测试要点.md:50` 三处写的才是对的 |
| T-2 | MED | `docs/nav/planner/traj_optimize/常见问题FAQ.md:59` | gcopter 重载「唯一调用处已被注释，见 `ros2_node.cpp:306`」 | `:306` 是 `//visualizer.visualize(result->ma_spline_traj, route);`，实参为 `MAsplineOutput`，对应 `visualizer.hpp:337` 重载；gcopter 的 `visualize(const Trajectory<D,2>&,...)`（`visualizer.hpp:349`）**无任何调用点（含注释）** |
| T-3 | LOW | `docs/nav/planner/traj_optimize/模块设计说明.md:24-27` | 需求表 4 行锚 `problem.md:16-20/12-15/21/23` | 实际：折叠控制句在 `:21`、折叠耗时在 `:17`、优化目标在 `:23`、时间自然分配在 `:25`；第 3/4 行互相错位 |
| T-4 | LOW | `docs/nav/planner/traj_optimize/常见问题FAQ.md:52`、`待确认清单.md:48`、`README.md:51-52`、`依赖关系.md:110/117/133`、`接口文档.md:208-209` | 引 `opt.md:1-4` / `:2` / `:4` 支撑 gcopter「未使用」 | `opt.md` 共 5 行，`:2`/`:4` 是**空行**，gcopter 句在 `:5`、spline_opt 句在 `:3` |
| T-5 | LOW | `docs/nav/planner/traj_optimize/流程图.md:77` | 代价组装含「并行步数」 | 字段为 `integral_num_steps`（积分采样数），设定点在 `traj_optimizer.h:271`，在所述 `284-301` 之外；消费点 `SplineOptimizer.hpp:2474` |
| T-6 | LOW | `docs/nav/planner/traj_optimize/README.md:80` | 「17 个文件全部列出」 | 表 `:85-102` 实为 18 行（17 + `CMakeLists.txt`）；模块内 4 个 `.md` 证据文件（`opt.md`、`problem.md`、2× `SplineOptimizerProtocols*`）不在任何文件表 |
| T-7 | LOW | `docs/nav/planner/traj_optimize/README.md:99`、`流程图.md:44`、`接口文档.md:167` | `minco.hpp:43` = Minco S3/S4；`SplineOptimizer.hpp:1786` = `synchronizeWorkingState` | `minco.hpp:43` 是 `class BandedSystem`（MINCO 在 `:202`/`:751`）；`:1786` 是 `__attribute__((noinline))`，声明在 `:1788` |

### 3.4 planner/controller

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| C-1 | MED | `docs/nav/planner/controller/接口文档.md:51`、`模块设计说明.md:97` | `Rd`「Hessian 只用对角线；**线性项用整矩阵**」⇒ 非对角项经线性项生效 | `g_` 只由 `Q/QN`（`mpc.h:327`）与 `R`（`:333`）构成，**`Rd` 没有线性项**；`Rd` 只出现在 `h_triplets` 的对角块（`:202-205`），故 `Rd` 非对角项**完全无效**。原句对 Q/QN/R 成立、对 `Rd` 不成立 |
| C-2 | MED | `docs/nav/planner/controller/README.md:26`（另 `流程图.md:48`） | 模块「追踪成**底盘指令**…输出加速度指令 (ax, ay)」 | 模块确实产出 `u_cmd`（`mpc.h:490`），但唯一调用方**丢弃** `u_cmd`/`predicted_inputs`，实际由 `predicted_states[1]` 速度组 `Twist` 下发（`src/ros2/src/ros2_node.cpp:219-225`）；本模块 FAQ/测试要点写的是对的，README 与之矛盾 |
| C-3 | LOW | `docs/nav/planner/controller/流程图.md:103-105` | 「本模块的参数在构造后确实不再更新」 | 每次成功求解都写回：`param_.u_prev = u_cmd;`（`mpc.h:525`）；同模块 `模块设计说明.md:89` 也这么写 |
| C-4 | LOW | `docs/nav/planner/controller/模块设计说明.md:26` vs `:98` | 同一「写而不读」结论，一处标【待确认】、一处标【事实】 | 可判定：全仓库 `u_prev` 仅 2 处（`mpc.h:34` 声明、`:525` 赋值），**无读取** ⇒ 应为【事实】 |
| C-5 | LOW | `docs/nav/planner/controller/接口文档.md:72` | 3 参 `solve`「只返回当前控制量的简化重载」未标调用状态 | 3 参重载（`mpc.h:50`，定义 `:389`）在 `:393` 直接转发给 5 参版本；仓库内唯一外部调用是 5 参（`ros2_node.cpp:179`）⇒ 属未使用的公开 API |

### 3.5 planner/replan

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| R-1 | HIGH | `docs/nav/planner/replan/模块设计说明.md:25` | 「【事实】2D 模型 yaw 不作为优化变量（**沿用参考航向**）」 | 2D 路径把 yaw **置零**：`ma_intput.model=OMNI_XY`（`fsm_replanner.cpp:97`）→ `optimize_xy` → `out.trajectory = extend_xy_with_zero_yaw(xy_spline)`（`traj_optimizer.h:253`）→ `xy_yaw_points.col(2).setZero()`（`:127`），边界 yaw 速率亦置零（`:131-134`）；输入 yaw 被丢弃。与本仓 `docs/nav/planner/traj_optimize/模块设计说明.md:28`「2D 模式把 yaw 置零扩展」直接矛盾 |
| R-2 | LOW | `docs/nav/planner/replan/流程图.md:62`、`常见问题FAQ.md:109-110`、`接口文档.md:97` | 横向偏差参考「优先用上次优化路径」 | `last_opt_path_` **从未被赋值**（仅 `fsm_replanner.h:65` 声明、`fsm_replanner.cpp:51` 读取），`.empty()` 恒真 ⇒ 该分支不可达，应列入死代码 |
| R-3 | LOW | `docs/nav/planner/replan/接口文档.md:38`、`模块设计说明.md:112`、`待确认清单.md:75` | `replan_param` 为正常对外参数成员；死代码「三处」/7 条 | `} replan_param;`（`fsm_replanner.h:34`）**全仓库无引用**；生效存储是 `planner_config_.replan_params`（写 `config.hpp:259`，读 `fsm_replanner.cpp:21/36/54`）⇒ 应计入死代码，且两处死代码计数互相矛盾 |
| R-4 | LOW | `docs/nav/planner/replan/README.md:33` | 「对外只暴露一个入口 `plan`」 | `one_plan` 为 public（`fsm_replanner.h:89-90`）且被 `接口文档.md:77` 列为公开方法；应为「唯一**生效**入口」 |
| R-5 | LOW | `docs/nav/planner/replan/模块设计说明.md:63-64`、`流程图.md:123`、`测试要点.md:55` | 「原样返回上一次的结果结构（`cpp:112`）」 | 每次调用开头 `result_.is_new_trajectory = false;`（`:18`）⇒ 并非原样，标志已被清除（这正是不重复提交旧轨迹的原因） |
| R-6 | LOW | `docs/nav/planner/replan/流程图.md:108` | 「三个条件都不成立」⇒ 沿用旧轨迹 | `need_replan_` 初值为 true（`h:63`）且规划失败后仍为 true（`cpp:78`，仅成功时由 `:109` 清除）⇒ 全不成立时仍可进入重规划分支（`:58`）；同文件主图 `:28` 与风险 5、FAQ Q2 表述正确 |
| R-7 | LOW | `docs/nav/planner/replan/测试要点.md:25` | 「失败/成功日志走重规划通道」 | 活代码在该通道只有两条**失败**日志（`cpp:77`、`:102`），无成功日志；其余在无调用点的 `one_plan` 内，`:86` 已注释 |

### 3.6 ros2

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| S-1 | HIGH | `docs/nav/ros2/模块设计说明.md:93-95`（另 `:110`、`流程图.md:63`） | 「【事实】节点是**多线程**的…ROS 执行器会在不同线程调度它们」；「组内串行、组间并发」 | 实为**单线程**：`rclcpp::WallRate rate(1000); while (...) { rclcpp::spin_some(nh); ... }`（`src/ma_nav.cpp:15-19`）；`spin_some(Node::SharedPtr)` 使用 `SingleThreadedExecutor`；全 `src/` 无 `MultiThreadedExecutor`、无 `std::thread`。整个「线程与并发」章节建立在错误前提上 |
| S-2 | HIGH | `docs/nav/ros2/流程图.md:169`、`常见问题FAQ.md:52-53` | 下发新轨迹时「跳过已过去事件并**补发**当前应有状态」 | 补发调用**被注释**：`if (desired_fold_state != current_fold_state_) { //publish_fold_cmd(desired_fold_state); current_fold_state_ = desired_fold_state; }`（`ros2_node.cpp:334-337`）⇒ 只更新记账变量，指令与内部状态静默发散 |
| S-3 | MED | `docs/nav/ros2/模块设计说明.md:101-104` | 锁「实际靠回调组串行化；主要保护点选区域容器，其中一处顺带读了一次栅格地图」 | 锁内实际包着 `grid_map->semantics_polygon_region(...)`（`ros2_node.cpp:513-523`），而该调用**会写地图**（`grid_map.hpp:122-134` 内 `set_semantics`）；`:793-831` 同样在锁内写。未加锁的反而是 `clicked_points_.push_back`（`:494`）与 `clicked_regions_` 写入（`:545-547`） |
| S-4 | MED | `docs/nav/ros2/模块设计说明.md:24` | 「规划、控制、可视化各自独立节拍，**互不阻塞**」 | `controller_timer_` 与 `pub_viz_timer_` 绑定**同一个** `control_cb_group_`（`ros2_node.cpp:79-89`，注释「可视化放控制组（轻量）」）⇒ 二者互斥；本文件 `:87-89` 自相矛盾 |
| S-5 | MED | `docs/nav/ros2/README.md:43-44` | 「**所有** `/ma_nav/*` 可视化话题都在 Visualizer 创建」 | `/ma_nav/clicked_regions` 在节点构造函数创建（`ros2_node.cpp:99`）；本模块 `接口文档.md:37` 自己列为节点侧 |
| S-6 | MED | `docs/nav/ros2/接口文档.md:150` | `controller_callback`：MPC 求解 → … → **限幅** → 发布 | 越界/NaN 时**告警并丢弃本帧、不下发**（`ros2_node.cpp:229-240`）；本模块 `测试要点.md:61`、`常见问题FAQ.md:22-23` 写的是对的 |
| S-7 | MED | `docs/nav/ros2/接口文档.md:15`、`README.md:17` | 「**189 条**符号」 | `docs/_evidence/ros2/inventory.json` 现为 **191**（5 文件、0 解析失败）；文档 `generated_at=06:00Z` 早于证据重跑 `14:35Z` |
| S-8 | LOW | `docs/nav/ros2/待确认清单.md:21` | 70 条上下文未知读取「**全部位于本模块的装载函数里**」并锚 `config.hpp:169-288` | 其中 5 条在 `Config` **构造函数**内、位于所述区间之外：`:334/338/339/342/343`（`config["map"]`、`config["mpc_param"]` 等） |
| S-9 | LOW | `docs/nav/ros2/模块设计说明.md:87-89` | 【推断】把可视化与控制同组的动机是二者「只读共享状态」 | `controller_callback` 会写共享状态：`t_track_`（`:140`）、`last_vx/vy_world_cmd_`、`has_last_mpc_cmd_`（`:187-189`）、`next_fold_event_idx_`（`:215`）、`current_fold_state_`（`:201/:337`）；代码注释给的动机是「轻量」 |
| S-10 | LOW | `docs/nav/ros2/接口文档.md:109` | `load_matrix4d`「16 元素按行、4 元素对角」 | 另有 4×4 **嵌套**分支（`config.hpp:139-153`）；同表对 `load_matrix3d`/`load_matrix2d` 都列了嵌套形式，口径不一致 |

### 3.7 utils

| ID | Sev | 位置 | 文档断言 | 代码事实 |
|---|---|---|---|---|
| U-1 | HIGH | `docs/nav/utils/接口文档.md:65`、`模块设计说明.md:37` | `utils::get_current_dir_name` = **返回当前工作目录名**，用于路径拼接 | 实为 **Unix 纪元秒（double）**：`return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();`（`scope_timer.hpp:35-36`）；消费方按时间戳用：`occ_timestamp_buffer_.resize(map_size, utils::get_current_dir_name())`（`prob_map.cpp:92`）、`:591`、`:832`。函数名沿用 ROG-Map 上游 |
| U-2 | MED | `docs/nav/utils/接口文档.md:67`、`流程图.md:107` | `TimeConsuming` 有「三种构造形式」含无参构造 | `TimeConsuming();`（`scope_timer.hpp:42`）**仅声明、全仓库无定义**（定义的只有 `:44`、`:53`），使用即链接失败；clangd 把声明记入 inventory，故校验器不报错 |
| U-3 | MED | `docs/nav/utils/接口文档.md:61-74` | 计时器接口仅列 ctors / `set_enbale` / `start` | 漏记 public `double stop()`（`scope_timer.hpp:90`），它被大量调用（`prob_map.cpp:330/337/341`、`inf_map.cpp:218/252`、`esdf_map.cpp:237/244`、`rog_map.cpp:258`），且 `stop()` 会置 `has_shown=true`（`:92`）使析构不再上报——该语义与成员 `has_shown`（`:118`）均未记录 |
| U-4 | MED | `docs/nav/utils/测试要点.md:52` | 「路径键最后一级是标量 ⇒ 返回空节点」 | `yaml_loader.hpp:139` 的 `if (node.IsScalar())` 判的是**父节点**（循环已消费中间段）；对 `a/b`（b 为标量）会在 `:142-145` 正常返回标量。真正被拦的是「路径中间节点自身是标量」 |
| U-5 | MED | `docs/nav/utils/依赖关系.md:79` | `planner/traj_optimize` 依赖本模块「计时」 | `grep -rn "TimeConsuming\|scope_timer" src/planner/traj_optimize/` 为空；真正使用者是 replan（`fsm_replanner.cpp:59/119/149`），而该表 replan 行（`:81`）没写；`README.md:68` 同表只写「L-BFGS 与 expected」，两篇矛盾 |
| U-6 | MED | `docs/nav/utils/接口文档.md:168` | 「日志/计时/装载三个头文件有中文行内注释」 | `scope_timer.hpp` 非 ASCII 行数为 **0**（`logger.hpp` 3、`yaml_loader.hpp` 2、`type_utils.hpp` 6） |
| U-7 | LOW | `docs/nav/utils/模块设计说明.md:99`、`常见问题FAQ.md:63`、`测试要点.md:32`、`待确认清单.md:24` | 「装载 N 个参数就是 N 次文件读取」 | `yaml_loader.hpp:49` 无条件 `YAML::LoadFile`，含斜杠的路径键再走 `:126` 第二次 ⇒ 路径键为 **2N** 次；唯一消费方 map 全用路径键 |
| U-8 | LOW | `docs/nav/utils/依赖关系.md:84-87` | `plotter.hpp`「**唯一**引用链」是节点头 include + 一个成员 | 还有 `src/ros2/src/ros2_node.cpp:7` 的 include（成员 `plotter_` 确无使用点，这部分正确） |
| U-9 | LOW | `docs/nav/utils/接口文档.md:133` | `MatrixFormatter`「支持精度**与对齐**控制」 | `fmt_eigen.hpp:34-81` 只解析 `.N` 精度（`:46-50`）与空格拼接，无任何对齐/宽度/填充逻辑 |
| U-10 | LOW | `docs/nav/utils/依赖关系.md:66`、`README.md:72`、`依赖关系.md:25/95`、`配置说明.md:50/62` | 锚 `scope_timer.hpp:2`、`CMakeLists.txt:66`、`CMakeLists.txt:32` | 分别应为 `scope_timer.hpp:30`（`:2` 是 license 注释）、根 `CMakeLists.txt:66`（HEAD 正确，工作区偏 1，见 S-1）、根 `CMakeLists.txt:32`（HEAD 正确） |
| U-11 | LOW | `docs/nav/utils/接口文档.md:122-123`、`配置说明.md:39` | 用 `yaml_loader.hpp:85-87` 支撑「命中→绿色成功」 | 那是**标量重载**的分支；向量重载的绿色分支在 `:66-68` |
| U-12 | LOW | `docs/nav/utils/接口文档.md:49` | `bt_replan` 通道 = 「行为树重规划」（未标死代码） | 全仓库仅 `logger.hpp:119` 定义、零引用（对比 `fsm_replan` 7、`planning` 11、`ros2` 41、`controller` 13）；`src/` 下无行为树模块。文档对 matchit/plotter 都标了「无人使用」，唯独此通道被当成在用 |
| U-13 | LOW | `docs/nav/utils/接口文档.md:24-27`、`模块设计说明.md:34`、`流程图.md:49` | 日志包装的调用形态是「格式串 + 参数」 | 首参是必填 `const std::shared_ptr<spdlog::logger>&`（`logger.hpp:14`），真实写法如 `logger::warn(logger::traj_opt, "...")`（`traj_optimizer.h:206`）；照文档写 `logger::info("x={}",1)` 无法编译 |
| U-14 | LOW | `docs/nav/utils/README.md:24` | 「另含**两个**第三方单头（L-BFGS、tl::expected）」 | 同篇 `:42/:87` 与 `接口文档.md:146`、`依赖关系.md:70` 都把 `matchit.h` 列为第三个 ⇒ 应为三个 |
| U-15 | LOW | `docs/nav/utils/流程图.md:15/31-40` | 「节点表逐条给出锚点」 | 缺 `path_planning→utils`、`controller→utils` 两行；图含 `fmt` 但边表无 `fmt` 边，且漏 `nlohmann/json` |
| U-16 | LOW | `docs/nav/utils/模块设计说明.md:21` | 「【事实】只依赖 Eigen / spdlog / yaml-cpp 与标准库」 | `fmt_eigen.hpp:27-28` 依赖 fmt、`plotter.hpp:4-8` 依赖 nlohmann/json 与 POSIX socket；本文档风险 3/4 与 `依赖关系.md:46-48` 自认这些依赖 |

---

## 第 4 节：文档系统层发现（记录 / 契约 / 工具）

| ID | Sev | 发现 | 证据 |
|---|---|---|---|
| D-1 | HIGH | **记录中存在被证伪的「零改动」声明**。`20261002_nav_ros2_docs.md:215` 与 `20261002_nav_utils_docs.md:31` 写「`src/`、`config/`、`launch/`、**`CMakeLists.txt` 零改动**」；`..._utils_docs.md:161` 写「改动全部落在 `docs/` 与授权的一行源码注释」 | `git status --porcelain` → ` M CMakeLists.txt`；该文件 mtime `22:12` 早于两份记录（`22:41` / `23:29`）；契约 §5.6 禁止改任何 `CMakeLists.txt` |
| D-2 | HIGH | **「人工审核闸门」可被机械绕过**，与设计承诺矛盾 | `verify_docs.py:281` 的拒绝名单为 `("", "TODO", "待填", "unknown")`，**不含 `未审核`**；另一道门只有 `status != "reviewed"`。63 篇均写 `reviewer: 未审核`，校验器 0 条 reviewer 警告。把 `status` 改为 `reviewed` 即得 `errors=0 warnings=0` → PASS，推翻 `20261002_module_doc_anti_hallucination_pipeline.md:165-166`「AI 无法自行绕过得到 PASS」 |
| D-3 | MED | **9 个模板全部漏 `worktree_dirty`**；而 `FRONT_MATTER_REQUIRED` 含该字段（属 error） | `docs/nav/example/README.md:66-78` 列 9 字段；`docs/nav/example/模板/*.md` 每篇仅 8 行字段（`grep -c '^worktree_dirty:'` = 0）；`verify_docs.py:49-50`。照模板抄必然校验失败 |
| D-4 | MED | **5 篇记录与 1 篇模块文档仍写着已不存在的「当前一致性」**（5 模块 / 45 篇 / 摘要 `7c453c4e8af72579` / `worktree_dirty: true`） | `20261002_module_doc_anti_hallucination_pipeline.md:331-332/342-343`，同样表述见 path_planning:344-345、traj_optimize:294-295、controller:263-264、replan:252-253；`docs/nav/planner/path_planning/待确认清单.md:84` 亦如此（且声明的是 map 的字段）。实际为 63 篇 / `be346b5265f1410d` / `false`；仅 `20261002_nav_utils_docs.md:249-250` 正确 |
| D-5 | MED | **标注契约只兑现约 10%**：2937 行带锚点，仅 301 行（10.2%）同时带三标记之一；`map/README.md`、`map/流程图.md`、`utils/流程图.md` **零标记**；且校验器**从不检查 `【事实】`** | 本次实测计数；`verify_docs.py:77` 仅含 `【推断】/【待确认】/待确认`，全文无「事实」 |
| D-6 | LOW | traj_optimize 记录的聚合行数不可复现：`20261002_nav_planner_traj_optimize_docs.md:41` 写「17 个文件 / **18228 行**」 | 该 17 个文件在 `inventory.json` 中合计 **17558** 逻辑行 |
| D-7 | LOW | 记录把校验器写成 496 行 | `20261002_nav_planner_path_planning_docs.md:67`；实际 `wc -l` = **497** |
| D-8 | LOW | `subnode_reads` 对账不平：planner 四篇共认领 16+22+10+9 = **57**，ros2 称全部 **70**，相差 13 条（`node["stage1"]`、`config["map"]` 等中间层级读取）无人解释 | `docs/nav/planner/*/待确认清单.md:30/28/27/28`、`docs/nav/ros2/待确认清单.md:28`、`params.json` |
| D-9 | LOW | **契约自身的 front-matter 示例是过期的** | `docs/nav/example/README.md:70-71` 仍为 `2eed7d3e884d` / `1ab15719a7937f37`，而 63 篇实际为 `8f98daee5bc7` / `be346b5265f1410d`；`20261002_module_doc_anti_hallucination_pipeline.md:268` 称已同步 |

---

## 第 5 节：被否定的误报（务必保留，避免误修）

本次共剔除 **8 条不成立的上报**：

| # | 曾报为 | 复核结论 |
|---|---|---|
| 1 | replan `依赖关系.md:26` 的 `CMakeLists.txt:93`「指向 traj_optimize」 | **正确**。`git show HEAD:CMakeLists.txt` 的 `:93` 即 `replan`；报错源于按脏工作区编号比对 |
| 2 | replan `测试要点.md:23` 的 `CMakeLists.txt:76`「指向注释」 | **正确**。HEAD `:76` 即 `ament_auto_add_executable(ma_nav src/ma_nav.cpp)` |
| 3 | utils `配置说明.md:50/62` 的 `CMakeLists.txt:32` | **正确**。HEAD `:32` 即 `add_definitions(-DROOT_DIR=...)` |
| 4 | utils `README.md:72` 等 `CMakeLists.txt:66` | **正确**。HEAD `:66` 即 `add_subdirectory(src/utils)` |
| 5 | （父代理最初判断）ros2 `README.md:77` 的 `CMakeLists.txt:94` 偏一行 | **正确**。HEAD `:94` 即 `ros2` |
| 6 | traj_optimize 逐类符号数 `456/258/82/2/73/48/23`、`798`、`144` | **全部精确吻合** inventory 逐文件计数（`spline_opt` 合计 798、`gcopter` 三头 144） |
| 7 | utils §二/§六 的 `28/16/11/48/418/391` 等数字 | **全部精确吻合** |
| 8 | 「7 个模块文档都没有使用【事实】/【推断】/【待确认】标注」 | **不成立**，系计数脚本在只读文件系统上 herestring 失败的假象；改用 Python 重算后确认全模块都在用（275/227/56），该结论已撤销 |

**结论**：第 1–5 条属同一类陷阱 —— **拿当前脏工作区行号去比文档，而契约 §3 定义的行号是相对声明的 `git_rev`**。
根因是 S-1 的根 `CMakeLists.txt` 未提交改动。

---

## Auditor Review

### Checks performed

- [x] 关键路径检查：7 模块调用链、数据流、继承关系逐条对源码核过
- [x] 机器层复跑：`verify_docs.py` ×7（`errors=0`，`warnings=9` 全为 `review.pending`）
- [x] 覆盖率独立重算：`find src/<module>` 与各 `inventory.json` 的 `files` 完全一致；文档提及率 0 缺失
- [x] 锚点审计：**按 `HEAD` 复核** 31 个根 `CMakeLists.txt` 锚点（不是按工作区）
- [x] 证据一致性：`173 yaml_leaves / 61 bindings / 70 subnode_reads / 4 unresolved_paths / 28 unread_keys / 34 interfaces` 与文档表述对齐
- [x] 结构/时效：63 篇 front-matter 9 字段齐全、`doc:` 与文件名一致；现算 `evidence_digest` 与文档一致
- [x] diff / 范围检查：`git status --porcelain` 全量分类，确认 `src/`、`config/`、`launch/` 无未提交改动
- [x] 工具实现检查：逐条读 `verify_docs.py`（497 行）判定盲区并实地构造反例
- [x] 记录准确性检查：定量声明逐条对现状核对
- [ ] **注入测试：未执行** —— `test_injection.py` 需写 `docs/tools/_injection_tmp/`，当前文件策略为只读，报 `Read-only file system`；其判据仅静态阅读
- [ ] **未执行构建**：AGENTS.md 禁止在未获用户明确许可前运行构建命令

### Issues found

见第 2、3、4 节。合计：

| 层 | HIGH | MEDIUM | LOW | 小计 |
|---|---|---|---|---|
| 文档内容（7 模块） | 9 | 18 | 约 25 | 约 52 |
| 文档系统（记录/契约/工具） | 2 | 3 | 4 | 9 |
| **合计** | **11** | **21** | **约 29** | **约 61** |
| 已剔除误报 | — | — | 8 | 8 |

按模块分布（文档内容）：map 20、utils 16、ros2 10、replan 7、traj_optimize 7、path_planning 6、controller 5。

### Final result

**NEEDS_FIX**

判据：

1. 机器可验证层全部通过（`errors=0`、覆盖率全满、front-matter 齐全、证据计数一致）；
2. 但**语义层在 7/7 模块均有实质错误**，其中 `map`（金样例）4 条 HIGH；
3. **防幻觉链路的闭环没有真正闭上**：校验器有 9 类已实地举例的盲区；「人工审核闸门」可被 `status: reviewed` 一行绕过（D-2）；标注契约只兑现约 10%（D-5）；
4. **记录层自身漂移**：存在被证伪的「零改动」声明（D-1）与过期的「当前一致性」（D-4）；
5. 按 `docs/nav/example/README.md` §6 的验收标准，当前 63 篇**不具备人工签核条件**，`status` 应保持 `draft`。

---

## 未解决问题 / 修复建议（未执行）

### A. 文档内容（范围仅 `docs/nav/`，不碰 `src/`、`config/`、`launch/`）

1. 先修 9 条内容层 HIGH：M-1~M-4、S-1、S-2、T-1、R-1、U-1。
2. 再修 18 条 MEDIUM（含 3 条「同模块自相矛盾」：S-4、S-6、U-5）。
3. LOW 按模块批量处理，重点是**指向注释/相邻重载/同名符号**的锚点。

### B. 文档系统（属 `docs/tools/`、`docs/nav/example/`，需单独授权）

1. `docs/nav/example/模板/*.md` ×9 补 `worktree_dirty`（D-3）。
2. 契约示例与「当前一致性」叙述刷新（D-9、D-4）。
3. 记录追加更正段，或建立**唯一**的基线快照文件（D-4）。
4. 若要真正闭环（D-2、S-4、D-6 相关）：
   - `reviewer` 改为**白名单**校验（拒绝 `未审核`）；
   - `FRONT_MATTER_REQUIRED` 补 `generated_by` / `generated_at`；
   - `worktree_dirty` 探测范围加入根 `CMakeLists.txt`；
   - 校验器增加 `end >= start`、`doc:` 与文件名比对；
   - 证据包自身的 `git_rev` 与文档声明做一致性比对。

### C. 需用户决策

1. 根 `CMakeLists.txt` 的未提交改动（D-1/S-1）：**提交、还原，还是保留**？在决定前，所有 `CMakeLists.txt:NN` 锚点在当前工作区均不可直接引用。
2. 是否授权修改 `docs/tools/`（校验器/模板/契约）。

---

## 交付说明

- 本次为**只读审核**：`docs/nav/`、`docs/tools/`、`src/`、`config/`、`launch/` 均**未修改**，
  仅新增本记录文件。
- **未执行构建**：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
- 未执行任何提交类操作（无 `git add/commit/push/tag/merge`），仅使用只读的
  `git status` / `git rev-parse` / `git show` / `git diff`。
