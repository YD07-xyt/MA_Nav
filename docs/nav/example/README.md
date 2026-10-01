# example —— 模块文档生成规范（Normative）

> 本目录定义「AI 读项目 → 生成模块文档」的**强制契约**。
> AI 在生成任何模块下的文档时，必须遵守本文件的规范，并参照 `模板/` 与金样例。
>
> 本目录只放**规范**与**模板**，不放文档副本。金样例是**唯一一份**真实文档：
> [`../map/README.md`](../map/README.md)（见 §7「金样例单一 owner」）。

---

## 0. 三层结构与角色

| 层 | 位置 | 角色 | 是否强制 |
|---|---|---|---|
| 规范 | `example/README.md`（本文件） | 契约：输入、输出、证据、校验、验收 | 强制 |
| 模板 | `example/模板/*.md` | 只有骨架与占位符，**没有内容** | 强制照抄结构 |
| 金样例 | `docs/nav/map/` | 已通过校验与人工审核的真实文档 | 强制对齐风格与粒度 |

**不要把模板当样例，也不要把金样例当模板。** 模板回答「有哪些表格」，
金样例回答「每条事实要写到什么粒度」。

---

## 1. 输入契约：只能基于 Evidence Pack

生成模块文档时，AI 的输入是**证据包**，不是整个仓库：

```
docs/_evidence/
  <module>/inventory.json   模块符号表（来自 clangd documentSymbol）
  repo/ros2.json            ROS2 接口面（话题/服务/定时器/ROS 参数）+ 话题名解析结果
  repo/params.json          YAML 叶子键（值+行号）、代码绑定、代码默认值、缺口
  repo/launch.json          launch 解析结果（实际启动的节点与参数）
```

生成前先运行 `docs/tools/` 下的提取器；生成时**必须**引用证据里的解析值，
不允许凭记忆或推测填写接口名、参数名、参数值。

**禁止**在未运行提取器的情况下直接翻仓库写文档 —— 那样无法区分
「注释里的旧值」与「当前生效的真值」，本仓库 `config/planner.yaml` 的
`cmd_vel_name: "/cmd_vel_chassis" #"/cmd_vel"` 就是现成的反例。

---

## 2. 输出契约

每个模块在 `docs/nav/<module>/` 下产出 **9 个文件**，文件名固定：

| # | 文件 | 内容 |
|---|---|---|
| 1 | `README.md` | 模块概述：背景、技术栈、模块、职责 |
| 2 | `模块设计说明.md` | 设计意图、类协作、数据流、关键决策 |
| 3 | `接口文档.md` | 文件/类/函数/成员变量表，与 Doxygen 标签对齐 |
| 4 | `依赖关系.md` | 模块内依赖、跨模块依赖、外部库依赖 |
| 5 | `流程图.md` | Mermaid 流程图 |
| 6 | `配置说明.md` | 参数表：YAML 值、代码默认值、读取位置 |
| 7 | `测试要点.md` | 高风险点与测试建议 |
| 8 | `常见问题FAQ.md` | 常见问题 |
| 9 | `待确认清单.md` | 不确定项、证据缺口 |

> 原 `example/` 里的文件名 `常见问题 FAQ.md` 含空格，本规范统一为
> `常见问题FAQ.md`（无空格），模块文档与模板保持一致。

### 2.1 front-matter（每个文件都必须有）

```yaml
---
module: map
doc: 配置说明
git_rev: 2eed7d3e884d
evidence_digest: 1ab15719a7937f37
worktree_dirty: false  # src/config/launch 下是否存在未提交改动（校验器会交叉核对）
generated_by: dsh + docs/tools
generated_at: 2026-10-02T00:00:00Z
status: draft          # draft | reviewed
reviewer: 待填          # 人工审核者；留空会被校验器判为 NEEDS_FIX
---
```

`evidence_digest` 由 `evidence_common.evidence_digest()` 计算。校验器会比对
当前证据摘要与文档记录值：不一致即判 `evidence.stale`（NEEDS_FIX），
提示该文档需要重新生成。**这是防止文档随时间漂移成幻觉的机制。**

`worktree_dirty` 必须如实填写：证据是从**工作区**提取的，而 `git_rev` 只是
HEAD 提交号。若源码有未提交改动而文档写 `worktree_dirty: false`，
校验器会判 `evidence.worktree_mismatch`（FAIL），因为那等于声称文档描述的是
一个干净的提交，而事实不是。

---

## 3. 证据契约：三段式 + 锚点

文档里每一句**事实性陈述**必须落入以下三类之一：

| 标注 | 含义 | 强制要求 |
|---|---|---|
| 【事实】 | 可从证据直接读出 | 必须带锚点 `相对路径:行号`，且该行附近真的出现被引用的符号 |
| 【推断】 | 由事实推导，但代码没有直说 | 必须显式写 `【推断】` 并给出推导依据（引用支撑它的事实锚点） |
| 【待确认】 | 证据不足或两侧对不上 | 必须显式写 `【待确认】`，并进入 `待确认清单.md` |

锚点格式（**唯一允许的格式**）：

```
src/map/src/rog_map.cpp:188
src/map/include/map/3d_occ_map/rog_map.h:133-160
config/map.yaml:4
```

- 路径一律相对仓库根
- 行号必须是当前 git revision 下的真实行号
- 一个锚点只能支撑它附近（默认 ±6 行）真实存在的符号

**违规示例**（会被校验器拦下）：

| 写法 | 触发的检查 | 原因 |
|---|---|---|
| 话题写 `/cmd_vel` | `ros2.unknown_topic` | 证据里解析值是 `/cmd_vel_chassis`；`/cmd_vel` 只存在于注释 |
| `src/map/src/rog_map.cpp:9999` | `anchor.line_out_of_range` | 文件只有 272 行 |
| `MaMap::update()` | `symbol.unknown` | 符号表中不存在 |
| 参数表写 `resolution = 0.2` | `param.value_mismatch` | YAML 真值为 `0.1` |
| 小节里只有断言没有锚点 | `annotation.unsupported_section` | 无法机器验证的内容必须降级标注 |
| 写「已测试通过」 | `claim.unverified_test` | 本仓库 `src/` 下没有任何测试 |

---

## 4. 校验契约

生成完成后必须运行：

```bash
python3 docs/tools/verify_docs.py --root . --module <module> \
    --docs docs/nav/<module> --evidence docs/_evidence \
    --report docs/_evidence/verify.<module>.json
```

退出码：`0=PASS`，`2=NEEDS_FIX`，`1=FAIL`。

校验器做三层**机器可验证**的比对：

| 层 | 检查 ID | 判据 |
|---|---|---|
| L1 存在性 | `anchor.file_missing` `anchor.line_out_of_range` | 锚点文件存在、行号在范围内 |
| L1 存在性 | `anchor.unsupported` | 锚点附近真的出现该行引用的符号 |
| L1 存在性 | `symbol.unknown` | 反引号里的 `::` 符号存在于 inventory |
| L2 一致性 | `ros2.unknown_topic` `ros2.unknown_type` | 话题名/消息类型是**解析后**的真值 |
| L2 一致性 | `param.value_mismatch` `param.unknown_key` | 参数值等于 YAML 真值，键路径存在 |
| L2 一致性 | `claim.unverified_test` | 无测试仓库禁止出现「已验证」类断言 |
| L3 覆盖率 | `coverage.missing_file` | 模块每个源文件都被文档提及 |
| L3 覆盖率 | `structure.missing_doc` `structure.front_matter` | 9 个文件齐全、front-matter 完整 |
| 结构 | `annotation.unsupported_section` `structure.mermaid` | 每节有锚点或标注、Mermaid 首行合法 |
| 时效 | `evidence.stale` `review.pending` | 证据摘要/git_rev 未过期、reviewer 已填 |

### 4.1 校验器**做不到**什么

必须明确边界，避免把「机器 PASS」误读成「内容正确」：

- **符号全对但结论错**（例如把 `prob_map` 说成 ESDF 的基类）：校验器抓不到
- **算法动机、设计意图**：抓不到
- **漏写段落**：只有当它导致 `coverage.missing_file` 或结构缺失时才抓得到

这些由**独立审核 Agent** 补位：只拿 Evidence Pack + 文档，任务是
「找出无法被证据支撑的句子」，产出 `待确认清单.md` 条目。
审核 Agent 与生成 Agent 必须**不同上下文**。

**`待确认清单.md` 为空视为不合格** —— 全无不确定项意味着过度自信，
而不是文档完美。

---

## 5. 禁止项

1. 编造类名、函数名、成员名、话题名、参数名、参数值
2. 把注释里的旧值当作当前值
3. 用 `推测`、`通常`、`应该` 这类模糊措辞代替锚点
4. 在无锚点的小节里下确定性结论
5. 声称「已验证 / 已测试 / 实测」而仓库中不存在对应测试
6. 修改 `src/`、`config/`、`launch/`、任何 `CMakeLists.txt` 或参数默认值
7. 在文档里复制大段源码代替解释（引用锚点即可）
8. 写入涉密或敏感材料（见 `../AGENTS.md`）

---

## 6. 验收标准

模块文档达到下列条件才算通过：

- [ ] `verify_docs.py` 判定 **`errors == 0`**
- [ ] 模块内**每一个源文件**都出现在文件表或明确列入「排除 + 理由」
- [ ] 每条【事实】都能点开到源码行，且锚点附近确有该符号
- [ ] `待确认清单.md` 非空，且与 `params.json` 的 `unresolved_paths` /
      `unread_keys` / `subnode_reads` 缺口对齐
- [ ] 注入测试：故意写入假话题、错误行号、编造符号后，
      校验器**必须 FAIL**（见 §8）

### 6.1 关于 `status: PASS` 与人工审核

`reviewer` 字段是**人工审核的机器闸门**，不允许 AI 自行填一个名字来"通过"。

因此在人工审核之前，正确状态是：

```
status = NEEDS_FIX，errors = 0，warnings = 每篇一条 review.pending
```

这不是失败，而是**设计要求**：机器可验证的部分已经全部通过，剩下的是
`docs/nav/AGENTS.md` 要求的「人工审核闭环」。人工确认后填写 `reviewer`
与 `status: reviewed`，才可能得到 `PASS`。

---

## 7. 金样例单一 owner

金样例**只有一份**：`docs/nav/map/`。

`example/` 不保存副本 —— 两份必然漂移。AI 需要看「真文档长什么样」时，
直接读 `docs/nav/map/`，其 front-matter 中 `doc` 字段指明该文件属于哪一类。

---

## 8. 注入测试

为证明校验器真的能拦住幻觉，而不是只会输出 PASS：

```bash
python3 docs/tools/test_injection.py --root .
```

该脚本在临时副本上注入三个错误（假话题 `/cmd_vel`、错行号
`rog_map.cpp:9999`、编造符号 `ROGMap::nonexistentMethod`），
要求校验器对它们**全部报错并返回 FAIL**，同时原文档 `errors == 0`。

---

## 9. 重新生成的完整命令

新增或更新一个模块的文档时，按下面顺序执行（**不需要构建被文档化的仓库**）：

```bash
cd /home/xyt/map/src/MA_Nav

# 1) 模块符号表（C++ 工具，需先构建一次）
./docs/tools/cpp/inventory/build/ma_nav_inventory \
    --root . --module map \
    --sources src/map/src --sources src/map/include \
    --compdb-dir build \
    --out docs/_evidence/map/inventory.json

# 2) 仓库级接口与配置证据（Python，零构建）
python3 docs/tools/extract_ros2.py   --root . --module ros2 --scan src \
    --binding-header src/ros2/include/ros2/config.hpp \
    --yaml config/planner.yaml --out docs/_evidence/repo/ros2.json
python3 docs/tools/extract_params.py --root . --module map \
    --yaml config/map.yaml --yaml config/planner.yaml \
    --scan src/map/include --scan src/ros2/include \
    --out docs/_evidence/repo/params.json
python3 docs/tools/extract_launch.py --root . --module ros2 \
    --launch launch/run.launch.py --out docs/_evidence/repo/launch.json

# 3) 生成文档（AI 只读 Evidence Pack，不自由翻仓库）
#    每个模块 9 篇，front-matter 的 evidence_digest 取第 4 步算出的值

# 4) 校验
python3 docs/tools/verify_docs.py --root . --module map \
    --docs docs/nav/map --evidence docs/_evidence \
    --report docs/_evidence/verify.map.json
python3 docs/tools/test_injection.py --root .   # 校验器自检
```

C++ 工具首次构建（独立工程，不进 colcon、不改 `src/` 下任何 CMakeLists）：

```bash
cmake -S docs/tools/cpp/inventory -B docs/tools/cpp/inventory/build
cmake --build docs/tools/cpp/inventory/build -j
```

### 9.1 已知覆盖边界

| 边界 | 影响 | 处理 |
|---|---|---|
| `symbol.unknown` 只查**本模块** inventory | 引用 utils / planner 等跨模块符号时，限定名会误报 | 跨模块符号不要用反引号限定名，改用「文件锚点」表述 |
| `subnode_reads` 未做跨函数调用点推导 | 形如 `node["key"]` 的读取无法确定绝对键路径 | 必须在 `待确认清单.md` 中如实列为缺口语义，不得猜 |
| 校验器无法判断语义对错 | 符号全对但结论错抓不到 | 由独立审核 Agent + 人工审核补位（§4.1） |

---

## 附：原始任务要求（保留原文，不再修改）

### README编写

#### 【背景】

README编写

项目：
技术栈：
模块：
职责：

#### 【模块概述】

### 【任务】

输出：
1. 模块概述
2. 文件与类说明（表格）
3. 接口文档（表格）
5. 流程图（Mermaid）
6. 配置说明
7. 测试要点
8. FAQ
9. 待确认清单

### 【要求】

- 只基于给定材料，不编造
- 不确定标注“待确认”
- 保留原始类名、字段名
- 中文输出，Markdown 格式
