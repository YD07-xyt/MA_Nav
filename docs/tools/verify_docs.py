#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 确定性校验器

设计前提：**工具不能判断「这句话对不对」，只能判断「这句话引用的东西存不存在、
值是否相等」。** 因此校验器只做三层机器可验证的比对：

  L1 存在性  锚点文件存在、行号在范围内、锚点附近真的出现被引用的符号
  L2 一致性  文档写的值 == 证据里的解析值（不是「文本出现过」）
  L3 覆盖率  真值集合 ⊆ 文档集合（漏写也算缺陷）

抓不到的语义幻觉（符号都在但结论错、算法动机）由独立审核 Agent 产出
「待确认清单」，本校验器**不假装能判定**。

用法::

    python3 docs/tools/verify_docs.py --root <repo> --module map \\
        --docs docs/nav/map --evidence docs/_evidence \\
        --report docs/_evidence/verify.map.json

退出码：0=PASS，2=NEEDS_FIX，1=FAIL
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import evidence_common as ec  # noqa: E402

GENERATOR = "docs/tools/verify_docs.py"

REQUIRED_DOCS = [
    "README.md",
    "模块设计说明.md",
    "接口文档.md",
    "依赖关系.md",
    "流程图.md",
    "配置说明.md",
    "测试要点.md",
    "常见问题FAQ.md",
    "待确认清单.md",
]
FRONT_MATTER_REQUIRED = ["module", "doc", "git_rev", "evidence_digest",
                        "worktree_dirty", "status", "reviewer"]

# 外部命名空间：这些 `::` 符号不属于被文档化的仓库，不参与符号存在性检查
EXTERNAL_NAMESPACES = {
    "std", "Eigen", "rclcpp", "rcl", "sensor_msgs", "nav_msgs", "geometry_msgs",
    "visualization_msgs", "std_msgs", "std_srvs", "tf2", "tf2_ros", "pcl",
    "pcl_conversions", "yaml", "spdlog", "osqp", "OsqpEigen", "ompl",
    "behaviortree_cpp", "boost", "fmt", "cv", "message_filters", "chrono",
    "filesystem", "placeholders", "literals", "this_thread", "string", "vector",
    "map", "set", "unordered_map", "memory", "optional", "function", "utility",
    "Eigen3", "yaml-cpp", "ament_cmake_auto", "cmake",
}

ANCHOR_RE = re.compile(
    r"(?<![/\w])([A-Za-z0-9_][A-Za-z0-9_./\-]*\.(?:cpp|cc|cxx|h|hpp|hxx|hh|yaml|yml|py|xml|json|pgm|txt|md)):(\d+)(?:-(\d+))?"
)
BACKTICK_RE = re.compile(r"`([^`\n]+)`")
TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")
FILESYSTEM_PREFIXES = ("/home", "/usr", "/opt", "/tmp", "/etc", "/var", "/bin", "/lib")
MSG_TYPE_RE = re.compile(r"^\w+::(?:msg|srv|action)::\w+$")
YAML_PATH_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z0-9_]+)+$")
# 代码/配置文件后缀：`rog_map.h`、`map.yaml` 这类是**文件名**，不是 YAML 键路径
CODE_FILE_EXT = {
    "h", "hpp", "hh", "hxx", "cpp", "cc", "cxx", "yaml", "yml", "py", "md",
    "json", "txt", "xml", "pgm", "launch", "rviz", "toml", "cfg", "cmake",
}
TEST_CLAIM_RE = re.compile(r"已测试|已验证|测试通过|实测|验证通过")
INFER_MARKERS = ("【推断】", "【待确认】", "待确认")
_YAML_ROOTS: set[str] = set()

MERMAID_KEYWORDS = (
    "graph", "flowchart", "sequenceDiagram", "classDiagram", "stateDiagram",
    "erDiagram", "gantt", "pie", "journey", "gitGraph", "mindmap", "timeline",
    "quadrantChart", "block-beta", "C4Context",
)


class Report:
    def __init__(self):
        self.errors: list[dict] = []
        self.warnings: list[dict] = []

    def error(self, check: str, doc: str, line, message: str, **extra):
        self.errors.append({"check": check, "doc": doc, "line": line,
                            "message": message, **extra})

    def warn(self, check: str, doc: str, line, message: str, **extra):
        self.warnings.append({"check": check, "doc": doc, "line": line,
                              "message": message, **extra})


def looks_like_yaml_path(tok: str) -> bool:
    """判断一个 token 是否真的是 YAML 键路径（而不是文件名）。"""
    if not YAML_PATH_RE.match(tok):
        return False
    if tok.split(".")[0] not in _YAML_ROOTS:
        return False
    if tok.rsplit(".", 1)[-1] in CODE_FILE_EXT:
        return False
    return True


def worktree_dirty(root: pathlib.Path):
    """只读探测被文档化源码是否有未提交改动（AGENTS.md 允许 git status）。"""
    try:
        proc = subprocess.run(
            ["git", "status", "--porcelain", "--", "src", "config", "launch"],
            cwd=str(root), capture_output=True, text=True, timeout=30)
    except Exception:
        return None, []
    if proc.returncode != 0:
        return None, []
    files = [line[3:].strip() for line in proc.stdout.splitlines() if line.strip()]
    return bool(files), files


def strip_fences(lines: list[str]) -> list[bool]:
    """标记每行是否位于 ``` 围栏代码块内部。"""
    inside = []
    in_fence = False
    for line in lines:
        s = line.strip()
        if s.startswith("```"):
            inside.append(True)
            in_fence = not in_fence
            continue
        inside.append(in_fence)
    return inside


def parse_front_matter(text: str):
    if not text.startswith("---"):
        return None, 0
    end = text.find("\n---", 3)
    if end < 0:
        return None, 0
    raw = text[3:end]
    end_line = text.count("\n", 0, end + 4) + 1
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(raw)
    except Exception:
        return None, end_line
    return (data if isinstance(data, dict) else None), end_line


def near_match(symbol: str, source_lines: list[str], center: int, window: int) -> bool:
    """符号是否出现在锚点附近（按 `::` 最后一段比较）。"""
    tail = symbol.split("::")[-1]
    if not tail:
        return False
    lo = max(0, center - 1 - window)
    hi = min(len(source_lines), center + window)
    for i in range(lo, hi):
        if tail in source_lines[i]:
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="校验模块文档与证据的一致性")
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", required=True)
    ap.add_argument("--docs", required=True, help="模块文档目录（相对 root）")
    ap.add_argument("--evidence", required=True, help="证据根目录（相对 root）")
    ap.add_argument("--report", default=None, help="校验报告 JSON 输出路径")
    ap.add_argument("--window", type=int, default=6, help="锚点附近比对行数")
    args = ap.parse_args()

    root = pathlib.Path(args.root).resolve()
    docs_dir = root / args.docs
    ev_root = root / args.evidence
    rep = Report()

    # ------------------------------------------------------------ 载入证据
    def load(p: pathlib.Path):
        if not p.is_file():
            return None
        return json.loads(p.read_text(encoding="utf-8"))

    inventory = load(ev_root / args.module / "inventory.json")
    ros2 = load(ev_root / "repo" / "ros2.json")
    params = load(ev_root / "repo" / "params.json")
    launch = load(ev_root / "repo" / "launch.json")

    if inventory is None:
        rep.error("evidence.missing", "-", None,
                  f"缺少证据文件 {ev_root / args.module / 'inventory.json'}；"
                  "没有证据就无法校验，禁止在无证据情况下判 PASS")
        summary = {"status": "FAIL", "errors": rep.errors, "warnings": rep.warnings}
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 1

    symbol_qnames = {s["qname"] for s in inventory.get("symbols", [])}
    symbol_names = {s["name"] for s in inventory.get("symbols", [])}
    module_files = set(inventory.get("files", []))

    ros_names = {i["name"] for i in (ros2 or {}).get("interfaces", []) if i.get("name")}
    ros_types = {i["msg_type"] for i in (ros2 or {}).get("interfaces", []) if i.get("msg_type")}
    ros_params = {p["name"] for p in (ros2 or {}).get("ros_params", [])}

    yaml_leaves = {l["path"]: l for l in (params or {}).get("yaml_leaves", [])}
    # 证据里明确登记为「代码读取但 YAML 未定义」的键，是**已知缺口**而不是幻觉：
    # 文档在待确认清单里点名这些键是正确行为，不应被判成 param.unknown_key。
    known_gap_paths = {
        u["yaml_path"] for u in (params or {}).get("unresolved_paths", [])
        if u.get("yaml_path")
    }
    yaml_roots = {p.split(".")[0] for p in yaml_leaves}
    global _YAML_ROOTS
    _YAML_ROOTS = yaml_roots
    for leaf in (params or {}).get("yaml_leaves", []):
        if str(leaf.get("value_repr", "")).startswith("/"):
            ros_names.add(leaf["value_repr"])
    for arg in (launch or {}).get("declared_args", []):
        if arg.get("name"):
            ros_params.add(arg["name"])
    # 代码里的默认值字符串（如 odom_topic 默认 "/lio/odom"）是有证据的，
    # 只是当前未生效。文档说明「代码默认值是 X」时不应被判成编造话题。
    for b in (params or {}).get("bindings", []):
        for lit in re.findall(r'"([^"]*)"', b.get("code_default_expr") or ""):
            if lit.startswith("/"):
                ros_names.add(lit)

    # ------------------------------------------------------------ 逐文档校验
    md_files = sorted(docs_dir.glob("*.md"))
    present = {p.name for p in md_files}
    for required in REQUIRED_DOCS:
        if required not in present:
            rep.error("structure.missing_doc", required, None,
                      f"缺少必需文档 {args.docs}/{required}")

    evidence_digest = ec.evidence_digest(str(ev_root))
    current_rev = ec.git_rev(str(root))
    dirty, dirty_files = worktree_dirty(root)
    all_mentioned_files: set[str] = set()

    for md in md_files:
        text = md.read_text(encoding="utf-8", errors="replace")
        lines = text.splitlines()
        in_fence = strip_fences(lines)
        rel_doc = str(md.relative_to(root))

        # ---- front-matter
        fm, fm_end = parse_front_matter(text)
        if fm is None:
            rep.error("structure.front_matter", rel_doc, 1,
                      "缺少 YAML front-matter（--- 开头）")
        else:
            for field in FRONT_MATTER_REQUIRED:
                if field not in fm:
                    rep.error("structure.front_matter", rel_doc, 1,
                              f"front-matter 缺少字段 {field}")
            if fm.get("module") not in (None, args.module):
                rep.error("structure.front_matter", rel_doc, 1,
                          f"front-matter module={fm.get('module')} 与 --module {args.module} 不符")
            if fm.get("evidence_digest") and fm["evidence_digest"] != evidence_digest:
                rep.warn("evidence.stale", rel_doc, 1,
                         f"证据摘要已变化：文档记录 {fm['evidence_digest']}，"
                         f"当前 {evidence_digest}；该文档需要重新生成")
            if fm.get("git_rev") and fm["git_rev"] != current_rev:
                rep.warn("evidence.stale", rel_doc, 1,
                         f"git_rev 已变化：文档 {fm['git_rev']}，当前 {current_rev}")
            claimed = fm.get("worktree_dirty")
            if claimed is not None and dirty is not None:
                if str(claimed).strip().lower() != str(dirty).lower():
                    rep.error("evidence.worktree_mismatch", rel_doc, 1,
                              f"front-matter 声明 worktree_dirty={claimed}，"
                              f"但 src/config/launch 下实际未提交改动为 {dirty}"
                              f"（{len(dirty_files)} 个文件）")
            reviewer = str(fm.get("reviewer", ""))
            if reviewer.strip() in ("", "TODO", "待填", "unknown"):
                rep.warn("review.pending", rel_doc, 1, "reviewer 未填写")
            # status=draft 即「尚未人工审核」。这条闸门不允许 AI 自行绕过：
            # 只有人工确认后才把 status 改为 reviewed。
            if str(fm.get("status", "")).strip() != "reviewed":
                rep.warn("review.pending", rel_doc, 1,
                         f"status={fm.get('status')}，尚未通过人工审核；"
                         "人工确认后改为 reviewed")

        # ---- 围栏内不参与锚点/符号检查（代码块是引用而非断言）
        body_start = fm_end

        # ---- L1 锚点存在性 + L2 锚点支撑性
        for idx, line in enumerate(lines):
            if idx < body_start or in_fence[idx]:
                continue
            for m in ANCHOR_RE.finditer(line):
                path, start, end = m.group(1), int(m.group(2)), m.group(3)
                target = root / path
                if not target.is_file():
                    rep.error("anchor.file_missing", rel_doc, idx + 1,
                              f"锚点文件不存在：{path}", anchor=path)
                    continue
                all_mentioned_files.add(path)
                total = sum(1 for _ in target.open(encoding="utf-8", errors="replace"))
                last = int(end) if end else start
                if start < 1 or last > total:
                    rep.error("anchor.line_out_of_range", rel_doc, idx + 1,
                              f"锚点行号越界：{path}:{start}"
                              f"{'-' + end if end else ''}（文件共 {total} 行）",
                              anchor=f"{path}:{start}")
                    continue
                # L2 支撑性：该行反引号里的符号，至少要有一个出现在锚点附近
                symbols = [s.strip() for s in BACKTICK_RE.findall(line)]
                symbols = [s for s in symbols if re.fullmatch(r"[A-Za-z_]\w*(::\w+)*", s)]
                # 外部命名空间的符号（std::atomic 等）不属于本仓库，不能要求
                # 它们出现在被引用文件的锚点附近。
                symbols = [s for s in symbols
                           if not (s.split("::")[0] in EXTERNAL_NAMESPACES)]
                if symbols:
                    src_lines = target.read_text(encoding="utf-8",
                                                 errors="replace").splitlines()
                    if not any(near_match(s, src_lines, start, args.window) for s in symbols):
                        rep.error("anchor.unsupported", rel_doc, idx + 1,
                                  f"锚点 {path}:{start} 附近未出现该行引用的符号 "
                                  f"{symbols}；锚点未支撑这句话",
                                  anchor=f"{path}:{start}")

        # ---- L2 符号存在性 / ROS 名 / 参数键
        for idx, line in enumerate(lines):
            if idx < body_start or in_fence[idx]:
                continue
            for tok in (s.strip() for s in BACKTICK_RE.findall(line)):
                # 只有形如 a::b::c 的 token 才当作符号检查；
                # `rog_map::Config(cfg_path)` 这类是代码片段，不是符号引用。
                if "::" in tok and not re.fullmatch(r"[A-Za-z_]\w*(::\w+)*", tok):
                    continue
                if MSG_TYPE_RE.match(tok):
                    if ros_types and tok not in ros_types:
                        rep.error("ros2.unknown_type", rel_doc, idx + 1,
                                  f"消息类型不在证据中：{tok}")
                    continue
                if TOPIC_RE.match(tok) and not tok.startswith(FILESYSTEM_PREFIXES):
                    if tok not in ros_names:
                        rep.error("ros2.unknown_topic", rel_doc, idx + 1,
                                  f"话题/服务名不在证据中：{tok}")
                    continue
                if "::" in tok:
                    ns = tok.split("::")[0]
                    if ns in EXTERNAL_NAMESPACES:
                        continue
                    if tok in symbol_qnames or tok in symbol_names:
                        continue
                    if any(q.endswith("::" + tok) or q == tok for q in symbol_qnames):
                        continue
                    rep.error("symbol.unknown", rel_doc, idx + 1,
                              f"符号在 inventory 中不存在：{tok}")
                    continue
                if looks_like_yaml_path(tok):
                    if tok not in yaml_leaves and tok not in known_gap_paths:
                        rep.error("param.unknown_key", rel_doc, idx + 1,
                                  f"YAML 键路径不在证据中：{tok}")

            # ---- 测试断言：无测试仓库禁止出现「已验证」类断言
            if TEST_CLAIM_RE.search(line) and not any(x in line for x in INFER_MARKERS):
                rep.error("claim.unverified_test", rel_doc, idx + 1,
                          "出现「已验证/已测试」类断言但未标注【推断】/【待确认】；"
                          "本仓库 src/ 下不存在任何测试")

        # ---- L2 参数表格一致性
        for idx, line in enumerate(lines):
            if idx < body_start or in_fence[idx] or not line.strip().startswith("|"):
                continue
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if all(set(c) <= set("-: ") for c in cells if c):
                continue
            for cell in cells:
                raw = cell.strip().strip("`").strip()
                if raw in yaml_leaves:
                    leaf = yaml_leaves[raw]
                    expected = str(leaf["value_repr"]).replace(" ", "").replace("`", "")
                    row_norm = [c.replace(" ", "").replace("`", "") for c in cells]
                    # 值只要在该行**出现过**即可（允许写在散文里，如「为 false」）；
                    # 精确到单元格会误报。真值仍来自解析结果，不会放过错值。
                    if not any(expected in c for c in row_norm):
                        rep.error("param.value_mismatch", rel_doc, idx + 1,
                                  f"参数 {raw} 的值与证据不符：证据为 {leaf['value_repr']}，"
                                  f"表格该行未出现该值（行内容 {cells}）")
                elif looks_like_yaml_path(raw) \
                        and raw not in yaml_leaves and raw not in known_gap_paths:
                    rep.error("param.unknown_key", rel_doc, idx + 1,
                              f"表格引用了不存在的 YAML 键路径：{raw}")

        # ---- C7 Mermaid 结构
        for idx, line in enumerate(lines):
            if line.strip().startswith("```mermaid"):
                j = idx + 1
                while j < len(lines) and not lines[j].strip().startswith("```"):
                    j += 1
                block = [x.strip() for x in lines[idx + 1:j] if x.strip()]
                if not block:
                    rep.error("structure.mermaid", rel_doc, idx + 1, "空的 mermaid 代码块")
                elif not block[0].startswith(MERMAID_KEYWORDS):
                    rep.error("structure.mermaid", rel_doc, idx + 1,
                              f"mermaid 块首行不是已知图类型：{block[0][:40]}")
                else:
                    joined = "\n".join(block)
                    if joined.count("[") != joined.count("]"):
                        rep.warn("structure.mermaid", rel_doc, idx + 1,
                                 "mermaid 块中 [] 不配对，可能无法渲染")

        # ---- C8 每节至少一条锚点或显式标注
        section_start = None
        for idx, line in enumerate(lines):
            if idx < body_start:
                continue
            if line.startswith("## "):
                if section_start is not None:
                    check_section(lines, section_start, idx, in_fence, rel_doc, rep)
                section_start = idx
        if section_start is not None:
            check_section(lines, section_start, len(lines), in_fence, rel_doc, rep)

    # ---- L3 覆盖率：模块每个源文件都必须在文档里被提到
    missing = sorted(f for f in module_files if f not in all_mentioned_files)
    if missing:
        rep.error("coverage.missing_file", args.docs, None,
                  f"模块内有 {len(missing)} 个源文件未被任何文档提及：{missing}")

    # ------------------------------------------------------------ 汇总
    status = "PASS"
    if rep.errors:
        status = "FAIL"
    elif rep.warnings:
        status = "NEEDS_FIX"

    report = {
        "schema": "ma_nav.verify.v1",
        "module": args.module,
        "docs": args.docs,
        "status": status,
        "error_count": len(rep.errors),
        "warning_count": len(rep.warnings),
        "evidence_digest": evidence_digest,
        "git_rev": current_rev,
        "worktree_dirty": dirty,
        "dirty_source_files": dirty_files,
        "errors": rep.errors,
        "warnings": rep.warnings,
        "coverage": {
            "module_files": len(module_files),
            "mentioned_files": len(module_files & all_mentioned_files),
            "missing_files": missing,
        },
    }
    if args.report:
        p = pathlib.Path(args.report)
        if not p.is_absolute():
            p = root / args.report
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                     encoding="utf-8")

    print(f"[verify] module={args.module} status={status} "
          f"errors={len(rep.errors)} warnings={len(rep.warnings)}")
    for e in rep.errors[:40]:
        print(f"  ERROR {e['check']:26s} {e['doc']}:{e['line']} {e['message']}")
    if len(rep.errors) > 40:
        print(f"  ... 其余 {len(rep.errors) - 40} 条见报告 JSON")
    for w in rep.warnings[:20]:
        print(f"  WARN  {w['check']:26s} {w['doc']}:{w['line']} {w['message']}")
    if args.report:
        print(f"  报告: {args.report}")

    return {"PASS": 0, "FAIL": 1, "NEEDS_FIX": 2}[status]


def check_section(lines, start, end, in_fence, rel_doc, rep):
    """一个 H2 小节内必须至少有一条锚点或显式【推断】/【待确认】标注。"""
    title = lines[start].strip()
    body = [(i, lines[i]) for i in range(start, end)]
    if not body:
        return
    has_anchor = any(ANCHOR_RE.search(l) for i, l in body if not in_fence[i])
    has_marker = any(any(m in l for m in INFER_MARKERS) for i, l in body)
    # 纯目录/清单性质的小节（内容很少）不强制
    content_lines = [l for i, l in body[1:] if l.strip() and not in_fence[i]]
    if has_anchor or has_marker or len(content_lines) <= 2:
        return
    rep.error("annotation.unsupported_section", rel_doc, start + 1,
              f"小节「{title}」既没有锚点（file:line），也没有【推断】/【待确认】标注；"
              "无法机器验证的内容必须显式降级")


if __name__ == "__main__":
    raise SystemExit(main())
