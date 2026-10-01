#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 证据提取器公共库

统一契约（所有证据 JSON 共有字段）::

    {
      "schema": "ma_nav.evidence.v1",
      "kind":   "inventory|ros2|params|launch",
      "module": "<模块名>",
      "generator": "<产出该文件的工具与版本>",
      "git_rev": "<短 revision>",
      "generated_at": "<UTC ISO8601>",
      "root": "<仓库绝对路径>",
      ... 各 kind 专属字段 ...
    }

本库只读被文档化的仓库，只写 --out 指定的 JSON。
"""

from __future__ import annotations

import datetime
import hashlib
import json
import os
import pathlib
import re
import sys

SCHEMA = "ma_nav.evidence.v1"
SOURCE_EXTS = {".cpp", ".cc", ".cxx", ".h", ".hpp", ".hxx", ".hh"}


def git_rev(root: str) -> str:
    """从 .git/HEAD 解析当前 revision，不调用 git（避免任何仓库写操作风险）。"""
    head_path = pathlib.Path(root) / ".git" / "HEAD"
    try:
        head = head_path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return "unknown"
    if head.startswith("ref: "):
        try:
            ref = (pathlib.Path(root) / ".git" / head[5:].strip()).read_text(
                encoding="utf-8", errors="replace"
            ).strip()
            if ref:
                return ref[:12]
        except OSError:
            pass
    return head[:12]


def now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def iter_module_files(root: str, dirs) -> list[str]:
    """递归收集模块内源文件，返回相对 root 的路径（已排序去重）。"""
    out: set[str] = set()
    for d in dirs:
        base = pathlib.Path(root) / d
        if not base.is_dir():
            print(f"警告: 目录不存在 {base}", file=sys.stderr)
            continue
        for p in base.rglob("*"):
            if p.is_file() and p.suffix in SOURCE_EXTS:
                out.add(str(p.relative_to(root)))
    return sorted(out)


def read_text(root: str, rel: str) -> str:
    try:
        return (pathlib.Path(root) / rel).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def emit(out_path: str, kind: str, module: str, root: str, generator: str, **payload) -> dict:
    """写出证据 JSON，返回写入的字典。"""
    doc = {
        "schema": SCHEMA,
        "kind": kind,
        "module": module,
        "generator": generator,
        "git_rev": git_rev(root),
        "generated_at": now_iso(),
        "root": str(pathlib.Path(root).resolve()),
    }
    doc.update(payload)
    text = json.dumps(doc, ensure_ascii=False, indent=2, sort_keys=False)
    p = pathlib.Path(out_path)
    if p.parent and str(p.parent) not in ("", "."):
        p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text + "\n", encoding="utf-8")
    return doc


def evidence_digest(evidence_dir: str) -> str:
    """对证据包内容取稳定摘要，用于文档 front-matter 的 stale 检测。"""
    h = hashlib.sha256()
    base = pathlib.Path(evidence_dir)
    for p in sorted(base.rglob("*.json")):
        # verify.*.json 是校验器自己的输出，不能算进证据摘要，否则自引用：
        # 报告一变摘要就变，所有文档立刻被判 stale。
        if p.name.startswith("verify."):
            continue
        h.update(p.name.encode())
        # 去掉 generated_at / git_rev 这类每次都变的字段，只对语义内容取摘要
        data = json.loads(p.read_text(encoding="utf-8"))
        for volatile in ("generated_at", "git_rev", "root"):
            data.pop(volatile, None)
        h.update(json.dumps(data, ensure_ascii=False, sort_keys=True).encode())
    return h.hexdigest()[:16]


# ------------------------------------------------------------------ 通用参数

def add_common_args(parser, default_out_kind: str):
    parser.add_argument("--root", required=True, help="仓库根目录")
    parser.add_argument("--module", required=True, help="模块名，写入证据 JSON")
    parser.add_argument("--out", required=True, help="输出证据 JSON 路径")
    return parser


# ------------------------------------------------------------------ YAML 工具

def _yaml_module():
    try:
        import yaml  # type: ignore
    except ImportError:  # pragma: no cover
        print("需要 PyYAML：pip install pyyaml", file=sys.stderr)
        raise SystemExit(3)
    return yaml


def _scalar_typed(node, yaml_mod):
    """把 ScalarNode 还原成带类型的 Python 值。"""
    tag = node.tag
    v = node.value
    if tag.endswith(":null"):
        return None
    if tag.endswith(":bool"):
        return v.lower() in ("true", "yes", "on")
    if tag.endswith(":int"):
        try:
            return int(v, 0)
        except ValueError:
            return v
    if tag.endswith(":float"):
        try:
            return float(v)
        except ValueError:
            return v
    return v


def yaml_leaves(path: str) -> list[dict]:
    """解析 YAML 并保留**行号**（yaml.safe_load 会丢行号，故用 compose）。

    返回 [{"path": "a.b.c", "value": <typed>, "value_repr": str, "line": int}, ...]
    """
    yaml_mod = _yaml_module()
    text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
    leaves: list[dict] = []

    def walk(node, prefix: str):
        if isinstance(node, yaml_mod.MappingNode):
            for k, v in node.value:
                key = str(_scalar_typed(k, yaml_mod))
                walk(v, f"{prefix}.{key}" if prefix else key)
        elif isinstance(node, yaml_mod.SequenceNode):
            # 序列本身也要作为一条记录：`range: [100, 60, 4]` 的真值是整条列表，
            # 若只展平成 range[0..2]，代码里 LoadParam(".../range", ...) 会被
            # 误报成「YAML 中不存在该键路径」。
            items = [_render_node(v, yaml_mod) for v in node.value]
            leaves.append({
                "path": prefix,
                "value": items,
                "value_repr": "[" + ", ".join(_repr_value(x) for x in items) + "]",
                "line": node.start_mark.line + 1,
                "is_sequence": True,
            })
            for i, v in enumerate(node.value):
                walk(v, f"{prefix}[{i}]")
        elif isinstance(node, yaml_mod.ScalarNode):
            val = _scalar_typed(node, yaml_mod)
            leaves.append({
                "path": prefix,
                "value": val,
                "value_repr": _repr_value(val),
                "line": node.start_mark.line + 1,
            })

    try:
        docs = list(yaml_mod.compose_all(text))
    except yaml_mod.YAMLError as exc:
        print(f"警告: YAML 解析失败 {path}: {exc}", file=sys.stderr)
        return []
    for doc in docs:
        if doc is not None:
            walk(doc, "")
    return leaves


def _repr_value(val) -> str:
    if val is None:
        return "null"
    if isinstance(val, bool):
        return "true" if val else "false"
    return str(val)


def _render_node(node, yaml_mod):
    """把任意 YAML 节点渲染成 Python 值（用于序列/嵌套结构的展示）。"""
    if isinstance(node, yaml_mod.ScalarNode):
        return _scalar_typed(node, yaml_mod)
    if isinstance(node, yaml_mod.SequenceNode):
        return [_render_node(v, yaml_mod) for v in node.value]
    if isinstance(node, yaml_mod.MappingNode):
        return {str(_scalar_typed(k, yaml_mod)): _render_node(v, yaml_mod) for k, v in node.value}
    return None


# ------------------------------------------- C++ 中 config["a"]["b"] 的键链提取

_CFG_CHAIN = re.compile(
    r"(?P<lhs>\w+)\s*=\s*(?P<obj>\w+)\s*(?P<chain>(?:\[\s*\"[^\"]+\"\s*\])+)"
)
_CFG_KEY = re.compile(r"\"([^\"]+)\"")


def split_top_level(args: str) -> list[str]:
    """按顶层逗号切分实参，忽略括号/尖括号/花括号/字符串内的逗号。"""
    out, buf, depth, in_str, esc = [], [], 0, False, False
    for ch in args:
        if in_str:
            buf.append(ch)
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
            buf.append(ch)
        elif ch in "(<[{":
            depth += 1
            buf.append(ch)
        elif ch in ")>]}":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            out.append("".join(buf).strip())
            buf = []
        else:
            buf.append(ch)
    if buf:
        out.append("".join(buf).strip())
    return [a for a in out if a != ""]


def balanced_args(text: str, open_paren: int) -> str:
    """从 `(` 位置起取括号内容（不含最外层括号）。"""
    depth, in_str, esc = 0, False, False
    for i in range(open_paren, len(text)):
        ch = text[i]
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return text[open_paren + 1:i]
    return text[open_paren + 1:]


def line_of(text: str, pos: int) -> int:
    return text.count("\n", 0, pos) + 1


def extract_config_bindings(source_files: dict[str, str]) -> list[dict]:
    """提取 `member = yaml["a"]["b"];` 这类 **代码变量 ↔ yaml 键链** 的绑定。

    这是把代码里的 `config.cmd_vel_name` 解析成 yaml 值 `ros2.cmd_vel_name`
    的 join key；没有它就无法从代码反查真值。
    """
    out: list[dict] = []
    for rel, text in source_files.items():
        for lineno, line in enumerate(text.splitlines(), 1):
            for m in _CFG_CHAIN.finditer(line):
                keys = _CFG_KEY.findall(m.group("chain"))
                out.append({
                    "cpp_member": m.group("lhs"),
                    "yaml_expr_root": m.group("obj"),
                    "yaml_keys": keys,
                    "yaml_path": ".".join(keys),
                    "file": rel,
                    "line": lineno,
                })
    return out
