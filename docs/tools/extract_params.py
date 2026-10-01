#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 证据提取器（配置参数）

本仓库存在**两种**截然不同的参数读取机制，提取器必须同时覆盖，否则会把
「读不到」误报成「没被读」：

A) ``yaml_loader::YamlLoader::LoadParam("rog_map/esdf/resolution", member, default)``
   —— map 模块使用。路径是**斜杠分隔字符串**，带 namespace 前缀
   （``Config(cfg_path, name_space = "rog_map")``）。默认值写在代码里，
   可能与 YAML 值不同（例如 odom_timeout 代码默认 0.05、YAML 为 0.5）。

B) ``config["ros2"]["map_topic_name"]`` / ``node["rho_energy"]``
   —— ros2 模块使用。前者根节点是文档根，键链即完整路径；后者作用在
   一个**局部子节点**上，从正则无法确定绝对路径，必须显式标记为
   ``context_unknown``，绝不能当成真值，也不能算作「未被引用」。

产出：

- ``yaml_leaves``      每个 YAML 叶子键的解析值 + 行号（真值）
- ``bindings``         已确定绝对路径的绑定，含代码默认值
- ``subnode_reads``    上下文未知的键读取（进「待确认清单」）
- ``unresolved_paths`` LoadParam 路径无法解析到 YAML 叶子（真实缺口）
- ``unread_keys``      YAML 中从未被任何读取点提及的键（软信号）

用法::

    python3 docs/tools/extract_params.py --root <repo> --module map \\
        --yaml config/map.yaml --yaml config/planner.yaml \\
        --scan src/map/include --scan src/ros2/include \\
        --out docs/_evidence/repo/params.json
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import evidence_common as ec  # noqa: E402

GENERATOR = "docs/tools/extract_params.py"

LOAD_PARAM_RE = re.compile(r"LoadParam\s*(?:<[^<>]*>)?\s*\(")
CHAIN_RE = re.compile(r"(\w+)\s*((?:\[\s*\"[^\"]+\"\s*\])+)")
KEY_RE = re.compile(r"\"([^\"]+)\"")
ASSIGN_PREFIX_RE = re.compile(r"(\w+)\s*=\s*$")
# 形如 `name_space + "/esdf/resolution"` 或 `prefix + "/x"`
PREFIX_PLUS_LITERAL_RE = re.compile(r"^\s*(\w+)\s*\+\s*\"([^\"]*)\"\s*$")
LITERAL_RE = re.compile(r"^\s*\"([^\"]*)\"\s*$")
# 形如 `const std::string& name_space = "rog_map"`
NAMESPACE_DEFAULT_RE = re.compile(
    r"\b(\w+)\s*=\s*\"([^\"]*)\"\s*[,)]")


def collect_namespace_defaults(text: str) -> dict[str, str]:
    """收集函数签名与局部声明中的字符串默认值，用于解析 LoadParam 前缀。

    例如 ``Config(const std::string& cfg_path, const std::string& name_space = "rog_map")``
    → ``{"name_space": "rog_map"}``
    """
    out: dict[str, str] = {}
    for line in text.splitlines():
        for m in NAMESPACE_DEFAULT_RE.finditer(line):
            out.setdefault(m.group(1), m.group(2))
    return out


def normalize_path(path: str) -> str:
    return ".".join(p for p in re.split(r"[/.]+", path.strip("/")) if p)


def main() -> int:
    ap = argparse.ArgumentParser(description="提取配置参数证据")
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", required=True)
    ap.add_argument("--yaml", action="append", default=[], help="YAML 文件（相对 root）")
    ap.add_argument("--scan", action="append", default=[], help="扫描读取点的目录（相对 root）")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    # ---------------------------------------------------------- YAML 真值
    leaves: list[dict] = []
    for rel in args.yaml:
        abs_p = pathlib.Path(args.root) / rel
        if not abs_p.is_file():
            print(f"警告: YAML 不存在 {rel}", file=sys.stderr)
            continue
        for leaf in ec.yaml_leaves(str(abs_p)):
            leaves.append({**leaf, "file": rel})
    by_path: dict[str, list[dict]] = {}
    for leaf in leaves:
        by_path.setdefault(leaf["path"], []).append(leaf)

    def match(path: str) -> list[dict]:
        return [{"file": x["file"], "value_repr": x["value_repr"], "line": x["line"]}
                for x in by_path.get(path, [])]

    # ---------------------------------------------------------- 扫描读取点
    scan_files = ec.iter_module_files(args.root, args.scan)
    bindings: list[dict] = []
    subnode_reads: list[dict] = []
    unresolved_paths: list[dict] = []

    for rel in scan_files:
        text = ec.read_text(args.root, rel)
        ns_defaults = collect_namespace_defaults(text)

        # --- A) LoadParam("prefix/path", member, default)
        for m in LOAD_PARAM_RE.finditer(text):
            arg_str = ec.balanced_args(text, m.end() - 1)
            arglist = ec.split_top_level(arg_str)
            if len(arglist) < 2:
                continue
            path_expr, member = arglist[0], arglist[1]
            default_expr = arglist[2] if len(arglist) > 2 else None
            line = ec.line_of(text, m.start())

            prefix_var, literal = None, None
            pm = PREFIX_PLUS_LITERAL_RE.match(path_expr)
            if pm:
                prefix_var, literal = pm.group(1), pm.group(2)
            else:
                lm = LITERAL_RE.match(path_expr)
                if lm:
                    literal = lm.group(1)

            if literal is None:
                unresolved_paths.append({
                    "file": rel, "line": line, "path_expr": path_expr,
                    "cpp_member": member, "why": "路径表达式不是 前缀+字面量",
                })
                continue

            prefix = ""
            if prefix_var:
                if prefix_var in ns_defaults:
                    prefix = ns_defaults[prefix_var]
                else:
                    unresolved_paths.append({
                        "file": rel, "line": line, "path_expr": path_expr,
                        "cpp_member": member,
                        "why": f"无法确定前缀变量 {prefix_var} 的取值",
                    })
                    continue
            full = normalize_path(f"{prefix}/{literal}" if prefix else literal)
            matches = match(full)
            rec = {
                "mechanism": "YamlLoader::LoadParam",
                "file": rel, "line": line,
                "yaml_path": full,
                "path_expr": path_expr,
                "cpp_member": member,
                "code_default_expr": default_expr,
                "matched_in": matches,
            }
            if matches:
                bindings.append(rec)
            else:
                unresolved_paths.append({**rec, "why": "YAML 中不存在该键路径"})

        # --- B) config["a"]["b"] / node["key"]
        for lineno, line_text in enumerate(text.splitlines(), 1):
            for cm in CHAIN_RE.finditer(line_text):
                keys = KEY_RE.findall(cm.group(2))
                if not keys:
                    continue
                lhs_m = ASSIGN_PREFIX_RE.search(line_text[:cm.start()])
                path = ".".join(keys)
                matches = match(path)
                if matches:
                    bindings.append({
                        "mechanism": "yaml-cpp 键链",
                        "file": rel, "line": lineno,
                        "yaml_path": path,
                        "path_expr": f'{cm.group(1)}{cm.group(2)}',
                        "cpp_member": lhs_m.group(1) if lhs_m else None,
                        "code_default_expr": None,
                        "matched_in": matches,
                    })
                else:
                    # 作用在局部子节点上，绝对路径不可知
                    candidates = sorted({
                        leaf["path"] for leaf in leaves
                        if leaf["path"].split(".")[-1] == keys[-1]
                    })
                    subnode_reads.append({
                        "mechanism": "yaml-cpp 子节点",
                        "file": rel, "line": lineno,
                        "keys": keys,
                        "subnode_root": cm.group(1),
                        "cpp_member": lhs_m.group(1) if lhs_m else None,
                        "context_unknown": True,
                        "suffix_candidates": candidates,
                    })

    # 去重
    def dedupe(rows: list[dict], fields) -> list[dict]:
        def norm(v):
            return tuple(v) if isinstance(v, list) else v

        seen, out = set(), []
        for r in rows:
            key = tuple(norm(r.get(f)) for f in fields)
            if key in seen:
                continue
            seen.add(key)
            out.append(r)
        return out

    bindings = dedupe(bindings, ("file", "line", "yaml_path", "cpp_member"))
    # 同一行内 `node["k"]` 可能出现两次（判断 + 赋值），按行去重即可，
    # 否则统计数会被重复计入。
    subnode_reads = dedupe(subnode_reads, ("file", "line", "keys"))
    unresolved_paths = dedupe(unresolved_paths, ("file", "line", "path_expr", "cpp_member"))

    referenced = {b["yaml_path"] for b in bindings}
    last_segments = {r["keys"][-1] for r in subnode_reads}

    def covered(path: str) -> bool:
        """序列元素 `a.b[0]` 的父键 `a.b` 被引用时，元素本身不算缺口。

        否则 `range: [100, 60, 4]` 会产出 3 条伪「未被引用」记录。
        """
        if path in referenced:
            return True
        base = path.split("[")[0]
        return base in referenced

    unread_keys = [
        {"file": leaf["file"], "path": leaf["path"],
         "value_repr": leaf["value_repr"], "line": leaf["line"]}
        for leaf in leaves
        if not covered(leaf["path"]) and leaf["path"].split(".")[-1] not in last_segments
    ]

    ec.emit(
        args.out, "params", args.module, args.root, GENERATOR,
        yaml_files=list(args.yaml),
        scanned_files=scan_files,
        yaml_leaves=leaves,
        bindings=sorted(bindings, key=lambda r: (r["file"], r["line"])),
        subnode_reads=sorted(subnode_reads, key=lambda r: (r["file"], r["line"])),
        unresolved_paths=sorted(unresolved_paths, key=lambda r: (r["file"], r["line"])),
        unread_keys=unread_keys,
    )
    print(f"[params] yaml 叶子 {len(leaves)}，解析绑定 {len(bindings)}，"
          f"上下文未知读取 {len(subnode_reads)}，未解析路径 {len(unresolved_paths)}，"
          f"未被提及键 {len(unread_keys)} → {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
