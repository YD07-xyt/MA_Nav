#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 证据提取器（launch）

用 Python 的 ``ast`` **真实解析** launch 文件，而不是正则匹配文本。

这样做的直接好处：被注释掉的节点（例如 `launch/run.launch.py` 里注释掉的
rviz2）不会出现在 AST 中，因此不会被误判为「实际启动」。文本匹配无法区分
「注释里的节点」和「真正启动的节点」。

产出：launch 声明参数、节点（含 package/executable/name/参数/remapping）、
以及 LaunchDescription 实际返回的可执行集合。

用法::

    python3 docs/tools/extract_launch.py --root <repo> --module ros2 \\
        --launch launch/run.launch.py --out docs/_evidence/repo/launch.json
"""

from __future__ import annotations

import argparse
import ast
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import evidence_common as ec  # noqa: E402

GENERATOR = "docs/tools/extract_launch.py"


def render(node) -> str:
    try:
        return ast.unparse(node)
    except Exception:  # pragma: no cover
        return "<无法渲染>"


def kwarg(call: ast.Call, name: str):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def positionals(call: ast.Call) -> list:
    return list(call.args)


def main() -> int:
    ap = argparse.ArgumentParser(description="提取 launch 证据")
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", default="ros2")
    ap.add_argument("--launch", action="append", required=True, help="launch 文件（相对 root）")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    files: list[dict] = []
    declared_args: list[dict] = []
    nodes: list[dict] = []
    active_exprs: list[dict] = []
    parse_errors: list[dict] = []

    for rel in args.launch:
        abs_p = pathlib.Path(args.root) / rel
        if not abs_p.is_file():
            parse_errors.append({"file": rel, "error": "文件不存在"})
            continue
        src = abs_p.read_text(encoding="utf-8", errors="replace")
        try:
            tree = ast.parse(src, filename=rel)
        except SyntaxError as exc:
            parse_errors.append({"file": rel, "error": f"语法错误: {exc}"})
            continue
        files.append({"file": rel, "lines": src.count("\n") + 1})

        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            fname = func.id if isinstance(func, ast.Name) else (
                func.attr if isinstance(func, ast.Attribute) else "")

            if fname == "DeclareLaunchArgument":
                pos = positionals(node)
                name = render(pos[0]) if pos else None
                if name and name.startswith(("'", '"')):
                    name = name.strip("'\"")
                default = kwarg(node, "default_value")
                declared_args.append({
                    "file": rel,
                    "line": node.lineno,
                    "name": name,
                    "default_expr": render(default) if default is not None else None,
                    "description_expr": render(kwarg(node, "description"))
                    if kwarg(node, "description") is not None else None,
                })
            elif fname == "Node":
                params = kwarg(node, "parameters")
                entries = []
                if params is not None:
                    try:
                        for elt in ast.literal_eval(params):
                            entries.append(elt)
                    except Exception:
                        entries.append({"__raw__": render(params)})
                remap = kwarg(node, "remappings")
                nodes.append({
                    "file": rel,
                    "line": node.lineno,
                    "package": render(kwarg(node, "package")) if kwarg(node, "package") else None,
                    "executable": render(kwarg(node, "executable")) if kwarg(node, "executable") else None,
                    "node_name": render(kwarg(node, "name")) if kwarg(node, "name") else None,
                    "namespace": render(kwarg(node, "namespace")) if kwarg(node, "namespace") else None,
                    "output": render(kwarg(node, "output")) if kwarg(node, "output") else None,
                    "parameters": entries,
                    "remappings": render(remap) if remap is not None else None,
                })
            elif fname == "LaunchDescription":
                pos = positionals(node)
                if pos:
                    try:
                        for elt in pos[0].elts:
                            active_exprs.append({
                                "file": rel,
                                "line": elt.lineno,
                                "expr": render(elt),
                                "commented_out": False,
                            })
                    except AttributeError:
                        active_exprs.append({"file": rel, "line": node.lineno,
                                             "expr": render(node), "commented_out": False})

    declared_args.sort(key=lambda d: (d["file"], d["line"]))
    nodes.sort(key=lambda d: (d["file"], d["line"]))
    active_exprs.sort(key=lambda d: (d["file"], d["line"]))

    ec.emit(
        args.out, "launch", args.module, args.root, GENERATOR,
        generator_note="基于 Python ast 解析，注释中的节点不会出现在结果中",
        files=files,
        declared_args=declared_args,
        nodes=nodes,
        active_in_launch_description=active_exprs,
        parse_errors=parse_errors,
    )
    print(f"[launch] 文件 {len(files)}，声明参数 {len(declared_args)}，"
          f"Node {len(nodes)}，LaunchDescription 条目 {len(active_exprs)} → {args.out}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
