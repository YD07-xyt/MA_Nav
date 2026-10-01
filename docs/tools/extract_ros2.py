#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 证据提取器（ROS2 接口）

从 C++ 源码穷举 ROS2 接口面：publisher / subscription / service / client /
timer，并把话题名解析成**真值**：

  - 字面量话题名 → 直接取值
  - `config.X` 形式 → 通过 `X = yaml["a"]["b"]` 绑定反查 YAML 取值

这一步是防幻觉的关键：`config/planner.yaml` 的注释里存在 `#"/cmd_vel"`，
纯文本匹配会把注释当成真值；本工具只认**解析后的 YAML 值**。

用法::

    python3 docs/tools/extract_ros2.py --root <repo> --module ros2 \\
        --scan src/ros2 --binding-header src/ros2/include/ros2/config.hpp \\
        --yaml config/planner.yaml --out docs/_evidence/repo/ros2.json
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import evidence_common as ec  # noqa: E402

GENERATOR = "docs/tools/extract_ros2.py"

# create_xxx<TYPE>(...)
CALL_RE = re.compile(r"create_(\w+)\s*<([^<>]*)>\s*\(")
# declare_parameter<T>("name"[, default]) / get_parameter("name", var)
DECLARE_RE = re.compile(r"declare_parameter\s*<\s*([^<>]+?)\s*>\s*\(\s*\"([^\"]+)\"")
GET_RE = re.compile(r"get_parameter\s*\(\s*\"([^\"]+)\"\s*,\s*([^)]+)\)")

ROLE = {
    "publisher": "publisher",
    "subscription": "subscription",
    "service": "service",
    "client": "client",
    "wall_timer": "wall_timer",
    "timer": "timer",
}

LITERAL_RE = re.compile(r"^\"([^\"]*)\"$")
CONFIG_MEMBER_RE = re.compile(r"(?:\w+\.)*(\w+)\s*$")


def split_top_level(args: str) -> list[str]:
    return ec.split_top_level(args)


def balanced_args(text: str, open_paren: int) -> str:
    return ec.balanced_args(text, open_paren)


def resolve_name(expr: str, bindings: dict, yaml_values: dict) -> dict:
    """把话题/服务名实参解析为真值。"""
    expr = expr.strip()
    m = LITERAL_RE.match(expr)
    if m:
        return {"name": m.group(1), "resolved_from": "literal", "unresolved_expr": None}
    m = CONFIG_MEMBER_RE.match(expr)
    if m:
        member = m.group(1)
        b = bindings.get(member)
        if b:
            val = yaml_values.get(b["yaml_path"])
            if val is not None:
                return {
                    "name": val["value_repr"],
                    "resolved_from": f"{b['file']}:{b['line']} → {b['yaml_path']}",
                    "unresolved_expr": None,
                }
            return {"name": None, "resolved_from": None,
                    "unresolved_expr": expr, "why": f"yaml 键 {b['yaml_path']} 不存在"}
    return {"name": None, "resolved_from": None, "unresolved_expr": expr,
            "why": "非字面量且无 config 绑定"}


def main() -> int:
    ap = argparse.ArgumentParser(description="提取 ROS2 接口面证据")
    ap.add_argument("--root", required=True)
    ap.add_argument("--module", default="ros2")
    ap.add_argument("--scan", action="append", required=True, help="扫描目录（相对 root）")
    ap.add_argument("--binding-header", action="append", default=[],
                    help="提供 config[\"a\"][\"b\"] 绑定的头文件（相对 root）")
    ap.add_argument("--yaml", action="append", default=[], help="用于解析话题名的 YAML（相对 root）")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    # 1) YAML 真值（带行号）
    yaml_values: dict[str, dict] = {}
    yaml_files: list[dict] = []
    for rel in args.yaml:
        abs_p = pathlib.Path(args.root) / rel
        if not abs_p.is_file():
            print(f"警告: YAML 不存在 {rel}", file=sys.stderr)
            continue
        leaves = ec.yaml_leaves(str(abs_p))
        yaml_files.append({"file": rel, "leaves": len(leaves)})
        for leaf in leaves:
            yaml_values[leaf["path"]] = {**leaf, "file": rel}

    # 2) config 绑定
    header_texts = {rel: ec.read_text(args.root, rel) for rel in args.binding_header}
    bindings = {}
    for b in ec.extract_config_bindings(header_texts):
        bindings[b["cpp_member"]] = b

    # 3) 扫描 C++ 调用
    scan_files = ec.iter_module_files(args.root, args.scan)
    items: list[dict] = []
    ros_params: list[dict] = []
    call_re = CALL_RE
    for rel in scan_files:
        text = ec.read_text(args.root, rel)
        for m in call_re.finditer(text):
            role_raw, msg_type = m.group(1), m.group(2).strip()
            open_paren = m.end() - 1
            arg_str = balanced_args(text, open_paren)
            arglist = split_top_level(arg_str)
            line = text.count("\n", 0, m.start()) + 1
            entry = {
                "role": ROLE.get(role_raw, role_raw),
                "msg_type": msg_type,
                "file": rel,
                "line": line,
                "args": arglist,
            }
            if role_raw in ("publisher", "subscription", "service", "client"):
                resolved = resolve_name(arglist[0], bindings, yaml_values) if arglist else {
                    "name": None, "unresolved_expr": None, "why": "无实参", "resolved_from": None}
                entry.update(resolved)
                if role_raw in ("publisher", "subscription"):
                    qos = arglist[1] if len(arglist) > 1 else None
                    if qos is not None and re.fullmatch(r"\d+", qos):
                        entry["qos_depth"] = int(qos)
                        entry["qos_expr"] = None
                    else:
                        entry["qos_depth"] = None
                        entry["qos_expr"] = qos
                if role_raw == "subscription" and len(arglist) > 2:
                    entry["handler_expr"] = arglist[2]
            elif role_raw in ("wall_timer", "timer"):
                entry["period_expr"] = arglist[0] if arglist else None
                entry["handler_expr"] = arglist[1] if len(arglist) > 1 else None
            items.append(entry)

        for m in DECLARE_RE.finditer(text):
            ros_params.append({
                "kind": "declare_parameter",
                "type": m.group(1).strip(),
                "name": m.group(2),
                "file": rel,
                "line": text.count("\n", 0, m.start()) + 1,
            })
        for m in GET_RE.finditer(text):
            ros_params.append({
                "kind": "get_parameter",
                "type": None,
                "name": m.group(1),
                "target": m.group(2).strip(),
                "file": rel,
                "line": text.count("\n", 0, m.start()) + 1,
            })

    items.sort(key=lambda e: (e["file"], e["line"]))
    doc = ec.emit(
        args.out, "ros2", args.module, args.root, GENERATOR,
        scanned_files=scan_files,
        yaml_files=yaml_files,
        config_bindings=sorted(bindings.values(), key=lambda b: (b["file"], b["line"])),
        interfaces=items,
        ros_params=ros_params,
    )
    resolved = [i for i in items if i.get("name")]
    print(f"[ros2] 接口 {len(items)} 条，已解析名 {len(resolved)} 条，"
          f"未解析 {len(items) - len(resolved)} 条，ros_param {len(ros_params)} 条 → {args.out}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
