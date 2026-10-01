#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MA_Nav 文档流水线 · 校验器注入测试

目的：证明 `verify_docs.py` **真的能拦住幻觉**，而不是只会输出 PASS。

做法：把模块文档复制到临时目录，注入三类**已知虚假事实**，再跑校验器：

  1. 假话题 `/cmd_vel`        —— 真值是 `/cmd_vel_chassis`，`/cmd_vel` 只存在于 YAML 注释
  2. 错行号 `...rog_map.cpp:9999` —— 该文件只有 272 行
  3. 编造符号 `rog_map::ROGMap::nonexistentMethod` —— 符号表里不存在

判定：

  - 原文档              → 必须 `errors == 0`
  - 注入后的副本        → 必须 `status == FAIL`，且上述三类检查**全部**出现
  - 两者都满足          → 本测试 PASS，退出码 0；否则退出码 1

用法::

    python3 docs/tools/test_injection.py --root .
"""

from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import subprocess
import sys

VERIFIER = "docs/tools/verify_docs.py"
WORKDIR = "docs/tools/_injection_tmp"

INJECT_SECTION = """
## 注入测试段落（本段仅存在于临时副本）

本节是**故意写入的假事实**，用于验证校验器能否拦住幻觉。

- 假话题：本模块发布点云话题 `/cmd_vel`，由 ros2 层直接创建（`src/ros2/src/ros2_node.cpp:98`）。
- 假行号：地图更新的实现在 `src/map/src/rog_map.cpp:9999`。
- 假符号：`rog_map::ROGMap::nonexistentMethod` 负责概率地图更新。
"""

EXPECTED = {
    "ros2.unknown_topic": "假话题 /cmd_vel 未被识破",
    "anchor.line_out_of_range": "越界行号 9999 未被识破",
    "symbol.unknown": "编造符号 nonexistentMethod 未被识破",
}


def run_verifier(root: pathlib.Path, docs: str, report: str) -> tuple[int, dict]:
    cmd = [
        sys.executable, str(root / VERIFIER),
        "--root", str(root),
        "--module", "map",
        "--docs", docs,
        "--evidence", "docs/_evidence",
        "--report", report,
    ]
    proc = subprocess.run(cmd, cwd=str(root), capture_output=True, text=True)
    report_path = root / report
    data = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}
    return proc.returncode, data


def main() -> int:
    ap = argparse.ArgumentParser(description="校验器注入测试")
    ap.add_argument("--root", default=".")
    ap.add_argument("--module", default="map")
    args = ap.parse_args()

    root = pathlib.Path(args.root).resolve()
    work = root / WORKDIR
    failures: list[str] = []

    # ---------------------------------------------------------- 1) 原文档
    rc_clean, clean = run_verifier(
        root, f"docs/nav/{args.module}", f"{WORKDIR}/verify.clean.json")
    n_clean_err = len(clean.get("errors", []))
    print(f"[1/2] 原文档：status={clean.get('status')} errors={n_clean_err} "
          f"warnings={len(clean.get('warnings', []))}")
    if n_clean_err != 0:
        failures.append(f"原文档本不应有 error，实际 {n_clean_err} 条")
    if clean.get("status") == "FAIL":
        failures.append("原文档被判 FAIL")

    # ---------------------------------------------------------- 2) 注入副本
    if work.exists():
        shutil.rmtree(work)
    dst = work / args.module
    shutil.copytree(root / "docs" / "nav" / args.module, dst)
    target = dst / "README.md"
    target.write_text(target.read_text(encoding="utf-8") + INJECT_SECTION, encoding="utf-8")

    rc_bad, bad = run_verifier(
        root, f"{WORKDIR}/{args.module}", f"{WORKDIR}/verify.injected.json")
    checks = {e["check"] for e in bad.get("errors", [])}
    print(f"[2/2] 注入副本：status={bad.get('status')} errors={len(bad.get('errors', []))}")
    for e in bad.get("errors", []):
        print(f"        {e['check']:26s} {e['doc']}:{e['line']} {e['message']}")

    if bad.get("status") != "FAIL":
        failures.append(f"注入副本未被判 FAIL，实际 status={bad.get('status')}")
    for check, why in EXPECTED.items():
        if check not in checks:
            failures.append(f"{why}（缺少检查 {check}）")
        else:
            print(f"       ✓ 已识破：{check}")

    # ---------------------------------------------------------- 清理与结论
    shutil.rmtree(work, ignore_errors=True)

    print()
    if failures:
        print("注入测试 FAIL：")
        for f in failures:
            print("  -", f)
        return 1
    print("注入测试 PASS：校验器对三类幻觉全部报错，且原文档 errors=0。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
