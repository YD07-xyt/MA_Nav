#!/usr/bin/env python3
"""离线复刻 terrain_analysis.cpp 的判据，对比上一版(v2) 与本版(v3)。

本版目标（对应用户反馈）：
  R1 限高杆下地面点少时仍能过滤限高杆；
  R2 台阶边沿不依赖台面一侧是否采到点，从低地一侧闭合；
  R3 台面一侧（台阶/高台顶面）不再被判成障碍。

输入模型：真实输入是 rog_map 3D 占据体素中心（0.05 m 分辨率），
本脚本把合成点云同样量化到体素中心，并模拟逐帧丢点后累积（rog_map 是累积地图）。

不参与构建，纯离线脚本。用法：python3 <本文件>
"""
import math
import random
from collections import defaultdict

RES = 0.05
N = 72                     # 3.6 m x 3.6 m 窗口（真实 10 m，局部模式一致，缩小只为跑得快）
GROUND_Z = 0.025           # 地面体素层中心
ROBOT_H = 0.24             # 与 config/map.yaml terrain/robot_height 一致
FRAMES = 10

# ---------------- 判据常量（与 terrain_analysis.cpp 对应） ----------------
K_PATCH_GAP = 0.12
K_GROUND_THICK = 0.12
K_OVH_SUP_MIN = 2
K_OVH_SUP_R = 2
K_OVH_SUP_TOL = 0.08
K_CLR_R = 2
K_CLR_Q = 0.75
K_CLR_BAND = 0.10
K_VERT_H = 0.12
K_MAX_CLIMB = 0.10

# v2（上一版）
V2_GROUNDED_TOL = 0.06
V2_GROUND_KERNEL = 3
V2_GROUND_Q = 0.25
V2_STEP_R = 2
V2_STEP_WIT = 2

# v3（本版）
V3_GROUNDED_TOL = 0.10
V3_GROUND_KERNEL = 7
V3_GROUND_Q = 0.10
V3_PLATEAU_R = 5
V3_PLATEAU_TOL = 0.05
V3_STEP_LEVEL_DIFF = K_MAX_CLIMB + 0.03
V3_RING_VERT_R = 1
V3_RING_LEVEL_R = 4

NAN = float("nan")


def isnan(v):
    return v != v


def quantize(pts):
    """量化到 rog_map 3D 占据体素中心。"""
    out = []
    for (x, y, z) in pts:
        out.append((
            (math.floor(x / RES) + 0.5) * RES,
            (math.floor(y / RES) + 0.5) * RES,
            (math.floor(z / RES) + 0.5) * RES,
        ))
    return out


def build_patches(pts):
    grid = defaultdict(list)
    for (x, y, z) in pts:
        gx, gy = int(math.floor(x / RES)), int(math.floor(y / RES))
        if 0 <= gx < N and 0 <= gy < N:
            grid[(gx, gy)].append(z)
    patches = {}
    for k, zs in grid.items():
        zs.sort()
        lst = []
        lo = hi = zs[0]
        for z in zs[1:]:
            if z - hi > K_PATCH_GAP:
                lst.append((lo, hi))
                lo = hi = z
            elif z > hi:
                hi = z
        lst.append((lo, hi))
        patches[k] = lst
    return patches


def quantile_filter(cand, kernel, q, keys=None):
    # C++ 里 z_ground 是满网格：本身没有地面候选的柱（墙/横梁/台面边）也要取得邻域地面。
    out = {}
    half = max(1, kernel // 2)
    for (gx, gy) in (keys if keys is not None else cand):
        vals = []
        for m in range(-half, half + 1):
            for n in range(-half, half + 1):
                v = cand.get((gx + m, gy + n))
                if v is not None and not isnan(v):
                    vals.append(v)
        if vals:
            vals.sort()
            out[(gx, gy)] = vals[int(q * (len(vals) - 1))]
    return out


def ground_field(patches, kernel, q):
    cand = {}
    for k, lst in patches.items():
        if lst and lst[0][1] - lst[0][0] <= K_GROUND_THICK:
            cand[k] = lst[0][1]
    return quantile_filter(cand, kernel, q, keys=patches.keys())


def cluster_quantile(values_by_cell, radius, q, band):
    out = dict(values_by_cell)
    for k, c in values_by_cell.items():
        gx, gy = k
        vals = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nk = (gx + dx, gy + dy)
                if nk in values_by_cell and abs(values_by_cell[nk] - c) <= band:
                    vals.append(values_by_cell[nk])
        if vals:
            vals.sort()
            out[k] = vals[int(q * (len(vals) - 1))]
    return out


def canopy_obstacle(k, patches, elev_bottom, clearance_agg):
    """悬空/抬高结构：净空 + 同层横向连续。返回 True=障碍。"""
    if clearance_agg[k] < ROBOT_H:
        return True
    pb = elev_bottom[k]
    gx, gy = k
    support = 0
    for dy in range(-K_OVH_SUP_R, K_OVH_SUP_R + 1):
        if support >= K_OVH_SUP_MIN:
            break
        for dx in range(-K_OVH_SUP_R, K_OVH_SUP_R + 1):
            if dx == 0 and dy == 0:
                continue
            for (qb, qt) in patches.get((gx + dx, gy + dy), []):
                if qb <= pb + K_OVH_SUP_TOL and qt >= pb - K_OVH_SUP_TOL:
                    support += 1
                    break
            if support >= K_OVH_SUP_MIN:
                break
    return support < K_OVH_SUP_MIN


# ---------------- v2：上一版（本次修改前） ----------------
def decide_v2(patches):
    g = ground_field(patches, V2_GROUND_KERNEL, V2_GROUND_Q)
    surface, has_ovh, clearance, lowest = {}, {}, {}, {}
    for k, lst in patches.items():
        gk = g.get(k, NAN)
        if isnan(gk) or not lst:
            continue
        lo = lst[0]
        thin = lo[1] - lo[0] <= K_GROUND_THICK
        grounded = lo[0] <= gk + V2_GROUNDED_TOL
        if thin and grounded:
            surface[k] = lo[1]
        for i, (b, t) in enumerate(lst):
            if b <= gk + V2_GROUNDED_TOL:
                continue
            has_ovh[k] = 1
            lowest[k] = i
            clearance[k] = b - gk
            break
    clr_agg = cluster_quantile(clearance, K_CLR_R, K_CLR_Q, K_CLR_BAND)
    elev_bottom = {k: patches[k][lowest[k]][0] for k in lowest}

    obst = set()
    for k, lst in patches.items():
        gk = g.get(k, NAN)
        if isnan(gk) or not lst:
            continue
        lo = lst[0]
        grounded = lo[0] <= gk + V2_GROUNDED_TOL
        obstacle = False
        if grounded and (lo[1] - gk) >= K_VERT_H:
            obstacle = True
        elif k in has_ovh:
            obstacle = canopy_obstacle(k, patches, elev_bottom, clr_agg)
        if not obstacle and k in surface:
            s = surface[k]
            lower = 0
            gx, gy = k
            for dy in range(-V2_STEP_R, V2_STEP_R + 1):
                if lower >= V2_STEP_WIT:
                    break
                for dx in range(-V2_STEP_R, V2_STEP_R + 1):
                    if dx == 0 and dy == 0:
                        continue
                    nk = (gx + dx, gy + dy)
                    sn = surface.get(nk, NAN)
                    if isnan(sn) or sn >= s - K_MAX_CLIMB:
                        continue
                    if nk in has_ovh and clr_agg.get(nk, 1e9) < ROBOT_H:
                        continue
                    lower += 1
                    if lower >= V2_STEP_WIT:
                        break
            obstacle = lower >= V2_STEP_WIT
        if obstacle:
            obst.add(k)
    return obst


# ---------------- v3：本版 ----------------
def decide_v3(patches):
    g = ground_field(patches, V3_GROUND_KERNEL, V3_GROUND_Q)
    landing, vertical, elevated, plateau = {}, {}, {}, {}
    landing_level, clearance, lowest = {}, {}, {}
    for k, lst in patches.items():
        gk = g.get(k, NAN)
        if isnan(gk) or not lst:
            continue
        lo = lst[0]
        grounded = lo[0] <= gk + V3_GROUNDED_TOL
        if grounded:
            if lo[1] - lo[0] <= K_GROUND_THICK:
                landing[k] = 1
                landing_level[k] = (lo[0] + lo[1]) * 0.5
            if lo[1] - gk >= K_VERT_H:
                vertical[k] = 1
        for i, (b, t) in enumerate(lst):
            if b <= gk + V3_GROUNDED_TOL:
                continue
            elevated[k] = 1
            lowest[k] = i
            clearance[k] = b - gk
            break

    # 抬高片标高与邻域可落脚面一致 → 可通行台面
    for k in elevated:
        gk = g[k]
        for (b, t) in patches[k]:
            if b <= gk + V3_GROUNDED_TOL:
                continue
            center = (b + t) * 0.5
            gx, gy = k
            found = False
            for dy in range(-V3_PLATEAU_R, V3_PLATEAU_R + 1):
                for dx in range(-V3_PLATEAU_R, V3_PLATEAU_R + 1):
                    nk = (gx + dx, gy + dy)
                    if nk in landing and abs(landing_level[nk] - center) <= V3_PLATEAU_TOL:
                        found = True
                        break
                if found:
                    break
            if found:
                plateau[k] = 1
                break

    # 台阶立面：竖直结构顶面标高与邻域可落脚面一致（墙/立柱顶面没有同层可落脚面）
    vert_step = set()
    for k in vertical:
        top = patches[k][-1][1]
        gx, gy = k
        found = False
        for dy in range(-V3_PLATEAU_R, V3_PLATEAU_R + 1):
            for dx in range(-V3_PLATEAU_R, V3_PLATEAU_R + 1):
                nk = (gx + dx, gy + dy)
                if nk in landing and abs(landing_level[nk] - top) <= V3_PLATEAU_TOL:
                    found = True
                    break
            if found:
                break
        if found:
            vert_step.add(k)

    clr_agg = cluster_quantile(clearance, K_CLR_R, K_CLR_Q, K_CLR_BAND)
    elev_bottom = {k: patches[k][lowest[k]][0] for k in lowest}

    obst = set()
    for k, lst in patches.items():
        if k not in g:
            continue
        obstacle = False
        if k in vertical:
            obstacle = True
        elif k in landing:
            lvl = landing_level[k]
            gx, gy = k
            # (i) 低处一侧有竖直结构，顶面高出本落脚面
            for dy in range(-V3_RING_VERT_R, V3_RING_VERT_R + 1):
                if obstacle:
                    break
                for dx in range(-V3_RING_VERT_R, V3_RING_VERT_R + 1):
                    if dx == 0 and dy == 0:
                        continue
                    nk = (gx + dx, gy + dy)
                    if nk in vert_step and patches[nk][-1][1] >= lvl + V3_STEP_LEVEL_DIFF:
                        obstacle = True
                        break
            # (ii) 低处一侧有更高的可落脚面
            for dy in range(-V3_RING_LEVEL_R, V3_RING_LEVEL_R + 1):
                if obstacle:
                    break
                for dx in range(-V3_RING_LEVEL_R, V3_RING_LEVEL_R + 1):
                    if dx == 0 and dy == 0:
                        continue
                    nk = (gx + dx, gy + dy)
                    if nk in landing and landing_level[nk] >= lvl + V3_STEP_LEVEL_DIFF:
                        obstacle = True
                        break
        if not obstacle and k in elevated and k not in plateau:
            obstacle = canopy_obstacle(k, patches, elev_bottom, clr_agg)
        if obstacle:
            obst.add(k)
    return obst


# ---------------- 场景 ----------------
PLAT_X = 1.60
BAR_X0, BAR_X1, BAR_Y0 = 0.40, 1.00, 0.20
TUN_X0, TUN_X1, TUN_Y0, TUN_Y1 = 0.40, 1.00, 1.90, 2.30
RAMP_X0, RAMP_Y0, RAMP_Y1 = 2.00, 1.90, 2.30
POLE_XY = (2.30, 1.00)


def layer(h):
    """物理高度 → 体素层中心（台阶/横梁高度对齐到 0.05 体素层）。"""
    return GROUND_Z + round(h / RES) * RES


def scene(seed, bar_w=0.20, p_under=0.35, step_h=0.20, q_top=1.0, riser=True,
          ramp=True, tunnel=True, wall=False, ceiling_clearance=None, platform=True):
    rnd = random.Random(seed)
    pts = []
    top_z = layer(step_h)
    bar_z = GROUND_Z + round(0.25 / RES) * RES

    def jit(s=0.008):
        return rnd.gauss(0.0, s)

    def surf(x, y):
        if ramp and RAMP_Y0 <= y <= RAMP_Y1 and x >= RAMP_X0:
            return top_z + 0.25 * (x - RAMP_X0)
        return top_z if (platform and x >= PLAT_X) else GROUND_Z

    by0, by1 = BAR_Y0, BAR_Y0 + bar_w
    for gy in range(N):
        for gx in range(N):
            cx, cy = (gx + rnd.random()) * RES, (gy + rnd.random()) * RES
            in_bar = BAR_X0 <= cx <= BAR_X1 and by0 <= cy <= by1
            if in_bar and rnd.random() >= p_under:
                continue
            if platform and cx >= PLAT_X and cx <= PLAT_X + 0.15 and rnd.random() >= q_top:
                continue
            if rnd.random() < 0.20:
                continue
            for _ in range(rnd.choice([1, 2, 2, 3])):
                pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02),
                            surf(cx, cy) + jit()))
    # 台阶立面（从低地一侧可见）
    if riser:
        for gy in range(N):
            cy = (gy + 0.5) * RES
            if rnd.random() < 0.10:
                continue
            z = GROUND_Z
            while z < top_z - 0.01:
                pts.append((PLAT_X - 0.005 + rnd.uniform(-0.008, 0.008),
                            cy + rnd.uniform(-0.02, 0.02), z))
                z += rnd.uniform(0.04, 0.06)
    # 限高杆横梁（净空 0.25，对齐体素层）
    for gx in range(N):
        for gy in range(N):
            cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
            if not (BAR_X0 <= cx <= BAR_X1 and by0 <= cy <= by1):
                continue
            for _ in range(rnd.choice([1, 2, 3])):
                pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02),
                            bar_z + jit(0.006)))
    for (px, py) in [(BAR_X0, by0), (BAR_X0, by1), (BAR_X1, by0), (BAR_X1, by1)]:
        z = GROUND_Z
        while z <= bar_z + 0.02:
            pts.append((px + rnd.uniform(-0.015, 0.015), py + rnd.uniform(-0.015, 0.015), z))
            z += 0.02
    if tunnel:
        for gx in range(N):
            for gy in range(N):
                cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
                if not (TUN_X0 <= cx <= TUN_X1 and TUN_Y0 <= cy <= TUN_Y1):
                    continue
                for _ in range(rnd.choice([1, 2])):
                    pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02),
                                layer(2.0) + jit(0.006)))
    # 细杆
    z = GROUND_Z + 0.20
    while z <= GROUND_Z + 1.20:
        if rnd.random() < 0.85:
            pts.append((POLE_XY[0] + rnd.uniform(-0.015, 0.015),
                        POLE_XY[1] + rnd.uniform(-0.015, 0.015), z))
        z += rnd.uniform(0.16, 0.24)
    # 细长矮墙（验证竖直结构判据没退化）
    if wall:
        for gx in range(N):
            cx = (gx + 0.5) * RES
            if not (0.30 <= cx <= 1.30):
                continue
            z = GROUND_Z
            while z <= layer(0.40):
                pts.append((cx + rnd.uniform(-0.01, 0.01), 3.05 + rnd.uniform(-0.01, 0.01), z))
                z += 0.03
    # 低顶棚专项：净空对齐体素层
    if ceiling_clearance is not None:
        ceil_z = GROUND_Z + round(ceiling_clearance / RES) * RES
        for gy in range(N):
            for gx in range(N):
                cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
                if 1.00 <= cx <= 1.60 and 1.00 <= cy <= 1.40:
                    for _ in range(2):
                        pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02),
                                    ceil_z + jit(0.006)))
    return pts


def accumulated(seed_base, frames=FRAMES, **kw):
    """逐帧累积（模拟 rog_map 累积地图）。"""
    pts = []
    for f in range(frames):
        pts += scene(seed_base + f, **kw)
    return quantize(pts)


# ---------------- 指标 ----------------
def frame_maps(seed_base, frames=FRAMES, **kw):
    """返回 (v2 障碍集合列表, v3 障碍集合列表)，每帧用"截至该帧的累积地图"。"""
    v2s, v3s = [], []
    pts = []
    for f in range(frames):
        pts += scene(seed_base + f, **kw)
        pat = build_patches(quantize(pts))
        v2s.append(decide_v2(pat))
        v3s.append(decide_v3(pat))
    return v2s, v3s


def cell_xy(k):
    return (k[0] + 0.5) * RES, (k[1] + 0.5) * RES


def bar_white(obst, by0, by1):
    """梁内（四角立柱内缩 0.06）的障碍格数。"""
    n = 0
    for k in obst:
        cx, cy = cell_xy(k)
        if BAR_X0 + 0.06 <= cx <= BAR_X1 - 0.06 and by0 + 0.06 <= cy <= by1 - 0.06:
            n += 1
    return n


def bar_flip(sets, by0, by1):
    """梁内格 10 帧内既有障碍又有空闲的格数。"""
    seq = defaultdict(int)
    for s in sets:
        for k in s:
            cx, cy = cell_xy(k)
            if BAR_X0 + 0.06 <= cx <= BAR_X1 - 0.06 and by0 + 0.06 <= cy <= by1 - 0.06:
                seq[k] += 1
    return sum(1 for c in seq.values() if c != len(sets))


def flip_count(sets):
    seq = defaultdict(int)
    for s in sets:
        for k in s:
            seq[k] += 1
    return sum(1 for c in seq.values() if c != len(sets))


def region_counts(sets, region_fn):
    """返回 (格数, 障碍帧次, 翻转格)。"""
    cells = set()
    for s in sets:
        cells |= s
    # 区域格数按网格统计
    n = 0
    for gx in range(N):
        for gy in range(N):
            if region_fn(*cell_xy((gx, gy))):
                n += 1
    occ = sum(1 for s in sets for k in s if region_fn(*cell_xy(k)))
    flip = 0
    seq = defaultdict(int)
    for s in sets:
        for k in s:
            if region_fn(*cell_xy(k)):
                seq[k] += 1
    for k, c in seq.items():
        if c != len(sets):
            flip += 1
    return n, occ, flip


def region_fn_main(by0, by1):
    def fn(cx, cy):
        if BAR_X0 <= cx <= BAR_X1 and by0 <= cy <= by1:
            return "横梁下"
        if TUN_X0 <= cx <= TUN_X1 and TUN_Y0 <= cy <= TUN_Y1:
            return "隧道顶下"
        if abs(cx - POLE_XY[0]) <= 0.07 and abs(cy - POLE_XY[1]) <= 0.07:
            return "细杆"
        for (bx, by) in [(BAR_X0, by0), (BAR_X0, by1), (BAR_X1, by0), (BAR_X1, by1)]:
            if abs(cx - bx) <= 0.12 and abs(cy - by) <= 0.12:
                return "限高杆立柱"
        if abs(cx - PLAT_X) <= 0.10 and cy < RAMP_Y0:
            return "台阶边沿带"
        if PLAT_X + 0.10 < cx <= 3.2 and cy < RAMP_Y0 - 0.10:
            return "台面(内)"
        if RAMP_X0 + 0.15 <= cx <= 3.0 and RAMP_Y0 + 0.13 <= cy <= RAMP_Y1 - 0.13:
            return "坡道面"
        if 1.85 <= cx <= 3.0 and 0.55 <= cy <= 1.80:
            return "台面(内)"
        if 1.90 <= cx <= 2.10 and RAMP_Y0 - 0.15 <= cy <= RAMP_Y1 + 0.15:
            return "坡道↔台面接缝"
        if cx < PLAT_X - 0.28 and cy < RAMP_Y0 - 0.28:
            return "平地"
        return "其他"
    return fn


print("=" * 100)
print("A. 限高杆：梁下地面点概率 p_under / 横梁宽度 bar_w 变化（累积地图 10 帧内逐帧最大）")
print("=" * 100)
print(f"{'场景':<30}{'旧:梁下白(逐帧max)':>20}{'新:梁下白(逐帧max)':>20}"
      f"{'旧:翻转格':>10}{'新:翻转格':>10}")
by0_default = BAR_Y0
for bar_w, p_under, name in [
    (0.10, 0.35, "bar_w=0.10 p_under=0.35"),
    (0.20, 0.35, "bar_w=0.20 p_under=0.35"),
    (0.20, 0.05, "bar_w=0.20 p_under=0.05"),
    (0.20, 0.00, "bar_w=0.20 p_under=0.00"),
    (0.25, 0.00, "bar_w=0.25 p_under=0.00"),
]:
    v2s, v3s = frame_maps(700, bar_w=bar_w, p_under=p_under)
    by1 = by0_default + bar_w
    m2 = max(bar_white(s, by0_default, by1) for s in v2s)
    m3 = max(bar_white(s, by0_default, by1) for s in v3s)
    print(f"{name:<30}{m2:>20}{m3:>20}"
          f"{bar_flip(v2s, by0_default, by1):>10}{bar_flip(v3s, by0_default, by1):>10}")

print()
print("=" * 100)
print("B. 台阶：高度 / 台面靠边点概率 q_top / 有无立面点（累积地图 10 帧内逐帧）")
print("   '断行' = 边沿 x∈[E-0.10,E+0.05] 没有障碍的行数；'台面白' = x>E+0.05 的白格（末帧）")
print("   H=0.10 是可攀爬台阶，断行 37 = 完全不判障碍（期望行为），看'台面白/低侧白'=0 即正确")
print("=" * 100)
print(f"{'场景':<26}{'旧断行(末帧)':>12}{'新断行(末帧)':>12}{'旧断行max':>10}{'新断行max':>10}"
      f"{'旧台面白':>9}{'新台面白':>9}{'旧低侧白':>9}{'新低侧白':>9}")
rows_total = int(RAMP_Y0 / RES)
for step_h, q_top, riser, name in [
    (0.10, 1.0, True, "H=0.10(可爬)"),
    (0.15, 1.0, True, "H=0.15"),
    (0.20, 1.0, True, "H=0.20"),
    (0.20, 0.3, True, "H=0.20 q_top=0.3"),
    (0.50, 1.0, True, "H=0.50"),
    (0.50, 0.3, True, "H=0.50 q_top=0.3"),
    (0.20, 1.0, False, "H=0.20 无立面点"),
    (0.50, 1.0, False, "H=0.50 无立面点"),
]:
    v2s, v3s = frame_maps(1900, step_h=step_h, q_top=q_top, riser=riser)

    def metrics(s):
        gap = 0
        for gy in range(rows_total):
            hit = any((gx, gy) in s for gx in range(N)
                      if PLAT_X - 0.10 <= (gx + 0.5) * RES <= PLAT_X + 0.05)
            gap += 0 if hit else 1
        top = sum(1 for k in s if PLAT_X + 0.05 < cell_xy(k)[0]
                  and cell_xy(k)[1] < RAMP_Y0 - 0.10)
        low = sum(1 for k in s if PLAT_X - 0.15 <= cell_xy(k)[0] <= PLAT_X - 0.02
                  and cell_xy(k)[1] < RAMP_Y0 - 0.10)
        return gap, top, low

    gap2, top2, low2 = metrics(v2s[-1])
    gap3, top3, low3 = metrics(v3s[-1])
    gap2_max = max(metrics(s)[0] for s in v2s)
    gap3_max = max(metrics(s)[0] for s in v3s)
    print(f"{name:<26}{gap2:>12}{gap3:>12}{gap2_max:>10}{gap3_max:>10}"
          f"{top2:>9}{top3:>9}{low2:>9}{low3:>9}")

print()
print("=" * 100)
print("C. 全场区域表（基准场：20cm 台阶 + 0.25m 净空限高杆 + 2m 隧道顶 + 坡道 + 细杆 + 矮墙）")
print("   障碍帧次 = 区域所有格 10 帧内判障碍次数之和；翻转格 = 10 帧内既障碍又空闲的格")
print("=" * 100)
v2s, v3s = frame_maps(700, wall=True)
fn = region_fn_main(BAR_Y0, BAR_Y0 + 0.20)
regions = ["平地", "坡道面", "坡道↔台面接缝", "台面(内)", "台阶边沿带", "限高杆立柱",
           "横梁下", "隧道顶下", "细杆"]
print(f"{'区域':<18}{'格数':>6}{'旧:障碍帧次':>14}{'旧:翻转格':>10}{'新:障碍帧次':>14}{'新:翻转格':>10}")
for r in regions:
    n = occ2 = flip2 = 0
    n, occ2, flip2 = region_counts(v2s, lambda cx, cy, r=r: fn(cx, cy) == r)
    _, occ3, flip3 = region_counts(v3s, lambda cx, cy, r=r: fn(cx, cy) == r)
    print(f"{r:<18}{n:>6}{occ2:>14}{flip2:>10}{occ3:>14}{flip3:>10}")

print()
print("=" * 100)
print("D. 闭环 / 连通性专项（逐帧最大值）")
print("=" * 100)


def edge_gap_rows(obst):
    gaps = 0
    for gy in range(rows_total):
        hit = any((gx, gy) in obst for gx in range(N)
                  if PLAT_X - 0.10 <= (gx + 0.5) * RES <= PLAT_X + 0.05)
        if not hit:
            gaps += 1
    return gaps


def beam_corridor(obst, by0, by1):
    return sum(1 for k in obst
               if BAR_X0 + 0.10 <= cell_xy(k)[0] <= BAR_X1 - 0.10
               and by0 + 0.10 <= cell_xy(k)[1] <= by1 - 0.10)


def ramp_junction(obst):
    return sum(1 for k in obst
               if 1.90 <= cell_xy(k)[0] <= 2.10
               and RAMP_Y0 - 0.15 <= cell_xy(k)[1] <= RAMP_Y1 + 0.15)


v2s, v3s = frame_maps(700)
print(f"{'指标':<40}{'旧':>10}{'新':>10}")
print(f"{'20cm 台阶边沿断裂行(末帧)':<38}"
      f"{edge_gap_rows(v2s[-1]):>10}{edge_gap_rows(v3s[-1]):>10}")
print(f"{'20cm 台阶边沿断裂行(逐帧max)':<36}"
      f"{max(edge_gap_rows(s) for s in v2s):>10}{max(edge_gap_rows(s) for s in v3s):>10}")
print(f"{'限高杆横梁正下方障碍格(末帧)':<36}"
      f"{beam_corridor(v2s[-1], BAR_Y0, BAR_Y0+0.20):>10}"
      f"{beam_corridor(v3s[-1], BAR_Y0, BAR_Y0+0.20):>10}")
print(f"{'坡道↔台面接缝障碍格(末帧)':<36}"
      f"{ramp_junction(v2s[-1]):>10}{ramp_junction(v3s[-1]):>10}")

print()
print("=" * 100)
print("E. 低顶棚（横梁下保留地面点）：净空 < robot_height 应挡、>= robot_height 应通行")
print("=" * 100)
print(f"{'净空':<12}{'旧:顶棚下白格':>16}{'新:顶棚下白格':>16}")
for clearance in (0.15, 0.20, 0.25, 0.30):
    v2s, v3s = frame_maps(900, ceiling_clearance=clearance, tunnel=False, ramp=False,
                          riser=False, p_under=1.0, step_h=0.20, platform=False)
    n2 = max(sum(1 for k in s if 1.02 <= cell_xy(k)[0] <= 1.58
                 and 1.02 <= cell_xy(k)[1] <= 1.38) for s in v2s)
    n3 = max(sum(1 for k in s if 1.02 <= cell_xy(k)[0] <= 1.58
                 and 1.02 <= cell_xy(k)[1] <= 1.38) for s in v3s)
    print(f"{clearance:<12.2f}{n2:>16}{n3:>16}")

print()
print("=" * 100)
print("F. 可攀爬 10cm 台阶 / 细长矮墙（不应回退）")
print("=" * 100)
v2s, v3s = frame_maps(1900, step_h=0.10)
print(f"10cm 台阶（可爬）：旧台面白格(末帧)="
      f"{sum(1 for k in v2s[-1] if cell_xy(k)[0] > PLAT_X + 0.05 and cell_xy(k)[1] < RAMP_Y0 - 0.10)}"
      f"  新={sum(1 for k in v3s[-1] if cell_xy(k)[0] > PLAT_X + 0.05 and cell_xy(k)[1] < RAMP_Y0 - 0.10)}")
pts = []
for f in range(FRAMES):
    pts += scene(700 + f, wall=True)
pat = build_patches(quantize(pts))
s2, s3 = decide_v2(pat), decide_v3(pat)
wall2 = sum(1 for k in s2 if 0.30 <= cell_xy(k)[0] <= 1.30 and 2.95 <= cell_xy(k)[1] <= 3.15)
wall3 = sum(1 for k in s3 if 0.30 <= cell_xy(k)[0] <= 1.30 and 2.95 <= cell_xy(k)[1] <= 3.15)
print(f"细长矮墙(末帧白格)：旧={wall2}  新={wall3}（应 >0，墙仍判障碍）")
