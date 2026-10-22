#!/usr/bin/env python3
"""离线复刻 terrain_analysis.cpp 的判据，用合成点云验证：
   1) 20cm 台阶边沿闭合
   2) 限高杆在不同帧之间不再翻转（旧逻辑会翻转）
   3) 平地/坡道/高台内部/隧道不被误判
"""
import math
import random
from collections import defaultdict

RES = 0.05
WIN = 3.0
NX = int(math.ceil(WIN / RES))
NY = int(math.ceil(WIN / RES))
KERNEL = 3
ROBOT_H = 0.24

K_PATCH_GAP = 0.12
K_GROUND_THICK = 0.12
K_GROUNDED_TOL = 0.06
K_VERT_H = 0.12
K_OVH_SUP_MIN = 2
K_OVH_SUP_R = 2
K_OVH_SUP_TOL = 0.08
K_CLR_R = 2
K_CLR_Q = 0.75
K_CLR_BAND = 0.10
K_MAX_CLIMB = 0.10
K_STEP_R = 2
K_STEP_WIT = 2

NAN = float("nan")


def isnan(v):
    return v != v


# ---------------- 场景 ----------------
# 平地 z=0；高台 x>=1.60, z=0.20（立面在 x=1.60）
# 限高杆：横梁 x∈[0.40,1.00], y∈[0.20,0.40], z∈[0.25,0.29]，四角立柱
# 隧道顶：x∈[0.40,1.00], y∈[2.00,2.60], z∈[2.00,2.10]
# 坡道：x∈[2.00,3.00], y∈[2.00,2.60], z=0.20+0.25*(x-2.00)（从台面起坡）
# 细杆：(2.50,1.00) 稀疏竖直点
PLAT_X = 1.60
BAR_X0, BAR_X1, BAR_Y0, BAR_Y1 = 0.40, 1.00, 0.20, 0.40
BAR_Z0, BAR_Z1 = 0.25, 0.29
TUN_X0, TUN_X1, TUN_Y0, TUN_Y1 = 0.40, 1.00, 2.00, 2.60
TUN_Z0, TUN_Z1 = 2.00, 2.10
RAMP_X0, RAMP_Y0, RAMP_Y1 = 2.00, 2.00, 2.60
RAMP_RISE = 0.25
POLE_XY = (2.50, 1.00)


def surf_z(x, y):
    if RAMP_Y0 <= y <= RAMP_Y1 and x >= RAMP_X0:
        return [0.20 + RAMP_RISE * (x - RAMP_X0)]
    return [0.20 if x >= PLAT_X else 0.0]


def gen_frame(seed):
    """一帧点云：(x,y,z) 列表，含表面抖动、丢点。"""
    rnd = random.Random(seed)
    pts = []

    def jit(s=0.010):
        return rnd.gauss(0.0, s)

    # 地面 / 台面 / 坡面：每格 1~3 个点，20% 丢格
    for gy in range(NY):
        for gx in range(NX):
            cx = (gx + rnd.random()) * RES
            cy = (gy + rnd.random()) * RES
            if rnd.random() < 0.20:
                continue
            for z in surf_z(cx, cy):
                for _ in range(rnd.choice([1, 2, 2, 3])):
                    pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02), z + jit()))

    # 20cm 台阶立面
    for gy in range(NY):
        cy = (gy + 0.5) * RES
        if rnd.random() < 0.10:
            continue
        z = 0.02
        while z < 0.20:
            pts.append((PLAT_X - 0.005 + rnd.uniform(-0.01, 0.01), cy + rnd.uniform(-0.02, 0.02), z))
            z += rnd.uniform(0.03, 0.07)

    # 限高杆横梁；梁下偶尔采到地面点（旧逻辑翻转的来源）
    for gx in range(NX):
        for gy in range(NY):
            cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
            if not (BAR_X0 <= cx <= BAR_X1 and BAR_Y0 <= cy <= BAR_Y1):
                continue
            for _ in range(rnd.choice([1, 2, 3])):
                z = rnd.choice([BAR_Z0, BAR_Z1, rnd.uniform(BAR_Z0, BAR_Z1)])
                pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02), z + jit(0.006)))
            if rnd.random() < 0.35:
                pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02), 0.0 + jit()))
    for (px, py) in [(BAR_X0, BAR_Y0), (BAR_X0, BAR_Y1), (BAR_X1, BAR_Y0), (BAR_X1, BAR_Y1)]:
        z = 0.0
        while z <= 0.32:
            pts.append((px + rnd.uniform(-0.015, 0.015), py + rnd.uniform(-0.015, 0.015), z))
            z += 0.02

    # 隧道顶
    for gx in range(NX):
        for gy in range(NY):
            cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
            if not (TUN_X0 <= cx <= TUN_X1 and TUN_Y0 <= cy <= TUN_Y1):
                continue
            for _ in range(rnd.choice([1, 2])):
                z = rnd.choice([TUN_Z0, TUN_Z1])
                pts.append((cx + rnd.uniform(-0.02, 0.02), cy + rnd.uniform(-0.02, 0.02), z + jit(0.006)))

    # 细杆：稀疏竖直点
    z = 0.20
    while z <= 1.20:
        if rnd.random() < 0.85:
            pts.append((POLE_XY[0] + rnd.uniform(-0.015, 0.015), POLE_XY[1] + rnd.uniform(-0.015, 0.015), z))
        z += rnd.uniform(0.16, 0.24)

    return pts


def world_to_grid(x, y):
    gx, gy = int(math.floor(x / RES)), int(math.floor(y / RES))
    if 0 <= gx < NX and 0 <= gy < NY:
        return gx, gy
    return None


def build_cells(pts):
    cells = [[[] for _ in range(NY)] for _ in range(NX)]
    for (x, y, z) in pts:
        idx = world_to_grid(x, y)
        if idx is None:
            continue
        cells[idx[0]][idx[1]].append(z)
    return cells


def build_patches(cells):
    patches = [[[] for _ in range(NY)] for _ in range(NX)]
    for gx in range(NX):
        for gy in range(NY):
            zs = sorted(cells[gx][gy])
            if not zs:
                continue
            out = []
            cur = [zs[0], zs[0]]
            for z in zs:
                if z - cur[1] > K_PATCH_GAP:
                    out.append((cur[0], cur[1]))
                    cur = [z, z]
                elif z > cur[1]:
                    cur[1] = z
            out.append((cur[0], cur[1]))
            patches[gx][gy] = out
    return patches


def quantile_filter(cand, k, q):
    out = [[NAN] * NY for _ in range(NX)]
    half = max(1, k // 2)
    for gx in range(NX):
        for gy in range(NY):
            vals = []
            for m in range(-half, half + 1):
                for n in range(-half, half + 1):
                    nx, ny = gx + m, gy + n
                    if 0 <= nx < NX and 0 <= ny < NY and not isnan(cand[nx][ny]):
                        vals.append(cand[nx][ny])
            if vals:
                vals.sort()
                out[gx][gy] = vals[int(q * (len(vals) - 1))]
    return out


def build_ground(patches):
    cand = [[NAN] * NY for _ in range(NX)]
    for gx in range(NX):
        for gy in range(NY):
            lst = patches[gx][gy]
            if not lst:
                continue
            lo = lst[0]
            if lo[1] - lo[0] <= K_GROUND_THICK:
                cand[gx][gy] = lo[1]
    return quantile_filter(cand, KERNEL, 0.25)


# ---------------- 旧逻辑（上一版） ----------------
def decide_old(patches, ground):
    obst = set()
    for gx in range(NX):
        for gy in range(NY):
            lst = patches[gx][gy]
            if not lst:
                continue
            g = ground[gx][gy]
            if isnan(g):
                continue
            low = lst[0]
            grounded = low[0] <= g + K_GROUNDED_TOL
            max_z = lst[-1][1]
            if grounded and (max_z - g) >= K_VERT_H:
                max_gap, last_top = 0.0, g
                for (b, t) in lst:
                    if b > last_top:
                        max_gap = max(max_gap, b - last_top)
                    if t > last_top:
                        last_top = t
                if max_gap < ROBOT_H:
                    obst.add((gx, gy))
                    continue
            blocked, undecided = False, False
            for (b, t) in lst:
                if b <= g + K_GROUNDED_TOL:
                    continue
                if b < g + 0.12:
                    blocked = True
                    break
                if b < g + ROBOT_H:
                    undecided = True
                    break
            if blocked:
                obst.add((gx, gy))
                continue
            if undecided:
                continue
            isolated = False
            for (b, t) in lst:
                if b <= g + K_GROUNDED_TOL:
                    continue
                support = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = gx + dx, gy + dy
                        if not (0 <= nx < NX and 0 <= ny < NY):
                            continue
                        for (qb, qt) in patches[nx][ny]:
                            if abs(qb - b) <= K_OVH_SUP_TOL:
                                support += 1
                                break
                if support < K_OVH_SUP_MIN:
                    isolated = True
                    break
            if isolated:
                obst.add((gx, gy))
    return obst


# ---------------- 新逻辑 ----------------
def decide_new(patches, ground):
    surface = [[NAN] * NY for _ in range(NX)]
    has_ovh = [[0] * NY for _ in range(NX)]
    clearance = [[NAN] * NY for _ in range(NX)]
    lowest_ovh = [[-1] * NY for _ in range(NX)]

    for gx in range(NX):
        for gy in range(NY):
            lst = patches[gx][gy]
            if not lst:
                continue
            g = ground[gx][gy]
            if isnan(g):
                continue
            lo = lst[0]
            if (lo[1] - lo[0]) <= K_GROUND_THICK and lo[0] <= g + K_GROUNDED_TOL:
                surface[gx][gy] = lo[1]
            for i, (b, t) in enumerate(lst):
                if b <= g + K_GROUNDED_TOL:
                    continue
                has_ovh[gx][gy] = 1
                lowest_ovh[gx][gy] = i
                clearance[gx][gy] = b - g
                break

    clr_agg = [[clearance[gx][gy] for gy in range(NY)] for gx in range(NX)]
    for gx in range(NX):
        for gy in range(NY):
            if not has_ovh[gx][gy]:
                continue
            vals = []
            for dy in range(-K_CLR_R, K_CLR_R + 1):
                for dx in range(-K_CLR_R, K_CLR_R + 1):
                    nx, ny = gx + dx, gy + dy
                    if (0 <= nx < NX and 0 <= ny < NY and has_ovh[nx][ny]
                            and abs(clearance[nx][ny] - clearance[gx][gy]) <= K_CLR_BAND):
                        vals.append(clearance[nx][ny])
            if not vals:
                continue
            vals.sort()
            clr_agg[gx][gy] = vals[int(K_CLR_Q * (len(vals) - 1))]

    obst = set()
    for gx in range(NX):
        for gy in range(NY):
            lst = patches[gx][gy]
            if not lst:
                continue
            g = ground[gx][gy]
            if isnan(g):
                continue
            lo = lst[0]
            grounded = lo[0] <= g + K_GROUNDED_TOL
            obstacle = False
            if grounded and (lo[1] - g) >= K_VERT_H:
                obstacle = True
            elif has_ovh[gx][gy]:
                if clr_agg[gx][gy] < ROBOT_H:
                    obstacle = True
                else:
                    p = lst[lowest_ovh[gx][gy]]
                    support, enough = 0, False
                    for dy in range(-K_OVH_SUP_R, K_OVH_SUP_R + 1):
                        if enough:
                            break
                        for dx in range(-K_OVH_SUP_R, K_OVH_SUP_R + 1):
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = gx + dx, gy + dy
                            if not (0 <= nx < NX and 0 <= ny < NY):
                                continue
                            for (qb, qt) in patches[nx][ny]:
                                if qb <= p[1] + K_OVH_SUP_TOL and qt >= p[0] - K_OVH_SUP_TOL:
                                    support += 1
                                    break
                            if support >= K_OVH_SUP_MIN:
                                enough = True
                                break
                    obstacle = support < K_OVH_SUP_MIN
            if not obstacle and not isnan(surface[gx][gy]):
                s = surface[gx][gy]
                lower = 0
                for dy in range(-K_STEP_R, K_STEP_R + 1):
                    if lower >= K_STEP_WIT:
                        break
                    for dx in range(-K_STEP_R, K_STEP_R + 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = gx + dx, gy + dy
                        if not (0 <= nx < NX and 0 <= ny < NY):
                            continue
                        sn = surface[nx][ny]
                        if isnan(sn) or sn >= s - K_MAX_CLIMB:
                            continue
                        if has_ovh[nx][ny] and clr_agg[nx][ny] < ROBOT_H:
                            continue
                        lower += 1
                        if lower >= K_STEP_WIT:
                            break
                obstacle = lower >= K_STEP_WIT
            if obstacle:
                obst.add((gx, gy))
    return obst


def region(cx, cy):
    if BAR_X0 <= cx <= BAR_X1 and BAR_Y0 <= cy <= BAR_Y1:
        return "限高杆横梁下"
    if TUN_X0 <= cx <= TUN_X1 and TUN_Y0 <= cy <= TUN_Y1:
        return "隧道顶下"
    if (abs(cx - BAR_X0) < 0.06 or abs(cx - BAR_X1) < 0.06) and BAR_Y0 - 0.06 <= cy <= BAR_Y1 + 0.06:
        return "限高杆立柱"
    if abs(cx - POLE_XY[0]) <= 0.05 and abs(cy - POLE_XY[1]) <= 0.05:
        return "细杆"
    if abs(cx - PLAT_X) <= 0.10 and cy < RAMP_Y0:
        return "20cm台阶边沿"
    if 1.72 <= cx <= 1.84 and cy < RAMP_Y0:
        return "台阶内侧环"
    if RAMP_X0 + 0.15 <= cx <= 2.85 and RAMP_Y0 + 0.13 <= cy <= RAMP_Y1 - 0.13:
        return "坡道面(远离侧边)"
    if RAMP_X0 + 0.15 <= cx <= 2.85 and RAMP_Y0 <= cy <= RAMP_Y1:
        return "坡道侧边条"
    if 1.85 <= cx <= 2.85 and 0.55 <= cy <= 1.90:
        return "高台内(距边>25cm)"
    if cx < PLAT_X - 0.10 and cy < RAMP_Y0:
        return "平地"
    return "其他"


FRAMES = 14
old_sets, new_sets = [], []
for f in range(FRAMES):
    cells = build_cells(gen_frame(1000 + f))
    patches = build_patches(cells)
    ground = build_ground(patches)
    old_sets.append(decide_old(patches, ground))
    new_sets.append(decide_new(patches, ground))

stat = defaultdict(lambda: {"n": 0, "old_occ": 0, "new_occ": 0, "old_flip": 0, "new_flip": 0})
for gx in range(NX):
    for gy in range(NY):
        cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
        s = stat[region(cx, cy)]
        s["n"] += 1
        old_seq = [(gx, gy) in st for st in old_sets]
        new_seq = [(gx, gy) in st for st in new_sets]
        s["old_occ"] += sum(old_seq)
        s["new_occ"] += sum(new_seq)
        s["old_flip"] += 1 if (any(old_seq) and not all(old_seq)) else 0
        s["new_flip"] += 1 if (any(new_seq) and not all(new_seq)) else 0

print(f"帧数={FRAMES}  分辨率={RES}  格数={NX}x{NY}  (障碍帧次 = 该区域所有格 %d 帧里判障碍的次数)" % FRAMES)
print(f"{'区域':<20}{'格数':>5}{'旧:障碍帧次':>12}{'旧:翻转格':>10}{'新:障碍帧次':>12}{'新:翻转格':>10}")
for r in ["平地", "限高杆横梁下", "限高杆立柱", "隧道顶下", "细杆", "20cm台阶边沿",
          "台阶内侧环", "高台内(距边>25cm)", "坡道面(远离侧边)", "坡道侧边条", "其他"]:
    if r not in stat:
        continue
    s = stat[r]
    print(f"{r:<20}{s['n']:>5}{s['old_occ']:>12}{s['old_flip']:>10}{s['new_occ']:>12}{s['new_flip']:>10}")

print("\n每帧障碍格数  旧:", [len(s) for s in old_sets])
print("              新:", [len(s) for s in new_sets])

# ---- 定位：新逻辑里"始终障碍"或"始终空闲"的反常格 ----
print("\n---- 新逻辑逐帧恒定障碍、但期望空闲的格 ----")
for r_name in ["隧道顶下", "坡道面(远离侧边)", "高台内(距边>25cm)", "平地", "限高杆横梁下"]:
    bad = []
    for gx in range(NX):
        for gy in range(NY):
            cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
            if region(cx, cy) != r_name:
                continue
            seq = [(gx, gy) in st for st in new_sets]
            if all(seq):
                bad.append((gx, gy, cx, cy))
    if bad:
        print(f"[{r_name}] 始终障碍 {len(bad)} 格，示例:", [(round(b[2], 2), round(b[3], 2)) for b in bad[:6]])

print("\n---- 新逻辑逐帧翻转格 ----")
flips = []
for gx in range(NX):
    for gy in range(NY):
        seq = [(gx, gy) in st for st in new_sets]
        if any(seq) and not all(seq):
            cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
            flips.append((region(cx, cy), round(cx, 2), round(cy, 2), sum(seq)))
print(flips[:20])

# ---- 帧 0 的细节：为什么这些格被判障碍 ----
cells = build_cells(gen_frame(1000))
patches = build_patches(cells)
ground = build_ground(patches)


# ---- 深挖：隧道顶 / 立柱 的异常格 ----
print("\n---- 隧道顶异常帧 ----")
for f in range(FRAMES):
    cells_f = build_cells(gen_frame(1000 + f))
    pf = build_patches(cells_f)
    gf = build_ground(pf)
    st = new_sets[f]
    tun = [(gx, gy) for (gx, gy) in st
           if region((gx + 0.5) * RES, (gy + 0.5) * RES) == "隧道顶下"]
    if not tun:
        continue
    print(f"帧{f}: 隧道障碍格 {len(tun)}")
    for (gx, gy) in tun[:4]:
        g = gf[gx][gy]
        print(f"  格({(gx+0.5)*RES:.2f},{(gy+0.5)*RES:.2f}) G={g:.3f} patches={[(round(b,3), round(t,3)) for b,t in pf[gx][gy]]}")
    break

print("\n---- 立柱格细节（帧2） ----")
cells_f = build_cells(gen_frame(1002))
pf = build_patches(cells_f)
gf = build_ground(pf)
for gx in range(NX):
    for gy in range(NY):
        cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
        if abs(cx - BAR_X0) < 0.06 and abs(cy - BAR_Y0) < 0.06:
            g = gf[gx][gy]
            print(f"  格({cx:.2f},{cy:.2f}) G={g:.3f} patches={[(round(b,3), round(t,3)) for b,t in pf[gx][gy]]}")

# ---- 单格解释器 ----
def explain(patches, ground, gx, gy):
    lst = patches[gx][gy]
    g = ground[gx][gy]
    lo = lst[0]
    print(f"\n格({(gx+0.5)*RES:.2f},{(gy+0.5)*RES:.2f}) G={g:.4f} patches={[(round(b,4),round(t,4)) for b,t in lst]}")
    grounded = lo[0] <= g + K_GROUNDED_TOL
    print(f"  grounded={grounded} low.top-G={lo[1]-g:.4f} vert={grounded and (lo[1]-g)>=K_VERT_H}")
    ho = [i for i,(b,t) in enumerate(lst) if b > g + K_GROUNDED_TOL]
    print(f"  overhead idx={ho} clearance={[round(lst[i][0]-g,4) for i in ho]}")
    if ho:
        vals = []
        for dy in range(-K_CLR_R, K_CLR_R+1):
            for dx in range(-K_CLR_R, K_CLR_R+1):
                nx, ny = gx+dx, gy+dy
                if not (0 <= nx < NX and 0 <= ny < NY):
                    continue
                ln = patches[nx][ny]
                gn = ground[nx][ny]
                if not ln or isnan(gn):
                    continue
                for (b,t) in ln:
                    if b > gn + K_GROUNDED_TOL:
                        vals.append(round(b-gn, 4)); break
        vals.sort()
        print(f"  c_agg窗口={len(vals)}个值 分位={vals[int(K_CLR_Q*(len(vals)-1))] if vals else None}")
        p = lst[ho[0]]
        sup = 0
        for dy in range(-K_OVH_SUP_R, K_OVH_SUP_R+1):
            for dx in range(-K_OVH_SUP_R, K_OVH_SUP_R+1):
                if dx == 0 and dy == 0: continue
                nx, ny = gx+dx, gy+dy
                if not (0 <= nx < NX and 0 <= ny < NY): continue
                for (qb,qt) in patches[nx][ny]:
                    if abs(qb-p[0]) <= K_OVH_SUP_TOL:
                        sup += 1; break
        print(f"  支撑柱数={sup}")
    s = None
    if (lo[1]-lo[0]) <= K_GROUND_THICK and lo[0] <= g + K_GROUNDED_TOL:
        s = lo[1]
    print(f"  surface={s}")
    if s is not None:
        lows = []
        for dy in range(-K_STEP_R, K_STEP_R+1):
            for dx in range(-K_STEP_R, K_STEP_R+1):
                if dx == 0 and dy == 0: continue
                nx, ny = gx+dx, gy+dy
                if not (0 <= nx < NX and 0 <= ny < NY): continue
                ln = patches[nx][ny]
                gn = ground[nx][ny]
                if not ln or isnan(gn): continue
                l2 = ln[0]
                if (l2[1]-l2[0]) <= K_GROUND_THICK and l2[0] <= gn + K_GROUNDED_TOL:
                    if l2[1] < s - K_MAX_CLIMB:
                        lows.append((round((nx+0.5)*RES,2), round((ny+0.5)*RES,2), round(l2[1],3)))
        print(f"  低于本面>10cm的邻格={len(lows)} {lows[:5]}")

cells0 = build_cells(gen_frame(1000))
p0 = build_patches(cells0)
g0 = build_ground(p0)
explain(p0, g0, int(0.98/RES), int(2.58/RES))

# ================= 闭环/连通性专项检查 =================
def step_edge_gap_rows(obst):
    """台阶边沿：逐 y 看 x∈[1.50,1.70] 里有没有障碍，返回断裂行数。"""
    gaps = 0
    for gy in range(NY):
        hit = any((gx, gy) in obst for gx in range(NX)
                  if 1.50 <= (gx + 0.5) * RES <= 1.70)
        if not hit:
            gaps += 1
    return gaps


def ramp_junction_obst(obst):
    """坡道↔台面接缝（x 1.85~2.15, y 2.10~2.50）里的障碍格数，应为 0。"""
    return sum(1 for (gx, gy) in obst
               if 1.85 <= (gx + 0.5) * RES <= 2.15 and 2.10 <= (gy + 0.5) * RES <= 2.50)


def beam_corridor_obst(obst):
    """限高杆横梁正下方（排除四角立柱 2 格内）的障碍格数，应为 0。"""
    n = 0
    for (gx, gy) in obst:
        cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
        if not (BAR_X0 <= cx <= BAR_X1 and BAR_Y0 <= cy <= BAR_Y1):
            continue
        if min(abs(cx - BAR_X0), abs(cx - BAR_X1)) <= 0.10:
            continue
        n += 1
    return n


print("\n================ 闭环 / 连通性（%d 帧） ================" % FRAMES)
print(f"{'指标':<34}{'旧':>8}{'新':>8}")
print(f"{'台阶边沿断裂行数(逐帧最大值)':<30}{max(step_edge_gap_rows(s) for s in old_sets):>8}{max(step_edge_gap_rows(s) for s in new_sets):>8}")
print(f"{'坡道↔台面接缝障碍格(逐帧最大值)':<28}{max(ramp_junction_obst(s) for s in old_sets):>8}{max(ramp_junction_obst(s) for s in new_sets):>8}")
print(f"{'横梁正下方障碍格(逐帧最大值)':<30}{max(beam_corridor_obst(s) for s in old_sets):>8}{max(beam_corridor_obst(s) for s in new_sets):>8}")

# ---- 台阶内侧环 & 边沿断裂行细节 ----
print("\n---- 台阶内侧环: 有障碍的格及其帧数 ----")
for gx in range(NX):
    for gy in range(NY):
        cx, cy = (gx + 0.5) * RES, (gy + 0.5) * RES
        if region(cx, cy) != "台阶内侧环":
            continue
        cnt = sum((gx, gy) in s for s in new_sets)
        if cnt:
            print(f"  ({cx:.2f},{cy:.2f}) 帧数={cnt}")

print("\n---- 台阶边沿断裂行（新逻辑最后一帧） ----")
cells_f = build_cells(gen_frame(1000 + FRAMES - 1))
pf = build_patches(cells_f)
gf = build_ground(pf)
obst = new_sets[-1]
for gy in range(NY):
    hit = any((gx, gy) in obst for gx in range(NX) if 1.50 <= (gx + 0.5) * RES <= 1.70)
    if hit:
        continue
    cy = (gy + 0.5) * RES
    print(f"  行 y={cy:.2f}:")
    for gx in range(NX):
        cx = (gx + 0.5) * RES
        if 1.50 <= cx <= 1.78:
            g = gf[gx][gy]
            print(f"     x={cx:.2f} G={g if isnan(g) else round(g,3)} patches={[(round(b,3),round(t,3)) for b,t in pf[gx][gy]]}")

# ---- 解释 (1.73,0.38) 为什么有时障碍 ----
gx_t, gy_t = int(1.725 / RES), int(0.375 / RES)
print(f"\n---- 解释格 ({(gx_t+0.5)*RES:.2f},{(gy_t+0.5)*RES:.2f}) ----")
for f in range(FRAMES):
    cells_f = build_cells(gen_frame(1000 + f))
    pf = build_patches(cells_f)
    gf = build_ground(pf)
    flag = (gx_t, gy_t) in new_sets[f]
    print(f"帧{f} 障碍={flag} G={gf[gx_t][gy_t]:.3f} patches={[(round(b,3),round(t,3)) for b,t in pf[gx_t][gy_t]]}")
