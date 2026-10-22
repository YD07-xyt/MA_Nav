#!/usr/bin/env python3
"""地面生长方案离线原型（只读 bag，不改 C++）。

输入：/ma_nav/map/occ（占据体素中心）、/ma_nav/map/unk（未知体素中心）、/lio/base_odom（base_link 与地同高）。
已知空闲 = 窗口内既不占据也不未知的体素（与 rog_map isKnownFree 一致）。

逐柱（0.05 m）分类：
  surface ：最低占据体素正下方不是已知空闲（实心物体/地面，下方射线打不进去）
  overhead：最低占据体素正下方 >= kFreeBelow 个体素已知空闲（悬空结构：限高杆、桌面）
  empty   ：柱内无占据（地面被掠射射线清掉也属于这一类）
  vertical：surface 且占据从最低点向上连续超过 kVertical → 墙/立柱/台阶立面，无条件障碍

地面生长（Dijkstra，按水平距离）：
  种子 = 轨迹足迹格，G = base_odom.z
  c → n：
    surface ：|z_n - G_c| <= kStepTol + tan15*bridge → 同一连续地面，G_n = z_n；
              否则高差跳变：标高处一格为台阶边沿障碍，不扩展
    overhead：G_n = G_c，净空 = 下沿 - G_n，>= robot_height 可通行并扩展，否则障碍
    empty   ：G_n = G_c，bridge 累加，超过 kBridgeMax 停止
  vertical 柱只标障碍不扩展。
  未被生长到的 surface/overhead 柱：不写（2D 保持原值）。
"""
import sqlite3, sys, heapq, math
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid as OG

BAG = sys.argv[1]
OUT = sys.argv[2]
RES = 0.05
WIN = 10.0              # 与 local_update_box 一致
Z0, NZ = -0.30, 20      # 体素层 [-0.30, 0.70)
ROBOT_H = 0.24
kFreeBelow = 2
kVertical = 0.10
kStepTol = 0.06         # 同一地面相邻格允许高差（体素量化 0.05 + 余量）
kSlope = math.tan(math.radians(15))
kBridgeMax = 0.6
kFootR = 0.15

con = sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True)
cur = con.cursor()
tid = {n: i for i, n in cur.execute('select id, name from topics')}


def rows(name):
    return cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[name],)).fetchall()


def xyz(b):
    m = deserialize_message(b, PointCloud2)
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


base = []
for ts, b in rows('/lio/base_odom'):
    p = deserialize_message(b, Odometry).pose.pose.position
    base.append((ts, p.x, p.y, p.z))
base = np.array(base)
occ_rows, unk_rows, grid_rows = rows('/ma_nav/map/occ'), rows('/ma_nav/map/unk'), rows('/ma_nav/grid_map_occ')
occ_ts = np.array([r[0] for r in occ_rows]); unk_ts = np.array([r[0] for r in unk_rows]); grid_ts = np.array([r[0] for r in grid_rows])

# 用户确认的杆位置 + 台阶带（世界系，米）
REGIONS = {
    'bar_x6': (5.90, 6.60, -7.00, -6.65),       # 用户确认：左缘含 x=6，中心略偏右；杆下是上台面的坡
    'bar_B16':     (10.60, 11.15, -8.75, -8.20),
    'bar_B18':     (11.55, 11.80, -7.90, -7.10),
}
STEP_BAND = (4.90, 5.85, -7.10, -6.60)  # 用户确认：t20 下方台阶，左界 x≈5 左一点，右侧紧邻杆


def analyze(k):
    ts = occ_ts[k]
    ku = int(np.argmin(abs(unk_ts - ts)))
    occ = xyz(occ_rows[k][1]); unk = xyz(unk_rows[ku][1])
    ib = int(np.searchsorted(base[:, 0], ts)) - 1
    rp = base[ib, 1:4]
    ox = round((rp[0] - WIN / 2) / RES) * RES; oy = round((rp[1] - WIN / 2) / RES) * RES
    N = int(round(WIN / RES))
    state = np.zeros((N, N, NZ), np.int8)          # 0 free, 1 occ, 2 unknown
    for arr, v in ((unk, 2), (occ, 1)):
        i = np.floor((arr[:, 0] - ox) / RES).astype(int); j = np.floor((arr[:, 1] - oy) / RES).astype(int)
        z = np.floor((arr[:, 2] - Z0) / RES + 1e-6).astype(int)
        m = (i >= 0) & (i < N) & (j >= 0) & (j < N) & (z >= 0) & (z < NZ)
        state[i[m], j[m], z[m]] = v
    zc = Z0 + (np.arange(NZ) + 0.5) * RES

    kind = np.zeros((N, N), np.int8)   # 0 empty 1 surface 2 overhead 3 vertical
    zlow = np.full((N, N), np.nan)
    occm = state == 1
    has = occm.any(2)
    lo = np.argmax(occm, 2)
    for i, j in zip(*np.nonzero(has)):
        l = lo[i, j]; zlow[i, j] = zc[l]
        below = state[i, j, max(0, l - kFreeBelow):l]
        if l >= kFreeBelow and np.all(below == 0):
            kind[i, j] = 2
        else:
            # 从最低点向上连续（允许 1 个空洞）的占据高度
            top = l; gap = 0
            for z in range(l + 1, NZ):
                if occm[i, j, z]: top = z; gap = 0
                else:
                    gap += 1
                    if gap > 1: break
            kind[i, j] = 3 if zc[top] - zc[l] >= kVertical else 1

    obst = np.zeros((N, N), bool); free = np.zeros((N, N), bool)
    obst |= kind == 3
    G = np.full((N, N), np.nan); dist = np.full((N, N), np.inf)
    pq = []
    seg = base[:ib + 1]
    for x, y, z in seg[::5, 1:4]:
        ci, cj = int((x - ox) / RES), int((y - oy) / RES); r = int(math.ceil(kFootR / RES))
        for a in range(ci - r, ci + r + 1):
            for c in range(cj - r, cj + r + 1):
                if 0 <= a < N and 0 <= c < N and math.hypot((a - ci), (c - cj)) * RES <= kFootR and dist[a, c] > 0:
                    dist[a, c] = 0; G[a, c] = z; free[a, c] = kind[a, c] != 3
                    heapq.heappush(pq, (0.0, a, c, 0.0))
    nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while pq:
        d, a, c, br = heapq.heappop(pq)
        if d > dist[a, c]: continue
        g = G[a, c]
        for da, dc, w in nb:
            n1, n2 = a + da, c + dc
            if not (0 <= n1 < N and 0 <= n2 < N): continue
            nd = d + w * RES
            if nd >= dist[n1, n2]: continue
            kd = kind[n1, n2]
            # 最低占据体素已高出上游地面 robot_height：不管下方是已知空闲还是未知、
            # 是薄片还是厚梁，都碰不到机器人。下方已知空闲 → 可通行；下方未知 → 不写，只当桥接。
            if kd != 0 and zlow[n1, n2] - g >= ROBOT_H:
                if kd == 2:
                    G[n1, n2] = g; dist[n1, n2] = nd; free[n1, n2] = True
                    heapq.heappush(pq, (nd, n1, n2, 0.0)); continue
                kd = 0
            if kd == 3:
                obst[n1, n2] = True; continue
            if kd == 1:
                dz = zlow[n1, n2] - g               # 体素中心与上游地面的高差
                if abs(dz) <= kStepTol + kSlope * br:
                    G[n1, n2] = zlow[n1, n2]; dist[n1, n2] = nd; free[n1, n2] = True
                    heapq.heappush(pq, (nd, n1, n2, 0.0))
                else:
                    if dz > 0: obst[n1, n2] = True      # 高处一侧是台沿
                    else: obst[a, c] = True; free[a, c] = False
                continue
            if kd == 2:
                clr = zlow[n1, n2] - g
                G[n1, n2] = g; dist[n1, n2] = nd
                if clr >= ROBOT_H:
                    free[n1, n2] = True; heapq.heappush(pq, (nd, n1, n2, 0.0))
                else:
                    obst[n1, n2] = True
                continue
            nbr = br + w * RES
            if nbr > kBridgeMax: continue
            G[n1, n2] = g; dist[n1, n2] = nd
            heapq.heappush(pq, (nd, n1, n2, nbr))
    free &= ~obst
    return dict(ts=ts, ox=ox, oy=oy, N=N, obst=obst, free=free, kind=kind, G=G, zlow=zlow, rp=rp, traj=seg[:, 1:3])


def old_grid(ts):
    kg = int(np.argmin(abs(grid_ts - ts)))
    m = deserialize_message(grid_rows[kg][1], OG)
    a = np.frombuffer(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    return a, (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)


def region_stats(mask_fn):
    out = {}
    for name, (x0, x1, y0, y1) in REGIONS.items():
        out[name] = mask_fn(x0, x1, y0, y1)
    return out


frames = [int(s) for s in sys.argv[3].split(',')] if len(sys.argv) > 3 else [127, 255, 383, 510]
# 新方案的 2D 回写模拟（建图模式持久层：障碍立即写，空闲立即清——与足迹/生长证据一致）
persist = {}
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, axs = plt.subplots(2, len(frames), figsize=(7 * len(frames), 13))
report = []
for col, k in enumerate(frames):
    r = analyze(k)
    for i, j in zip(*np.nonzero(r['obst'] | r['free'])):
        key = (round(r['ox'] / RES) + i, round(r['oy'] / RES) + j)
        persist[key] = 1 if r['obst'][i, j] else 0
    og, (gr, gx, gy) = old_grid(r['ts'])

    def old_cnt(x0, x1, y0, y1):
        a0, a1 = int((y0 - gy) / gr), int((y1 - gy) / gr); b0, b1 = int((x0 - gx) / gr), int((x1 - gx) / gr)
        return int((og[a0:a1, b0:b1] > 50).sum())

    def new_cnt(x0, x1, y0, y1):
        return sum(v for (a, c), v in persist.items() if x0 <= (a + .5) * RES < x1 and y0 <= (c + .5) * RES < y1)

    def step_closed(cnt_fn):
        x0, x1, y0, y1 = STEP_BAND; ok = 0; tot = 0
        for x in np.arange(x0, x1, RES):
            tot += 1; ok += cnt_fn(x, x + RES, y0, y1) > 0
        return f'{ok}/{tot}'
    t = (r['ts'] - base[0, 0]) / 1e9
    line = f'frame {k} t={t:.1f}s  old: ' + ' '.join(f'{n}={old_cnt(*b)}' for n, b in REGIONS.items()) + f' step_cols_closed={step_closed(old_cnt)}'
    line += '\n' + ' ' * (len(f"frame {k} t={t:.1f}s  ")) + 'new: ' + ' '.join(f'{n}={new_cnt(*b)}' for n, b in REGIONS.items()) + f' step_cols_closed={step_closed(new_cnt)}'
    print(line, flush=True); report.append(line)
    # 画图
    ax = axs[0, col]
    ax.imshow(og, origin='lower', extent=[gx, gx + og.shape[1] * gr, gy, gy + og.shape[0] * gr], cmap='gray_r', vmin=0, vmax=100)
    ax.set_title(f'OLD frame {k} t={t:.0f}s')
    ax = axs[1, col]
    img = np.zeros((int(20 / RES), int(20 / RES)))
    for (a, c), v in persist.items():
        ii, jj = c + int(11 / RES), a + int(4 / RES)
        if 0 <= ii < img.shape[0] and 0 <= jj < img.shape[1]: img[ii, jj] = 100 if v else 0
    ax.imshow(img, origin='lower', extent=[-4, 16, -11, 9], cmap='gray_r', vmin=0, vmax=100)
    ax.set_title(f'NEW (ground grow) frame {k}')
    for ax in axs[:, col]:
        ax.plot(r['traj'][:, 0], r['traj'][:, 1], 'r-', lw=.6)
        for n, (x0, x1, y0, y1) in REGIONS.items():
            ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='c', lw=1.2))
        x0, x1, y0, y1 = STEP_BAND
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='orange', lw=1.2))
        ax.set_xlim(-1, 13); ax.set_ylim(-10, 1); ax.set_aspect('equal')
plt.tight_layout(); plt.savefig(OUT, dpi=55)
print('saved', OUT)
