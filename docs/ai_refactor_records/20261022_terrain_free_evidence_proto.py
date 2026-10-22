#!/usr/bin/env python3
"""terrain 悬空判据：引入 ROG-map free/unknown 证据的离线原型（只读 bag，不改 C++）。

OLD = 当前 src/map/src/terrain_analysis.cpp 的逐行移植（robot_height=0.24）。
NEW = OLD + 以下改动：
  1) 浮空薄层不当地面来源：本列最低占据体素正下方 kFloatBelow 个体素已知空闲 → 该列 ground_raw 置 NaN，
     参考地面改由邻域（非浮空列）给出；邻域也没有 → 本格未知，不写。
  2) 间隙判据带证据：z_low - z_ref 落在 [H_LO, H_HI) 时，只有间隙段已知空闲比例 >= kGapFreeRatio
     且 z_low 正下方体素已知空闲才放行；间隙 >= H_HI 维持旧行为（直接放行）。
  3) 间隙段未知占多数且本列是浮空候选（最低体素以下没有占据）→ 未知不写，保留旧值。

用法：
  python3 20261022_terrain_free_evidence_proto.py diag  <frame,...>   # 打印杆/台阶区逐列诊断
  python3 20261022_terrain_free_evidence_proto.py replay [stride] [out.png]  # 全 bag 回放 + 持久层对比
"""
import sqlite3, sys
import numpy as np
from scipy.ndimage import generic_filter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid

BAG = '/home/xyt/map/src/MA_Nav/rosbag/rosbag2_2026_10_20-20_15_24/rosbag2_2026_10_20-20_15_24_0.db3'
RES = 0.05
N = 201                      # local_update_box 10 m → 201 格（cfg 规整为奇数）
WIN = N * RES
Z0, NZ = -1.00, 50           # 体素层 [-1.0, 1.5)，体素中心 = Z0 + (k+0.5)*RES
KERNEL = 3
BAND = 0.08
JUMP = 0.10
GRAD = 0.08                  # steep_threshold
H_OLD = 0.24
# ---- NEW 参数 ----
H_HI = 0.24                  # 间隙 >= H_HI：不需证据直接放行（与旧行为一致）
H_LO = float(sys.argv[4]) if len(sys.argv) > 4 else 0.18
kFloatBelow = 2              # 最低占据体素下方连续已知空闲体素数，判浮空
kGapFreeRatio = 0.5
kFreeRun = int(sys.argv[5]) if len(sys.argv) > 5 else 3   # 下沿正下方连续已知空闲体素数（3 → 0.15 m）
kUnkSkip = int(sys.argv[6]) if len(sys.argv) > 6 else 1   # 下沿正下方允许跳过的未知体素数（混合像素拖低）
kEdgeGuard = int(sys.argv[7]) if len(sys.argv) > 7 else 0 # 台沿保护邻域半径（格），0 关闭
kFloatRefRadius = int(sys.argv[8]) if len(sys.argv) > 8 else 6  # 悬空列找邻域地面的半径（格）
FREE_FRAMES = 2              # ma_map kFreeFramesToClear

REGIONS = {
    'bar_x6': (5.90, 6.60, -7.00, -6.65),
    'bar_B16': (10.60, 11.15, -8.75, -8.20),
    'bar_B18': (11.55, 11.80, -7.90, -7.10),
}
STEP_BAND = (4.90, 5.85, -7.10, -6.60)

con = sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True)
cur = con.cursor()
tid = {n: i for i, n in cur.execute('select id, name from topics')}


class LazyTopic:
    """只缓存 (timestamp, rowid)，按需单帧读取 data。
    注意：/ma_nav/map/unk 每帧约 240 万点，整话题 fetchall 近 20 GB，曾导致整机卡死。"""

    def __init__(self, name):
        idx = cur.execute('select timestamp, id from messages where topic_id=? order by timestamp',
                          (tid[name],)).fetchall()
        self.ts = np.array([r[0] for r in idx], dtype=np.int64)
        self.ids = [r[1] for r in idx]

    def __len__(self):
        return len(self.ids)

    def data(self, k):
        return cur.execute('select data from messages where id=?', (self.ids[k],)).fetchone()[0]


def rows(name):
    # 仅用于小话题（odom）
    return cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[name],)).fetchall()


def xyz(b):
    m = deserialize_message(b, PointCloud2)
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


base = np.array([(ts, *(lambda p: (p.x, p.y, p.z))(deserialize_message(b, Odometry).pose.pose.position))
                 for ts, b in rows('/lio/base_odom')])
occ_rows, unk_rows = LazyTopic('/ma_nav/map/occ'), LazyTopic('/ma_nav/map/unk')
unk_ts = unk_rows.ts


def nanmedian_filter(a, k):
    return generic_filter(a, lambda v: np.nan if np.all(np.isnan(v)) else np.nanmedian(v),
                          size=k, mode='constant', cval=np.nan)


def neigh_min(a, r):
    """5x5 邻域（含自身）nan-min。"""
    p = np.pad(a, r, constant_values=np.nan)
    stack = [p[r + dx:r + dx + N, r + dy:r + dy + N] for dx in range(-r, r + 1) for dy in range(-r, r + 1)]
    with np.errstate(all='ignore'):
        return np.nanmin(np.stack(stack), 0)


def gradient8(z):
    p = np.pad(z, 1, constant_values=np.nan)
    g = np.zeros_like(z)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            d = np.abs(z - p[1 + dx:1 + dx + N, 1 + dy:1 + dy + N])
            g = np.fmax(g, np.nan_to_num(d, nan=0.0))
    return g


def load_frame(k):
    ts = int(occ_rows.ts[k])
    occ = xyz(occ_rows.data(k))
    unk = xyz(unk_rows.data(int(np.argmin(abs(unk_ts - ts)))))
    rp = base[max(0, int(np.searchsorted(base[:, 0], ts)) - 1), 1:4]
    ox = round((rp[0] - WIN / 2) / RES) * RES   # 与 setupGrid 一致（窗口原点对齐分辨率）
    oy = round((rp[1] - WIN / 2) / RES) * RES
    state = np.zeros((N, N, NZ), np.int8)       # 0 已知空闲 1 占据 2 未知
    for arr, v in ((unk, 2), (occ, 1)):
        i = np.floor((arr[:, 0] - ox) / RES + 1e-6).astype(int)
        j = np.floor((arr[:, 1] - oy) / RES + 1e-6).astype(int)
        z = np.floor((arr[:, 2] - Z0) / RES + 1e-6).astype(int)
        m = (i >= 0) & (i < N) & (j >= 0) & (j < N) & (z >= 0) & (z < NZ)
        state[i[m], j[m], z[m]] = v
    return dict(ts=ts, ox=ox, oy=oy, rp=rp, state=state)


ZC = Z0 + (np.arange(NZ) + 0.5) * RES


def analyze(fr, new):
    st = fr['state']
    occm = st == 1
    has = occm.any(2)
    lo = np.argmax(occm, 2)
    hi = NZ - 1 - np.argmax(occm[:, :, ::-1], 2)
    ground_raw = np.where(has, ZC[lo], np.nan)
    max_h = np.where(has, ZC[hi], np.nan)

    floating = np.zeros((N, N), bool)
    fr_stat = fr.setdefault('stat', {})
    if new:
        # 悬空列：最低占据体素正下方连续 kFreeRun 个体素已知空闲。
        # 射线穿过了它的下方 → 它不是地面，而是悬空结构（杆）；下方净空 >= kFreeRun*RES 已被观测证实。
        # 台面/台阶顶面下方是实心，射线进不去，只会是未知 → 不会被判成悬空。
        run = free_run_below_tol(st, lo)
        cand = has & (run >= kFreeRun)
        # 邻域地面：非候选列的最低占据，半径 kFloatRefRadius（杆宽 0.25–0.6 m，杆中间列近邻也全是候选）
        g_nb = neigh_min(np.where(cand, np.nan, ground_raw), kFloatRefRadius)
        # 已知空闲段的底（体素下边界）。必须向下一直通到邻域地面附近，净空才算被射线证实；
        # 台沿外侧被掠射射线刻出的一小段空气、底下没有地面的，过不了这一条。
        # 偏保守：按跳过了 kUnkSkip 个未知体素估计空闲段底；地面是下一格占据体素中心，容差半格。
        free_bottom = ZC[lo] - RES / 2 - (run + kUnkSkip) * RES
        with np.errstate(invalid='ignore'):
            floating = cand & (ZC[lo] - g_nb >= H_LO) & (free_bottom <= g_nb + RES / 2 + 1e-6)
        fr_stat['cand_rejected'] = cand & ~floating
        if kEdgeGuard:
            # 台沿保护：台面边沿格跨在立面外的空气上，立面没扫到时它也"下方已知空闲"。
            # 区别在于台面会向一侧延伸成实心面：邻格同一高度有占据、且其正下方是未知（射线进不去）、
            # 且那就是邻格的最低占据（邻格是"顶面"而不是杆下方还有地面）。
            solid_top = has & ~floating & (lo >= 1)
            solid_top &= st[np.arange(N)[:, None], np.arange(N)[None, :], np.clip(lo - 1, 0, NZ - 1)] == 2
            R = kEdgeGuard
            guard = np.zeros((N, N), bool)
            lo_s = np.where(solid_top, lo, -100)
            p = np.pad(lo_s, R, constant_values=-100)
            for dx in range(-R, R + 1):
                for dy in range(-R, R + 1):
                    if dx == 0 and dy == 0: continue
                    nb = p[R + dx:R + dx + N, R + dy:R + dy + N]
                    guard |= np.abs(nb - lo) <= 1
            fr_guard = floating & guard
            fr_stat['guarded'] = fr_guard
            floating = floating & ~guard
        fr_stat['floating'] = floating
        src = np.where(floating, np.nan, ground_raw)
    else:
        src = ground_raw

    ground = nanmedian_filter(src, KERNEL)
    gmin = neigh_min(src, 2)
    with np.errstate(invalid='ignore'):
        z_ref = np.where(ground - gmin > JUMP, gmin, ground)
    if new:
        # 浮空列：自身不提供地面，参考地面只取邻域非浮空列的最低地面；邻域也没有 → NaN（未知）
        z_ref = np.where(floating, np.nan, z_ref)
    grad = gradient8(z_ref)

    obst = np.zeros((N, N), bool); free = np.zeros((N, N), bool)
    info = {}
    for i, j in zip(*np.nonzero(has)):
        if floating[i, j]:
            # 悬空列：下方净空已被射线证实，可通行（场地内下沿 15–25 cm 的悬空结构都可钻）
            free[i, j] = True
            info[(i, j)] = (np.nan, ZC[lo[i, j]], np.nan, 1.0, 0.0, True, True, ZC[hi[i, j]] - ZC[lo[i, j]])
            continue
        zr = z_ref[i, j]
        if np.isnan(zr):
            continue
        col = occm[i, j]
        above = col & (ZC > zr + BAND)
        if not above.any():
            if grad[i, j] > GRAD: obst[i, j] = True
            else: free[i, j] = True
            continue
        kl = int(np.argmax(above)); zl = ZC[kl]
        gap = zl - zr
        if not new:
            (free if gap >= H_OLD else obst)[i, j] = True
            continue
        # 间隙段体素：(z_ref, z_low) 开区间
        g0 = int(np.floor((zr - Z0) / RES)) + 1
        seg = st[i, j, max(g0, 0):kl]
        fr_free = float((seg == 0).mean()) if seg.size else 0.0
        fr_unk = float((seg == 2).mean()) if seg.size else 0.0
        below_free = kl >= 1 and st[i, j, kl - 1] == 0
        info[(i, j)] = (zr, zl, gap, fr_free, fr_unk, below_free, bool(floating[i, j]), ZC[hi[i, j]] - zl)
        run_kl = int(free_run_below_tol(st[i:i + 1, j:j + 1], np.array([[kl]]))[0, 0])
        if gap >= H_HI:
            free[i, j] = True
        elif gap >= H_LO and run_kl >= kFreeRun and \
                ZC[kl] - RES / 2 - (run_kl + kUnkSkip) * RES <= zr + RES / 2 + 1e-6:
            # 本列有地面，下沿到地面之间已被射线扫空（量化/拖点使 gap 偏小）
            free[i, j] = True
        else:
            obst[i, j] = True
    free &= ~obst
    return obst, free, info


def free_run_below_tol(st, k):
    """同 free_run_below，但允许下沿正下方紧邻的 1 个体素是未知：
    杆底边混合像素把下沿拖低一格，这一格射线永远清不掉。"""
    run = free_run_below(st, k)
    n0, n1 = k.shape
    I, J = np.arange(n0)[:, None], np.arange(n1)[None, :]
    best = run
    kk = k.copy()
    ok = np.ones_like(k, bool)
    for _ in range(kUnkSkip):
        ok &= (kk >= 1) & (st[I, J, np.clip(kk - 1, 0, NZ - 1)] == 2)
        kk = np.maximum(kk - 1, 0)
        best = np.where(ok, np.maximum(best, free_run_below(st, kk)), best)
    return best


def free_run_below(st, k):
    """每列：从体素 k-1 向下连续已知空闲的体素数（遇到占据/未知停止）。"""
    n0, n1 = k.shape
    run = np.zeros((n0, n1), np.int32)
    alive = np.ones((n0, n1), bool)
    I, J = np.arange(n0)[:, None], np.arange(n1)[None, :]
    for d in range(1, NZ):
        kk = k - d
        valid = alive & (kk >= 0)
        s = st[I, J, np.clip(kk, 0, NZ - 1)]
        alive = valid & (s == 0)
        run += alive
        if not alive.any():
            break
    return run


def world_idx(fr):
    return int(round(fr['ox'] / RES)), int(round(fr['oy'] / RES))


def region_cells(box):
    x0, x1, y0, y1 = box
    return [(a, c) for a in range(int(np.floor(x0 / RES)), int(np.ceil(x1 / RES)))
            for c in range(int(np.floor(y0 / RES)), int(np.ceil(y1 / RES)))
            if x0 <= (a + .5) * RES < x1 and y0 <= (c + .5) * RES < y1]


def diag(frames):
    for k in frames:
        fr = load_frame(k)
        oi, oj = world_idx(fr)
        t = (fr['ts'] - base[0, 0]) / 1e9
        print(f'==== frame {k} t={t:.1f}s robot=({fr["rp"][0]:.2f},{fr["rp"][1]:.2f},{fr["rp"][2]:.3f})')
        oo, of, _ = analyze(fr, False)
        no, nf, info = analyze(fr, True)
        for name, box in list(REGIONS.items()) + [('step', STEP_BAND)]:
            cnt = {'old_o': 0, 'old_f': 0, 'new_o': 0, 'new_f': 0, 'new_unk': 0}
            lines = []
            for a, c in region_cells(box):
                i, j = a - oi, c - oj
                if not (0 <= i < N and 0 <= j < N): continue
                cnt['old_o'] += oo[i, j]; cnt['old_f'] += of[i, j]
                cnt['new_o'] += no[i, j]; cnt['new_f'] += nf[i, j]
                if (i, j) in info and not no[i, j] and not nf[i, j]: cnt['new_unk'] += 1
                if (i, j) in info and name != 'step':
                    zr, zl, gap, ff, fu, bf, fl, th = info[(i, j)]
                    lines.append(f'   ({(a+.5)*RES:.3f},{(c+.5)*RES:.3f}) zref={zr:+.3f} zlow={zl:+.3f} gap={gap:.3f} '
                                 f'free={ff:.2f} unk={fu:.2f} below_free={int(bf)} float={int(fl)} thick={th:.2f} '
                                 f'-> {"O" if no[i,j] else "F" if nf[i,j] else "?"}')
            d = np.hypot((box[0] + box[1]) / 2 - fr['rp'][0], (box[2] + box[3]) / 2 - fr['rp'][1])
            print(f' {name:8s} dist={d:.1f}m {cnt}')
            if '-v' in sys.argv:
                print('\n'.join(lines[:40]))


def replay(stride, out):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    persist = {'old': {}, 'new': {}}      # (a,c) -> [occ, streak, last_frame]
    snaps = []
    frames = list(range(0, len(occ_rows), stride))
    snap_at = set(frames[int(len(frames) * q)] for q in (0.25, 0.5, 0.75)) | {frames[-1]}
    for fid, k in enumerate(frames):
        fr = load_frame(k)
        oi, oj = world_idx(fr)
        for tag, new in (('old', False), ('new', True)):
            o, f, _ = analyze(fr, new)
            P = persist[tag]
            for i, j in zip(*np.nonzero(o)):
                P[(i + oi, j + oj)] = [1, 0, P.get((i + oi, j + oj), [0, 0, -10])[2]]
            for i, j in zip(*np.nonzero(f)):
                key = (i + oi, j + oj)
                e = P.get(key, [0, 0, -10])
                if e[0] == 0:
                    P[key] = e; continue
                e[1] = e[1] + 1 if e[2] == fid - 1 else 1
                e[2] = fid
                if e[1] >= FREE_FRAMES: e[0] = 0; e[1] = 0
                P[key] = e
        t = (fr['ts'] - base[0, 0]) / 1e9
        msg = f'frame {k:3d} t={t:5.1f}s robot=({fr["rp"][0]:5.2f},{fr["rp"][1]:5.2f})'
        for tag in ('old', 'new'):
            P = persist[tag]
            s = ' '.join(f'{n}={sum(P.get(c, [0])[0] for c in region_cells(b))}' for n, b in REGIONS.items())
            x0, x1, y0, y1 = STEP_BAND
            cols = np.arange(int(np.floor(x0 / RES)), int(np.ceil(x1 / RES)))
            closed = sum(any(P.get((a, c), [0])[0] for c in range(int(np.floor(y0 / RES)), int(np.ceil(y1 / RES)))) for a in cols)
            msg += f' | {tag}: {s} step={closed}/{len(cols)}'
        print(msg, flush=True)
        if k in snap_at:
            snaps.append((k, t, {tg: {kk: v[0] for kk, v in persist[tg].items()} for tg in persist}))
    fig, axs = plt.subplots(2, len(snaps), figsize=(6 * len(snaps), 10))
    for col, (k, t, S) in enumerate(snaps):
        for row, tg in enumerate(('old', 'new')):
            ax = axs[row, col]
            img = np.full((int(12 / RES), int(16 / RES)), np.nan)
            for (a, c), v in S[tg].items():
                ii, jj = c + int(10 / RES), a + int(2 / RES)
                if 0 <= ii < img.shape[0] and 0 <= jj < img.shape[1]: img[ii, jj] = v
            ax.imshow(img, origin='lower', extent=[-2, 14, -10, 2], cmap='gray_r', vmin=0, vmax=1)
            for n, (x0, x1, y0, y1) in REGIONS.items():
                ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='c', lw=1.2))
            x0, x1, y0, y1 = STEP_BAND
            ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='orange', lw=1.2))
            ax.set_xlim(3, 13); ax.set_ylim(-10, -4); ax.set_aspect('equal')
            ax.set_title(f'{tg.upper()} frame {k} t={t:.0f}s')
    plt.tight_layout(); plt.savefig(out, dpi=60)
    print('saved', out)


if __name__ == '__main__':
    if sys.argv[1] == 'diag':
        diag([int(s) for s in sys.argv[2].split(',')])
    else:
        replay(int(sys.argv[2]) if len(sys.argv) > 2 else 1,
               sys.argv[3] if len(sys.argv) > 3 else '/tmp/free_evidence_cmp.png')
