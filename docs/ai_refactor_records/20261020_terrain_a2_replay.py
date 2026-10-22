#!/usr/bin/env python3
"""离线复算 terrain_analysis 的 A2 方案（波前/地面流形 + 自由空间交叉）。

只读 bag，不写任何数据文件；所有结果打印到 stdout。

用法：
  python3 20261020_terrain_a2_replay.py old          # 复刻 dc1024b 旧逻辑，与 bag 内 2D 地图对比
  python3 20261020_terrain_a2_replay.py old --dump   # 额外打印诊断
  python3 20261020_terrain_a2_replay.py a2 [--frames 0,50,100]  # 复算 A2 方案（需要 free space，读 unk 云）

背景（数据来源，均为 bag 已录 topic，只读）：
  /lio/base_odom        nav_msgs/Odometry    地图节点实际使用的 odom
  /ma_nav/map/occ       sensor_msgs/PointCloud2  占据体素点云 = terrain_analyzer 输入
  /ma_nav/map/unk       sensor_msgs/PointCloud2  局部地图内 UNKNOWN 体素 = 3D 自由空间反推
  /ma_nav/grid_map_occ  nav_msgs/OccupancyGrid   旧方案 2D 输出（对照真值）
  /lio/cloud_world      sensor_msgs/PointCloud2  帧对齐参考
"""
import argparse
import sqlite3
import time
import warnings

import numpy as np
from scipy.ndimage import minimum_filter, binary_dilation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid

warnings.filterwarnings('ignore')

BAG = '/home/xyt/map/src/MA_Nav/rosbag2_2026_10_20-13_46_27/rosbag2_2026_10_20-13_46_27_0.db3'

# ---- terrain 参数（config/map.yaml + ma_map.hpp） ----
RES = 0.05            # 栅格分辨率
ROG_LOCAL_UPDATE_BOX = (10.0, 10.0, 4.0)   # rog_map.raycasting.local_update_box
# 注意 cfg 会把 local_update_box 规整成奇数格：half=100 -> 201 格 -> 10.05 m
SIZE_X = (int(ROG_LOCAL_UPDATE_BOX[0] / 2 / RES) * 2 + 1) * RES   # 10.05
KERNEL = 3
ROBOT_H = 0.24
BAND = 0.08          # kObstacleBand
JUMP = 0.10          # kGroundJumpThresh
KR = 2               # kGroundMinRadius (5x5)
STEEP = 0.08         # steep_threshold
K_FREE = 2           # kFreeFramesToClear

GRID_RES = 0.05
GRID_ORIGIN = (-6.0, -10.0)
GRID_W, GRID_H = 560, 400


def load_bag(topics):
    con = sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True)
    cur = con.cursor()
    ids = {n: cur.execute('select id from topics where name=?', (n,)).fetchone()[0] for n in topics}
    out = {}
    for name in topics:
        rows = cur.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                           (ids[name],)).fetchall()
        out[name] = rows
    return out


def pc2_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)


def odom_pos(blob):
    m = deserialize_message(blob, Odometry)
    p = m.pose.pose.position
    return np.array([p.x, p.y, p.z])


def extract_stage1():
    rows = load_bag(['/lio/base_odom', '/ma_nav/map/occ', '/ma_nav/grid_map_occ', '/lio/cloud_world'])
    base_ts = np.array([t for t, _ in rows['/lio/base_odom']], np.int64)
    base = np.array([odom_pos(b) for _, b in rows['/lio/base_odom']])
    occ_ts, occ = [], []
    for t, b in rows['/ma_nav/map/occ']:
        occ_ts.append(t)
        occ.append(pc2_xyz(deserialize_message(b, PointCloud2)))
    occ_ts = np.array(occ_ts, np.int64)
    grid_ts, grids = [], []
    for t, b in rows['/ma_nav/grid_map_occ']:
        m = deserialize_message(b, OccupancyGrid)
        grid_ts.append(t)
        grids.append(np.frombuffer(m.data, np.int8).reshape(m.info.height, m.info.width).copy())
    grid_ts = np.array(grid_ts, np.int64)
    cw_ts = np.array([t for t, _ in rows['/lio/cloud_world']], np.int64)
    return dict(base_ts=base_ts, base=base, occ_ts=occ_ts, occ=occ,
                grid_ts=grid_ts, grids=grids, cw_ts=cw_ts)


def nearest(ts, t):
    i = int(np.searchsorted(ts, t))
    cands = [c for c in (i - 1, i) if 0 <= c < len(ts)]
    return min(cands, key=lambda k: abs(ts[k] - t))


# ================= 旧逻辑（dc1024b）复刻 =================
def replay_old(d, size=SIZE_X, warm_start=None, verbose=False):
    occ_ts, occ = d['occ_ts'], d['occ']
    base_ts, base = d['base_ts'], d['base']
    grid_ts, grids = d['grid_ts'], d['grids']
    pair = [nearest(grid_ts, t) for t in occ_ts]
    pubs = [(g == 100) for g in grids]

    W = int(np.ceil(size / RES))
    ng = W * W
    occ2d = np.zeros((GRID_H, GRID_W), bool) if warm_start is None else warm_start.copy()
    streak = np.zeros((GRID_H, GRID_W), np.int16)
    last = np.full((GRID_H, GRID_W), -10, np.int32)
    fid = 0
    per_iou, white_n, snapshots = [], [], []
    t0 = time.time()
    for k in range(len(occ)):
        t = occ_ts[k]
        pi = nearest(base_ts, t)
        px, py = base[pi, 0], base[pi, 1]
        ox = np.round((px - size * 0.5) / RES) * RES
        oy = np.round((py - size * 0.5) / RES) * RES
        pts = occ[k]
        gx = np.floor((pts[:, 0] - ox) / RES).astype(np.int64)
        gy = np.floor((pts[:, 1] - oy) / RES).astype(np.int64)
        valid = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < W)
        ci = gy[valid] * W + gx[valid]
        zv = pts[valid, 2]
        cnt = np.bincount(ci, minlength=ng)
        ground = np.full(ng, np.inf)
        np.minimum.at(ground, ci, zv)
        maxz = np.full(ng, -np.inf)
        np.maximum.at(maxz, ci, zv)
        vc = (cnt > 0).reshape(W, W)
        ground = np.where(vc, ground.reshape(W, W), np.nan)
        maxz = np.where(vc, maxz.reshape(W, W), np.nan)

        pad = np.pad(ground, 1, constant_values=np.nan)
        sw = np.lib.stride_tricks.sliding_window_view(pad, (KERNEL, KERNEL))
        g = np.nanmedian(sw, axis=(-2, -1))

        ninf = np.where(np.isfinite(ground), ground, np.inf)
        gmin = minimum_filter(ninf, size=2 * KR + 1, mode='constant', cval=np.inf)
        gmin = np.where(np.isfinite(gmin), gmin, np.nan)
        zref = np.where(g - gmin > JUMP, gmin, g)

        stack = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                sh = np.full_like(zref, np.nan)
                ys = slice(max(0, dy), W + min(0, dy))
                yt = slice(max(0, -dy), W + min(0, -dy))
                xs = slice(max(0, dx), W + min(0, dx))
                xt = slice(max(0, -dx), W + min(0, -dx))
                sh[ys, xs] = zref[yt, xt]
                stack.append(np.abs(zref - sh))
        grad = np.nanmax(np.stack(stack), axis=0)
        grad = np.where(np.isnan(grad), 0.0, grad)

        refb = zref.reshape(-1)[ci]
        m2 = zv > refb + BAND
        zlow = np.full(ng, np.inf)
        np.minimum.at(zlow, ci[m2], zv[m2])
        zlow = np.where(np.isfinite(zlow), zlow, np.nan).reshape(W, W)

        obs = np.zeros((W, W), bool)
        fre = np.zeros((W, W), bool)
        nolow = vc & np.isnan(zlow)
        obs[nolow & (grad > STEEP)] = True
        fre[nolow & (grad <= STEEP)] = True
        haslow = vc & ~np.isnan(zlow)
        okclear = haslow & ((zlow - zref) >= ROBOT_H)
        obs[haslow & ~okclear] = True
        fre[okclear] = True

        fid += 1
        gyv, gxv = np.nonzero(obs)
        if len(gyv):
            cxs = ox + (gxv + 0.5) * RES
            cys = oy + (gyv + 0.5) * RES
            ixs = np.floor((cxs - 0.25 * GRID_RES - GRID_ORIGIN[0]) / GRID_RES).astype(np.int64)
            iys = np.floor((cys - 0.25 * GRID_RES - GRID_ORIGIN[1]) / GRID_RES).astype(np.int64)
            ixs = np.clip(ixs, 0, GRID_W - 1)
            iys = np.clip(iys, 0, GRID_H - 1)
            occ2d[iys, ixs] = True
            streak[iys, ixs] = 0
        gyv, gxv = np.nonzero(fre)
        if len(gyv):
            cxs = ox + (gxv + 0.5) * RES
            cys = oy + (gyv + 0.5) * RES
            ixs = np.floor((cxs - 0.25 * GRID_RES - GRID_ORIGIN[0]) / GRID_RES).astype(np.int64)
            iys = np.floor((cys - 0.25 * GRID_RES - GRID_ORIGIN[1]) / GRID_RES).astype(np.int64)
            ixs = np.clip(ixs, 0, GRID_W - 1)
            iys = np.clip(iys, 0, GRID_H - 1)
            sel = occ2d[iys, ixs]
            if sel.any():
                ii, jj = iys[sel], ixs[sel]
                st = np.where(last[ii, jj] == fid - 1, streak[ii, jj] + 1, 1)
                streak[ii, jj] = st
                last[ii, jj] = fid
                cl = st >= K_FREE
                if cl.any():
                    occ2d[ii[cl], jj[cl]] = False
                    streak[ii[cl], jj[cl]] = 0
        pub = pubs[pair[k]]
        inter = (occ2d & pub).sum()
        un, pn = int(occ2d.sum()), int(pub.sum())
        per_iou.append(inter / max(1, un + pn - inter))
        white_n.append(un)
        snapshots.append(occ2d.copy())
        if verbose and k % 50 == 0:
            print(f'  frame {k:3d}: replay_white={un:5d} pub_white={pn:5d} IoU={per_iou[-1]:.3f}'
                  f' ({time.time()-t0:.1f}s)', flush=True)
    return dict(per_iou=np.array(per_iou), white_n=np.array(white_n),
                snapshots=snapshots, occ2d=occ2d, pair=pair, pubs=pubs)


def report_old(d, r):
    pub_last = r['pubs'][r['pair'][-1]]
    per = r['per_iou']
    print('旧逻辑复刻（bag 内 2D 地图对照）:')
    print(f'  per-frame IoU: min={per.min():.3f} p10={np.percentile(per,10):.3f} '
          f'p50={np.median(per):.3f} p90={np.percentile(per,90):.3f} last={per[-1]:.3f}')
    print(f'  white count: replay first5={r["white_n"][:5].tolist()} last5={r["white_n"][-5:].tolist()}')
    print(f'               pub    first5={[int(r["pubs"][r["pair"][k]].sum()) for k in range(5)]} '
          f'last5={[int(r["pubs"][r["pair"][k]].sum()) for k in range(len(r["pair"])-5, len(r["pair"]))]}')
    occ2d = r['occ2d']
    inter = (occ2d & pub_last).sum()
    print(f'  FINAL: replay={int(occ2d.sum())} pub={int(pub_last.sum())} inter={int(inter)} '
          f'recall={inter/max(1,pub_last.sum()):.3f} precision={inter/max(1,occ2d.sum()):.3f} '
          f'IoU={inter/max(1,(occ2d|pub_last).sum()):.3f}')
    only_pub = pub_last & ~occ2d
    only_rep = occ2d & ~pub_last
    near_pub = binary_dilation(pub_last, np.ones((5, 5), bool), iterations=2)
    near_rep = binary_dilation(occ2d, np.ones((5, 5), bool), iterations=2)
    print(f'  final mismatch: pub-only={int(only_pub.sum())} replay-only={int(only_rep.sum())}')
    print(f'    pub-only near replay white(<10cm): {int((only_pub & near_rep).sum())}/{int(only_pub.sum())}')
    print(f'    replay-only near pub white(<10cm): {int((only_rep & near_pub).sum())}/{int(only_rep.sum())}')
    return dict(precision=inter/max(1,occ2d.sum()), recall=inter/max(1,pub_last.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['old', 'a2'])
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--size', type=float, default=SIZE_X)
    args = ap.parse_args()
    print(f'terrain size_x={SIZE_X:.3f} W={int(np.ceil(SIZE_X/RES))}', flush=True)
    d = extract_stage1()
    print(f'extract: base={len(d["base"])} occ={len(d["occ"])} grid={len(d["grids"])} '
          f'cloud_world={len(d["cw_ts"])}', flush=True)
    if args.mode == 'old':
        r = replay_old(d, size=args.size, verbose=args.verbose)
        report_old(d, r)
    else:
        print('A2 mode not implemented yet')


if __name__ == '__main__':
    main()
