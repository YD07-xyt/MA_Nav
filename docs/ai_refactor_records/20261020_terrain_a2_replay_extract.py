#!/usr/bin/env python3
"""Stage-1 offline extraction for terrain replay validation (A2 方案离线复算).

只读 bag，结果缓存到 /dev/shm（不写仓库数据文件）。

缓存内容：
  - /lio/base_odom 位姿（地图节点实际使用的 odom）
  - /lio/odom 位姿（8Hz，与地图帧对齐）
  - /ma_nav/map/occ 占据点云（= terrain_analyzer 的输入 real_occ_map）
  - /ma_nav/grid_map_occ 2D 占据栅格（旧方案运行结果，用作对照真值）
  - /lio/cloud_world 时间戳（帧对齐参考）
"""
import sqlite3
import pickle
import time
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid

BAG = '/home/xyt/map/src/MA_Nav/rosbag2_2026_10_20-13_46_27/rosbag2_2026_10_20-13_46_27_0.db3'
OUT = '/dev/shm/terrain_stage1.pkl'

con = sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True)
cur = con.cursor()
topic_ids = {}
for name in ['/lio/base_odom', '/lio/odom', '/ma_nav/map/occ', '/ma_nav/grid_map_occ', '/lio/cloud_world']:
    row = cur.execute('select id from topics where name=?', (name,)).fetchone()
    topic_ids[name] = row[0]
print('topic ids:', topic_ids, flush=True)


def fetch(name):
    return cur.execute(
        'select timestamp, data from messages where topic_id=? order by timestamp',
        (topic_ids[name],)).fetchall()


def pc2_xyz(msg):
    off = {fd.name: fd.offset for fd in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'],
                   'formats': ['<f4', '<f4', '<f4'],
                   'offsets': [off['x'], off['y'], off['z']],
                   'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32)


def odom_pose(blob):
    m = deserialize_message(blob, Odometry)
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    return np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float64)


t0 = time.time()
base = [(ts, odom_pose(b)) for ts, b in fetch('/lio/base_odom')]
print('base_odom:', len(base), f'{time.time()-t0:.1f}s', flush=True)
lio = [(ts, odom_pose(b)) for ts, b in fetch('/lio/odom')]
print('lio/odom:', len(lio), f'{time.time()-t0:.1f}s', flush=True)

occ = []
for ts, b in fetch('/ma_nav/map/occ'):
    occ.append((ts, pc2_xyz(deserialize_message(b, PointCloud2))))
print('occ:', len(occ), f'{time.time()-t0:.1f}s', flush=True)

grids = []
for ts, b in fetch('/ma_nav/grid_map_occ'):
    m = deserialize_message(b, OccupancyGrid)
    arr = np.frombuffer(m.data, dtype=np.int8).reshape(m.info.height, m.info.width).copy()
    meta = np.array([m.info.resolution, m.info.origin.position.x, m.info.origin.position.y], np.float64)
    grids.append((ts, arr, meta))
print('grids:', len(grids), 'meta=', grids[0][2], 'shape=', grids[0][1].shape,
      f'{time.time()-t0:.1f}s', flush=True)

cw_ts = np.array([ts for ts, _ in fetch('/lio/cloud_world')], dtype=np.int64)
print('cloud_world ts:', len(cw_ts), flush=True)

data = dict(
    base_ts=np.array([t for t, _ in base], dtype=np.int64),
    base=np.array([p for _, p in base], dtype=np.float64),
    lio_ts=np.array([t for t, _ in lio], dtype=np.int64),
    lio=np.array([p for _, p in lio], dtype=np.float64),
    occ_ts=np.array([t for t, _ in occ], dtype=np.int64),
    occ=occ,
    grid_ts=np.array([t for t, _, _ in grids], dtype=np.int64),
    grids=[(a, m) for _, a, m in grids],
    grid_meta=grids[0][2],
    cw_ts=cw_ts,
)
with open(OUT, 'wb') as f:
    pickle.dump(data, f, protocol=4)
print('saved', OUT, f'{time.time()-t0:.1f}s', flush=True)
