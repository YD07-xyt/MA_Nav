#!/usr/bin/env python3
"""离线抽取：地面生长方案复算用（只读 bag，缓存到 /dev/shm）。

缓存：
  - /lio/base_odom 全部位姿（base_link，与地同高）
  - /ma_nav/map/occ 每隔 N 帧一份 + 最后一帧（terrain 的输入 real_occ_map）
  - /ma_nav/grid_map_occ 最后一帧（旧方案结果，对照）
"""
import sqlite3, pickle, sys, time
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid

BAG = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/xyt/map/src/MA_Nav/rosbag/rosbag2_2026_10_20-20_15_24/rosbag2_2026_10_20-20_15_24_0.db3'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/dev/shm/ground_grow.pkl'
OCC_STRIDE = 10

con = sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True)
cur = con.cursor()
tid = {n: i for i, n in cur.execute('select id, name from topics')}


def rows(name):
    return cur.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[name],))


def pc2_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)


t0 = time.time()
base = []
for ts, b in rows('/lio/base_odom'):
    m = deserialize_message(b, Odometry)
    p, q = m.pose.pose.position, m.pose.pose.orientation
    base.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
base = np.array(base, dtype=np.float64)
print('base_odom', len(base), f'{time.time()-t0:.1f}s', flush=True)

occ, last = [], None
for i, (ts, b) in enumerate(rows('/ma_nav/map/occ')):
    last = (ts, b)
    if i % OCC_STRIDE == 0:
        occ.append((ts, pc2_xyz(deserialize_message(b, PointCloud2))))
occ.append((last[0], pc2_xyz(deserialize_message(last[1], PointCloud2))))
print('occ frames', len(occ), 'last pts', len(occ[-1][1]), f'{time.time()-t0:.1f}s', flush=True)

g = None
for ts, b in rows('/ma_nav/grid_map_occ'):
    g = (ts, b)
m = deserialize_message(g[1], OccupancyGrid)
grid = np.frombuffer(m.data, dtype=np.int8).reshape(m.info.height, m.info.width).copy()
gmeta = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
print('grid', grid.shape, gmeta, flush=True)

pickle.dump(dict(base=base, occ=occ, grid=grid, gmeta=gmeta), open(OUT, 'wb'), protocol=4)
print('saved', OUT, f'{time.time()-t0:.1f}s')
