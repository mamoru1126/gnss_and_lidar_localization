#!/usr/bin/env python3
"""AWSIM で録った bag（ROS 2 Humble、MCAP）と同じトピック・型の、小さな合成の bag と地図のタイルを作る（テスト用）。

  python3 make_fake_awsim_bag.py <出力のフォルダ>

<出力>/bag/fake.mcap   真値・IMU・速度・点群
<出力>/tiles/          地図（tiled_pcd_map_tiler と同じ形のタイルと tile_index.yaml。共分散なし）
<出力>/map.pcd         同じ地図の PCD（tiled_pcd_map_tiler の入力。推定ノードで使うタイルはこれから作る）
<出力>/route.txt       走った経路（「x y」の行）
場所は、壁で囲んだ 70 m × 50 m に、中の建物、柱と箱（地図上の初期化で位置が一つに決まるように不規則に置く）。
車は西新宿の地図座標のあたりで、最初に 5 s 止まり、40 m × 20 m の四角を 6 km/h で 1 周する（drive_core の追従で走らせる）。
IMU と LiDAR の取り付けは、下の R_BASE_IMU と T_BASE_LIDAR（真値）。
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[2] / "tools" / "tile_demo"))
import make_bag as MB  # noqa: E402
import rigid as A  # noqa: E402
from drive_core import PurePursuit  # noqa: E402

G = 9.80665
ORIGIN = np.array([81200.0, 49800.0])  # 地図座標（MGRS 54SUE の中）
R_BASE_IMU = A.rpy_to_rot(math.pi, 0.0, math.radians(90))  # 上下逆さまで 90° 回った IMU
# AWSIM v1.3.1 の velodyne_top と同じく、センサキットの上で約 90° 回っている
T_BASE_LIDAR = A.se3(A.rpy_to_rot(0.0, math.radians(1.0), math.radians(88.0)), [0.9, 0.0, 2.0])
ROUTE = ORIGIN + np.array([[0, 0], [40, 0], [40, 20], [0, 20], [0, 0.5]], dtype=float)

MB.MSGDEFS.update({
    "sensor_msgs/msg/Imu": "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
                           "geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
                           "geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance",
    "autoware_vehicle_msgs/msg/VelocityReport": "std_msgs/Header header\nfloat32 longitudinal_velocity\n"
                                                     "float32 lateral_velocity\nfloat32 heading_rate",
})
MB.DEPS.update({
    "sensor_msgs/msg/Imu": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/Quaternion",
                            "geometry_msgs/msg/Vector3"],
    "autoware_vehicle_msgs/msg/VelocityReport": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time"],
    "geometry_msgs/msg/PoseStamped": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/Pose",
                                      "geometry_msgs/msg/Point", "geometry_msgs/msg/Quaternion"],
})


def world_points():
    """道の両側の壁（高さ 0〜4 m、道の外側 6 m）と地面。"""
    rng = np.random.default_rng(1)
    pts = []
    x0, y0 = ORIGIN - 15
    x1, y1 = ORIGIN + [55, 35]
    g = np.mgrid[x0:x1:0.4, y0:y1:0.4].reshape(2, -1).T
    pts.append(np.c_[g, np.zeros(len(g))])
    for (ax, ay), (bx, by) in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
        n = int(math.hypot(bx - ax, by - ay) / 0.2)
        s = np.linspace(0, 1, n)
        for z in np.arange(0.0, 4.0, 0.25):
            pts.append(np.c_[ax + s * (bx - ax), ay + s * (by - ay), np.full(n, z)])
    # 中の建物（四角の内側）
    cx0, cy0 = ORIGIN + [8, 5]
    cx1, cy1 = ORIGIN + [32, 15]
    for (ax, ay), (bx, by) in [((cx0, cy0), (cx1, cy0)), ((cx1, cy0), (cx1, cy1)), ((cx1, cy1), (cx0, cy1)), ((cx0, cy1), (cx0, cy0))]:
        n = int(math.hypot(bx - ax, by - ay) / 0.2)
        s = np.linspace(0, 1, n)
        for z in np.arange(0.0, 6.0, 0.25):
            pts.append(np.c_[ax + s * (bx - ax), ay + s * (by - ay), np.full(n, z)])
    # 形の手がかり（柱と箱を不規則に置く）。壁と建物だけだと四角が対称で、地図上の初期化で候補が絞れない
    for cx, cy, r in [(-6, -5, 0.3), (12, -7, 0.5), (27, -4, 0.3), (47, 3, 0.4), (46, 26, 0.3), (18, 27, 0.6),
                      (-5, 12, 0.4), (-8, 28, 0.3), (36, 10, 0.3)]:
        a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        for z in np.arange(0.0, 5.0, 0.25):
            pts.append(np.c_[ORIGIN[0] + cx + r * np.cos(a), ORIGIN[1] + cy + r * np.sin(a), np.full(len(a), z)])
    for (bx, by, w, d, h) in [(4, -9, 3, 1.5, 1.5), (35, -9, 1.5, 2.5, 2.5), (48, 14, 2, 4, 1.2), (-9, 5, 2, 2, 3.0)]:
        g = np.mgrid[0:w:0.2, 0:d:0.2, 0:h:0.2].reshape(3, -1).T
        edge = (np.isclose(g[:, 0], 0) | (g[:, 0] > w - 0.21) | np.isclose(g[:, 1], 0) | (g[:, 1] > d - 0.21)
                | (g[:, 2] > h - 0.21))
        pts.append(g[edge] + [ORIGIN[0] + bx, ORIGIN[1] + by, 0.0])
    p = np.vstack(pts)
    return p + rng.normal(0, 0.01, p.shape)


def drive():
    """止まって 5 s、それから四角を 1 周。100 Hz の (t, x, y, yaw, v, yaw_rate)。"""
    dt = 0.01
    pp = PurePursuit(ROUTE, speed=1.67)
    x, y, yaw = ROUTE[0, 0], ROUTE[0, 1], 0.0
    pp.start(x, y)
    v, out, t = 0.0, [], 0.0
    while True:
        if t < 5.0:
            steer, acc, done = 0.0, 0.0, False
        else:
            steer, _vr, acc, done = pp.step(x, y, yaw, v)
        v = max(0.0, v + acc * dt)
        wz = v / pp.wb * math.tan(steer)
        out.append((t, x, y, yaw, v, wz))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += wz * dt
        t += dt
        if (done and v < 0.01) or t > 200:
            break
    return np.array(out)


def main():
    out = Path(sys.argv[1])
    (out / "bag").mkdir(parents=True, exist_ok=True)
    (out / "tiles" / "tiles").mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "route.txt", ROUTE, fmt="%.3f")
    tr = drive()
    t0 = 1_700_000_000.0
    ts, xs, ys, yaws, vs, wzs = tr.T
    ts = ts + t0
    z = 0.0
    w = MB.McapWriter(str(out / "bag" / "fake.mcap"))
    qos = MB.qos_yaml(False, 10)
    ch_gt = w.channel(w.schema("geometry_msgs/msg/PoseStamped"), "/awsim/ground_truth/vehicle/pose", qos)
    ch_imu = w.channel(w.schema("sensor_msgs/msg/Imu"), "/sensing/imu/tamagawa/imu_raw", qos)
    ch_vel = w.channel(w.schema("autoware_vehicle_msgs/msg/VelocityReport"), "/vehicle/status/velocity_status", qos)
    ch_pc = w.channel(w.schema("sensor_msgs/msg/PointCloud2"), "/sensing/lidar/top/pointcloud_raw", qos)
    rng = np.random.default_rng(2)
    world = world_points()
    msgs = []
    for i in range(len(ts)):
        c = MB.Cdr()
        c.header(int(ts[i] * 1e9), "base_link")
        for v in (xs[i], ys[i], z, 0.0, 0.0, math.sin(yaws[i] / 2), math.cos(yaws[i] / 2)):
            c.f64(v)
        msgs.append((int(ts[i] * 1e9), ch_gt, c.bytes()))
    acc_long = np.gradient(vs, 0.01)
    for i in range(0, len(ts), 3):  # 約 33 Hz
        f_base = np.array([acc_long[i], vs[i] * wzs[i], G])
        w_base = np.array([0.0, 0.0, wzs[i]])
        f_imu, w_imu = R_BASE_IMU.T @ f_base, R_BASE_IMU.T @ w_base
        c = MB.Cdr()
        c.header(int(ts[i] * 1e9), "tamagawa/imu_link")
        for v in (0, 0, 0, 1):
            c.f64(v)
        for _ in range(9):
            c.f64(0)
        for v in w_imu + rng.normal(0, 0.001, 3):
            c.f64(v)
        for _ in range(9):
            c.f64(0)
        for v in f_imu + rng.normal(0, 0.02, 3):
            c.f64(v)
        for _ in range(9):
            c.f64(0)
        msgs.append((int(ts[i] * 1e9), ch_imu, c.bytes()))
        c = MB.Cdr()
        c.header(int(ts[i] * 1e9), "base_link")
        c.f32(vs[i])
        c.f32(0.0)
        c.f32(wzs[i])
        msgs.append((int(ts[i] * 1e9), ch_vel, c.bytes()))
    for i in range(0, len(ts), 10):  # 10 Hz
        T_wb = A.se3(A.rot_z(yaws[i]), [xs[i], ys[i], z])
        T_wl = T_wb @ T_BASE_LIDAR
        d = world - T_wl[:3, 3]
        near = np.linalg.norm(d, axis=1) < 30.0
        pw = world[near]
        pw = pw[rng.choice(len(pw), min(3000, len(pw)), replace=False)]
        pl = (pw - T_wl[:3, 3]) @ T_wl[:3, :3]
        msgs.append((int(ts[i] * 1e9) + 1000, ch_pc,
                     MB.ser_cloud(int(ts[i] * 1e9), "velodyne_top", pl.astype(np.float32),
                                  np.full(len(pl), 50.0, dtype=np.float32))))
    for t, cid, data in sorted(msgs, key=lambda m: m[0]):
        w.message(cid, t, data)
    w.close()

    # 地図のタイル（20 m、tiled_pcd_map_tiler と同じ形）
    keys = np.floor(world[:, :2] / 20.0).astype(int)
    tiles = []
    for ix, iy in sorted(set(map(tuple, keys))):
        sel = world[(keys[:, 0] == ix) & (keys[:, 1] == iy)].astype("<f4")
        f = f"tiles/{ix}_{iy}.bin"
        (out / "tiles" / f).write_bytes(b"TPCMTIL1" + struct.pack("<QI", len(sel), 0) + sel.tobytes())
        tiles.append(f"  - {{ix: {ix}, iy: {iy}, file: {f}}}")
    (out / "tiles" / "tile_index.yaml").write_text("tile_size: 20.0\ntiles:\n" + "\n".join(tiles) + "\n")
    pc = world.astype("<f4")
    (out / "map.pcd").write_bytes(
        (f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(pc)}\nHEIGHT 1\n"
         f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pc)}\nDATA binary\n").encode() + pc.tobytes())
    print(f"fake AWSIM bag: {out / 'bag' / 'fake.mcap'}（{ts[-1] - ts[0]:.0f} s）、tiles {len(tiles)}")


if __name__ == "__main__":
    main()
