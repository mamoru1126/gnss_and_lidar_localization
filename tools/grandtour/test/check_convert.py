#!/usr/bin/env python3
"""合成ミッションから作った bag・真値・パラメータ・地図が、合成したときの真値に合うかを確かめる（ROS なしで動く）。

  python3 check_convert.py <bag.mcap> <地図のフォルダ>

MCAP は、このファイルの小さな読み出し（圧縮なし・チャンクあり。make_bag.py の書き出しの形）で読む。
ROS 2 そのもので読めるかは check_bag_ros2.py（CI の ros2 ジョブ）で確かめる。
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import analysis as A  # noqa: E402
from make_fake_missions import BASE_HEIGHT, T_BASE_BOXBASE, T_BOXBASE_LIVOX  # noqa: E402

errors = []


def check(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond:
        errors.append(msg)


# ------------------------------------------------------------------ MCAP の読み出し（テスト用の最小限）
def read_mcap(path):
    b = Path(path).read_bytes()
    assert b[:8] == b"\x89MCAP0\r\n"
    pos, channels, msgs = 8, {}, {}

    def records(buf, pos, end):
        while pos + 9 <= end:
            op, ln = struct.unpack_from("<BQ", buf, pos)
            yield op, buf[pos + 9: pos + 9 + ln]
            pos += 9 + ln

    def handle(op, body):
        if op == 0x04:
            cid, _sid = struct.unpack_from("<HH", body, 0)
            n = struct.unpack_from("<I", body, 4)[0]
            channels[cid] = body[8:8 + n].decode()
        elif op == 0x05:
            cid, _seq, t, _pt = struct.unpack_from("<HIQQ", body, 0)
            msgs.setdefault(channels[cid], []).append((t, body[22:]))

    for op, body in records(b, pos, len(b) - 8):
        if op == 0x06:  # chunk
            n_comp = struct.unpack_from("<I", body, 28)[0]
            recs_len = struct.unpack_from("<Q", body, 32 + n_comp)[0]
            data = body[40 + n_comp: 40 + n_comp + recs_len]
            for op2, body2 in records(data, 0, len(data)):
                handle(op2, body2)
        elif op in (0x04, 0x05):
            handle(op, body)
        elif op == 0x0F:
            break
    return msgs


class Rd:
    """CDR（リトルエンディアン）の読み出し。"""

    def __init__(self, data):
        self.b, self.p = data, 4

    def _al(self, n):
        self.p += (-(self.p - 4)) % n

    def u32(self):
        self._al(4)
        v = struct.unpack_from("<I", self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        self._al(4)
        v = struct.unpack_from("<i", self.b, self.p)[0]
        self.p += 4
        return v

    def f64(self, n=1):
        self._al(8)
        v = struct.unpack_from(f"<{n}d", self.b, self.p)
        self.p += 8 * n
        return v if n > 1 else v[0]

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode()
        self.p += n
        return s

    def header(self):
        sec, nsec = self.i32(), self.u32()
        return sec + nsec * 1e-9, self.string()


def main():
    bag, map_dir = Path(sys.argv[1]), Path(sys.argv[2])
    stem = bag.with_suffix("")
    msgs = read_mcap(bag)
    counts = {k: len(v) for k, v in msgs.items()}
    print("topics:", counts)
    check(set(counts) == {"/sensing/imu", "/sensing/odom", "/sensing/lidar/points", "/tf_static", "/initialpose",
                          "/groundtruth/pose"}, "topics")
    for k, v in msgs.items():
        ts = [t for t, _ in v]
        check(ts == sorted(ts), f"{k}: time ordered")
    dur = (msgs["/sensing/imu"][-1][0] - msgs["/sensing/imu"][0][0]) * 1e-9
    check(abs(counts["/sensing/imu"] / dur - 200) < 5, f"IMU rate {counts['/sensing/imu'] / dur:.1f} Hz")
    check(abs(counts["/sensing/odom"] / dur - 50) < 3, f"ODOM rate {counts['/sensing/odom'] / dur:.1f} Hz")
    check(counts["/sensing/lidar/points"] == 30 - 5, f"scans {counts['/sensing/lidar/points']}（30 から --drop-lidar の 5 を除く）")
    check(counts["/tf_static"] == 1 and counts["/initialpose"] == 1, "tf_static / initialpose once")

    # IMU: base の向き・m/s²（最初は止まっていて水平）
    r = Rd(msgs["/sensing/imu"][0][1])
    _, frame = r.header()
    r.f64(4)
    oc = r.f64(9)
    gyro = r.f64(3)
    r.f64(9)
    acc = np.array(r.f64(3))
    check(frame == "base_link" and oc[0] == -1.0, f"imu frame {frame}")
    check(abs(acc[2] - 9.81) < 0.1 and np.hypot(acc[0], acc[1]) < 0.1, f"imu acc in base {np.round(acc, 3)}")

    # 初期姿勢 = 真値（--init-error 0,0）
    import csv
    gt = list(csv.DictReader(open(Path(str(stem) + "_groundtruth.csv"))))
    t_gt = np.array([float(g["t"]) for g in gt])
    r = Rd(msgs["/initialpose"][0][1])
    t_init, frame = r.header()
    p = np.array(r.f64(3))
    q = np.array(r.f64(4))
    i = int(np.argmin(np.abs(t_gt - t_init)))
    g = gt[i]
    check(frame == "map", "initialpose frame map")
    check(np.hypot(p[0] - float(g["x"]), p[1] - float(g["y"])) < 0.01, f"initialpose = groundtruth {p[:2]} / {g['x']}, {g['y']}")
    yaw_i = A.yaw_of(A.quat_to_rot(q))
    check(abs(A.wrap(yaw_i - float(g["yaw"]))) < 0.01 and abs(float(g["yaw"])) < 0.05, f"initialpose yaw {yaw_i:.3f}（東向き = 0）")

    # パラメータ
    prm = yaml.safe_load(Path(str(stem) + "_params.yaml").read_text())["gll_localizer"]["ros__parameters"]
    T = T_BASE_BOXBASE @ T_BOXBASE_LIVOX
    rpy = np.rad2deg(A.rpy_of(T[:3, :3]))
    check(np.allclose(prm["lidar"]["extrinsic_xyz"], T[:3, 3], atol=1e-3), f"extrinsic_xyz {prm['lidar']['extrinsic_xyz']}")
    rr, pp, yy = np.deg2rad(prm["lidar"]["extrinsic_rpy_deg"])  # rpyToMatrix で行列に戻して比べる
    Rb = A.rot_z(yy) @ np.array([[math.cos(pp), 0, math.sin(pp)], [0, 1, 0], [-math.sin(pp), 0, math.cos(pp)]]) @ \
        np.array([[1, 0, 0], [0, math.cos(rr), -math.sin(rr)], [0, math.sin(rr), math.cos(rr)]])
    check(np.allclose(Rb, T[:3, :3], atol=1e-3), f"extrinsic_rpy_deg {prm['lidar']['extrinsic_rpy_deg']}（{np.round(rpy, 3)}）")
    check(abs(prm["lidar"]["base_link_height"] - BASE_HEIGHT) < 0.03, f"base_link_height {prm['lidar']['base_link_height']}")
    check(abs(prm["attitude"]["static_init_time"] - 2.7) < 0.1, f"static_init_time {prm['attitude']['static_init_time']}（3 s 止まっている）")
    check(prm["gnss"]["utm_zone"] == 32 and prm["lidar"]["time_field"] == "", "utm_zone / time_field")

    # 地図
    head = (map_dir / "map.pcd").read_bytes()
    k = head.index(b"DATA binary\n") + len(b"DATA binary\n")
    n = int(head[:k].decode().split("POINTS ")[1].split()[0])
    pts = np.frombuffer(head[k:], dtype="<f4").reshape(n, 4)
    check(n > 1000, f"map points {n}")
    check(abs(np.median(pts[:, 2]) + BASE_HEIGHT) < 0.02, f"map ground z {np.median(pts[:, 2]):.3f}（-{BASE_HEIGHT}）")
    near0 = np.hypot(pts[:, 0], pts[:, 1]) < 0.8
    check(not np.any(near0 & (pts[:, 2] > -BASE_HEIGHT + 0.05)), "robot's own points removed")
    my = yaml.safe_load((map_dir / "maps.yaml").read_text())
    an = my["map_groups"][0]["anchor"]
    check(my["utm"]["zone"] == 32 and my["map_groups"][0]["tile_index"] == "tiles/tile_index.yaml", "maps.yaml")
    g0 = gt[0]
    check(abs(an["grid_heading_deg"] - 90.0) < 0.1, f"anchor grid_heading {an['grid_heading_deg']}（地図の x = 東）")
    check(np.hypot(an["easting"] - float(g0["x"]), an["northing"] - float(g0["y"])) < 0.05,
          f"anchor ({an['easting']:.2f}, {an['northing']:.2f}) = ETH-1 の最初の base（{g0['x']}, {g0['y']}）")

    if errors:
        print(f"\n{len(errors)} check(s) failed")
        sys.exit(1)
    print("\nconvert checks passed")


if __name__ == "__main__":
    main()
