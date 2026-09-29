#!/usr/bin/env python3
"""合成の AWSIM bag から awsim_to_bag.py が作った bag・真値・パラメータが、合成したときの値に合うかを確かめる。

  python3 check_awsim.py <出力.mcap>

awsim_to_bag.py は --gnss-off-time 10:5 --initial-pose 0,0 --drop-lidar 20:2 で動かしておく（run_tests.sh）。
"""
import csv
import math
import struct
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import mcap_io as IO  # noqa: E402
import rigid as A  # noqa: E402
from make_fake_awsim_bag import ORIGIN, T_BASE_LIDAR  # noqa: E402

E0, N0 = 300000.0, 3900000.0
errors = []


def check(cond, msg):
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond:
        errors.append(msg)


def main():
    bag = Path(sys.argv[1])
    stem = bag.with_suffix("")
    msgs = {}
    for topic, _typ, t_log, data in IO.read_messages(bag, None):
        msgs.setdefault(topic, []).append((t_log, data))
    counts = {k: len(v) for k, v in msgs.items()}
    print("topics:", counts)
    check(set(counts) == {"/sensing/imu", "/sensing/odom", "/sensing/gnss/fix", "/sensing/lidar/points", "/tf_static",
                          "/groundtruth/pose", "/initialpose"}, "topics")
    for k, v in msgs.items():
        ts = [t for t, _ in v]
        check(ts == sorted(ts), f"{k}: time ordered")

    gt = list(csv.DictReader(open(Path(str(stem) + "_groundtruth.csv"))))
    g_t = np.array([float(g["t"]) for g in gt])
    g_xy = np.array([[float(g["x"]), float(g["y"])] for g in gt])
    g_yaw = np.array([float(g["yaw"]) for g in gt])
    check(np.allclose(g_xy[0], ORIGIN + [E0, N0], atol=0.01), f"groundtruth csv in UTM {g_xy[0]}")
    dur = g_t[-1] - g_t[0]

    # IMU: base_link の向き（最初は止まっていて水平）、角速度 z が真値の yaw rate に合う
    imu = []
    for _t, d in msgs["/sensing/imu"]:
        t, frame, gyro, acc = IO.imu(d)
        imu.append((t, *gyro, *acc))
    imu = np.array(imu)
    check(frame == "base_link", f"imu frame {frame}")
    check(abs(imu[0, 6] - 9.81) < 0.1 and np.hypot(imu[0, 4], imu[0, 5]) < 0.1, f"imu acc in base {np.round(imu[0, 4:], 3)}")
    yr = np.gradient(np.unwrap(g_yaw), g_t)
    yr_i = np.interp(imu[:, 0], g_t, yr)
    ok = (imu[:, 0] > g_t[0] + 1) & (imu[:, 0] < g_t[-1] - 1)
    err = np.sqrt(np.mean((imu[ok, 3] - yr_i[ok]) ** 2))
    check(err < 0.02 and np.max(np.abs(yr_i[ok])) > 0.1, f"gyro z = yaw rate（rms {err:.4f} rad/s）")
    check(np.max(np.abs(imu[ok, 1:3])) < 0.02, "gyro x, y ≈ 0")

    # ODOM
    od = []
    for _t, d in msgs["/sensing/odom"]:
        c = IO.Cdr(d)
        t, frame = c.header()
        child = c.string()
        c.f64(7 + 36)
        v = c.f64(6)
        od.append((t, v[0], v[5]))
    od = np.array(od)
    check(frame == "odom" and child == "base_link", f"odom frames {frame} → {child}")
    check(abs(np.max(od[:, 1]) - 1.67) < 0.1, f"odom max speed {np.max(od[:, 1]):.2f}")

    # GNSS: lat/lon → UTM ≈ 真値 + lever（z 軸まわりだけなので水平は真値の位置）、外した区間は status -1
    from pyproj import Transformer
    fwd = Transformer.from_crs("EPSG:4326", "EPSG:32654", always_xy=True)
    gn = []
    for _t, d in msgs["/sensing/gnss/fix"]:
        c = IO.Cdr(d)
        t, frame = c.header()
        st = struct.unpack_from("<b", c.b, c.p)[0]
        c.p += 1
        c.p += (-(c.p - 4)) % 2
        c.p += 2
        lat, lon, alt = c.f64(3)
        cov = c.f64(9)
        gn.append((t, st, lat, lon, alt, cov[0]))
    gn = np.array(gn)
    check(frame == "gnss_link", f"gnss frame {frame}")
    rate = len(gn) / dur
    check(abs(rate - 10) < 0.5, f"gnss rate {rate:.1f} Hz")
    t0 = gn[0, 0]
    off = (gn[:, 0] >= t0 + 10) & (gn[:, 0] < t0 + 15)
    check(np.all(gn[off, 1] == -1) and np.all(gn[~off, 1] == 2) and off.sum() == 50,
          f"status: -1 for 10..15 s（{int(off.sum())}）, 2 elsewhere")
    e, n = fwd.transform(gn[:, 3], gn[:, 2])
    ref = np.c_[np.interp(gn[:, 0], g_t, g_xy[:, 0]), np.interp(gn[:, 0], g_t, g_xy[:, 1])]
    d = np.hypot(e - ref[:, 0], n - ref[:, 1])
    check(np.sqrt(np.mean(d[~off] ** 2)) < 0.06, f"gnss horizontal rms {np.sqrt(np.mean(d[~off] ** 2)):.3f} m")
    check(abs(np.median(gn[:, 4]) - 1.5) < 0.1, f"gnss alt {np.median(gn[:, 4]):.2f}（lever z 1.5）")

    # 点群・tf_static・初期姿勢
    n_pc = counts["/sensing/lidar/points"]
    expect = int(round(dur * 10)) - 20
    check(abs(n_pc - expect) <= 2, f"scans {n_pc}（約 {expect}: --drop-lidar 20:2 で 20 個減る）")
    _t, _f, xyz = IO.cloud_xyz(msgs["/sensing/lidar/points"][0][1])
    check(_f == "velodyne_top" and len(xyz) > 100, f"cloud frame {_f}, {len(xyz)} points")
    c = IO.Cdr(msgs["/tf_static"][0][1])
    check(c.u32() == 1, "tf_static 1 transform")
    _t, parent = c.header()
    child = c.string()
    p = np.array(c.f64(3))
    q = np.array(c.f64(4))
    check(parent == "base_link" and child == "velodyne_top", f"tf {parent} → {child}")
    check(np.allclose(p, T_BASE_LIDAR[:3, 3], atol=0.05), f"tf translation {np.round(p, 3)}")
    c = IO.Cdr(msgs["/initialpose"][0][1])
    ti, frame = c.header()
    p = np.array(c.f64(3))
    i = int(np.argmin(np.abs(g_t - ti)))
    check(frame == "map" and np.hypot(*(p[:2] - g_xy[i])) < 0.02, f"initialpose = groundtruth {np.round(p[:2], 2)}")

    # パラメータ
    prm = yaml.safe_load(Path(str(stem) + "_params.yaml").read_text())["gll_localizer"]["ros__parameters"]
    check(prm["gnss"]["utm_zone"] == 54 and prm["gnss"]["lever_arm"] == [0.0, 0.0, 1.5], "gnss params")
    check(abs(prm["attitude"]["static_init_time"] - 3.0) < 0.05, f"static_init_time {prm['attitude']['static_init_time']}")
    check(prm["imu"]["rotation_rpy_deg"] == [0.0, 0.0, 0.0], "imu rotation 0（base_link に回して出している）")
    check(np.allclose(prm["lidar"]["extrinsic_xyz"], T_BASE_LIDAR[:3, 3], atol=0.05), f"extrinsic_xyz {prm['lidar']['extrinsic_xyz']}")
    rpy = np.rad2deg(A.rpy_of(T_BASE_LIDAR[:3, :3]))
    check(np.allclose(prm["lidar"]["extrinsic_rpy_deg"], rpy, atol=0.3), f"extrinsic_rpy_deg {prm['lidar']['extrinsic_rpy_deg']}")
    check(math.isclose(prm["lidar"]["base_link_height"], 0.0), "base_link_height 0")

    if errors:
        print(f"\n{len(errors)} check(s) failed")
        sys.exit(1)
    print("\nawsim convert checks passed")


if __name__ == "__main__":
    main()
