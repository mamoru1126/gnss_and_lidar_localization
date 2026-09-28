#!/usr/bin/env python3
"""GrandTour の Hugging Face 版と同じ構成の、小さな合成ミッションを作る（テスト用）。

  python3 make_fake_missions.py <出力フォルダ>

<出力>/<フォルダ名>/data/<トピック>.tar（中身は Zarr v2 のグループ）と、data/.zgroup、metadata/*.yaml を作る。
ミッション:
  ETH-1  40 m × 20 m の四角を反時計回りに 1 周（最初の 3 s は静止）
  ETH-3  同じ四角の半分を時計回り（逆向き）に歩き、北へ 30 m 出る
  SBB-1  5 km 離れた場所の直線
配列名と属性は、GrandTour のサンプルのノートブック（explore.ipynb）の一覧に合わせる。
静的 TF は、GrandTour のサンプルの get_static_transform と同じ解釈で格納する。
必要なパッケージ: numpy、zarr（3.x）、pyproj。
"""
import shutil
import sys
import tarfile
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analysis as A  # noqa: E402

G = 9.80665
RNG = np.random.default_rng(42)

# 静的 TF の真値（T_parent_child = child の姿勢を parent で表したもの）
T_BASE_BOXBASE = None  # rpy() の定義の後で決める（後ろ向きで、少し傾いている）
T_BOXBASE_LIVOX = A.se3(np.diag([1.0, -1.0, -1.0]), [0.30, 0.0, 0.11])  # 上下逆さまの Mid-360
T_BOXBASE_ADIS = A.se3(np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]), [-0.02, 0.19, 0.08])
BASE_HEIGHT = 0.55
PRISM_LEVER = np.array([-0.20, 0.05, 0.60])  # base 座標系でのプリズムの位置


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


T_BASE_BOXBASE = A.se3(rpy(0.05, -0.03, np.deg2rad(170)), [0.10, 0.0, 0.25])


def tf_entry(T_parent_child, parent, child):
    """GrandTour の tf 属性の形。格納するのは T_child_parent（サンプルの get_static_transform の解釈）。"""
    Ti = A.inv_se3(T_parent_child)
    q = A.rot_to_quat(Ti[:3, :3])
    return dict(base_frame_id=parent, child_frame_id=child,
                translation=dict(x=float(Ti[0, 3]), y=float(Ti[1, 3]), z=float(Ti[2, 3])),
                rotation=dict(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])))


def polyline(points, speed, dt, still=0.0):
    """折れ線を一定の速さで歩く。(t, xy, yaw) を返す。最初に still 秒止まる。"""
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    t_move = np.arange(0, s[-1] / speed, dt)
    si = t_move * speed
    xy = np.column_stack([np.interp(si, s, pts[:, 0]), np.interp(si, s, pts[:, 1])])
    k = np.clip(np.searchsorted(s, si, side="right") - 1, 0, len(seg) - 1)
    d = pts[k + 1] - pts[k]
    yaw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    # 角で向きがゆっくり変わるように平滑化
    w = max(int(1.0 / dt), 1)
    yaw = np.convolve(np.pad(yaw, (w, w), mode="edge"), np.ones(2 * w + 1) / (2 * w + 1), mode="valid")
    n_still = int(still / dt)
    t = np.arange(n_still + len(t_move)) * dt
    xy = np.vstack([np.repeat(xy[:1], n_still, axis=0), xy])
    yaw = np.concatenate([np.full(n_still, yaw[0]), yaw])
    return t, xy, yaw


def mission_truth(points, t0, still):
    dt = 0.005  # 200 Hz（IMU の周期）
    t, xy, yaw = polyline(points, 0.6, dt, still)
    moving = np.linalg.norm(np.gradient(xy, dt, axis=0), axis=1) > 0.05
    roll = np.where(moving, np.deg2rad(2.0) * np.sin(2 * np.pi * 2.0 * t), 0.0)
    pitch = np.where(moving, np.deg2rad(1.5) * np.sin(2 * np.pi * 2.0 * t + 0.7), 0.0)
    R = np.stack([rpy(a, b, c) for a, b, c in zip(roll, pitch, yaw)])
    p = np.column_stack([xy, np.full(len(t), BASE_HEIGHT)])
    return dict(t=t + t0, R=R, p=p, dt=dt)


def pick(truth, rate):
    step = int(round(1.0 / (rate * truth["dt"])))
    return np.arange(0, len(truth["t"]), step)


def odometry_arrays(t, R, p, dt):
    v_world = np.gradient(p, dt, axis=0) if len(p) > 1 else np.zeros_like(p)
    v_body = np.einsum("nji,nj->ni", R, v_world)
    yaw = np.unwrap(A.yaw_of(R))
    w_body = np.column_stack([np.zeros(len(t)), np.zeros(len(t)), np.gradient(yaw, dt)])
    q = np.stack([A.rot_to_quat(r) for r in R])
    return dict(timestamp=t, pose_pos=p, pose_orien=q, twist_lin=v_body, twist_ang=w_body,
                pose_cov=np.zeros((len(t), 36)), twist_cov=np.zeros((len(t), 36)), sequence_id=np.arange(len(t)))


def write_topic(root, name, arrays, attrs):
    g = root.create_group(name)
    for k, v in arrays.items():
        v = np.asarray(v)
        chunks = (min(len(v), 4096),) + v.shape[1:] if v.ndim else None
        a = g.create_array(k, shape=v.shape, dtype=v.dtype, chunks=chunks)
        a[...] = v
    g.attrs.update(attrs)


def scan_points(n):
    """base 座標系の点: 地面（半径 1〜15 m）と、ロボット自身（base の近く）。"""
    r = np.sqrt(RNG.uniform(1.0, 225.0, n))
    a = RNG.uniform(-np.pi, np.pi, n)
    ground = np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n, -BASE_HEIGHT)])
    body = RNG.uniform([-0.4, -0.25, 0.0], [0.4, 0.25, 0.3], (n // 20, 3))
    return np.vstack([ground, body])


def make_mission(out_root, folder, points, origin_en, still, t0, utm_inv, prism_until=0.6, gnss_gap=None):
    from zarr import open_group
    truth = mission_truth(points, t0, still)
    E0, N0 = origin_en
    work = Path(tempfile.mkdtemp())
    root = open_group(str(work / "data"), mode="w", zarr_format=2)
    T_enu_base = [A.se3(R, p) for R, p in zip(truth["R"], truth["p"])]

    # 後処理の GNSS 解（box_base = cpt7_imu の姿勢、ENU）
    i = pick(truth, 20)
    T_box = np.stack([T_enu_base[k] @ T_BASE_BOXBASE for k in i])
    tc = odometry_arrays(truth["t"][i], T_box[:, :3, :3], T_box[:, :3, 3], 0.05)
    write_topic(root, "cpt7_ie_tc_odometry", tc, dict(frame_id="enu_origin", topic="cpt7_ie_tc_odometry",
                                                      description="Novatel Inertial Explorer tightly coupled pose estimation (highest precision)."))
    rt = dict(tc)
    rt["pose_pos"] = tc["pose_pos"] + np.cumsum(RNG.normal(0, 0.01, tc["pose_pos"].shape), axis=0)
    write_topic(root, "cpt7_ie_rt_odometry", rt, dict(frame_id="enu_origin", topic="cpt7_ie_rt_odometry",
                                                      description="Novatel Inertial Explorer real-time pose estimation using PPP (lower precision)."))
    # NavSatFix（10 Hz、途中に欠け）
    i = pick(truth, 10)
    tn = truth["t"][i]
    pb = np.stack([(T_enu_base[k] @ T_BASE_BOXBASE)[:3, 3] for k in i])
    if gnss_gap:
        keep = ~((tn - t0 > gnss_gap[0]) & (tn - t0 < gnss_gap[0] + gnss_gap[1]))
        tn, pb = tn[keep], pb[keep]
    lon, lat = utm_inv(E0 + pb[:, 0], N0 + pb[:, 1])
    cov = np.tile(np.diag([0.02 ** 2, 0.02 ** 2, 0.04 ** 2]).reshape(1, 9), (len(tn), 1))
    write_topic(root, "navsatfix_cpt7_ie_tc", dict(timestamp=tn, lat=lat, long=lon, alt=450.0 + pb[:, 2], cov=cov,
                                                   cov_type=np.full(len(tn), 2), sequence_id=np.arange(len(tn))),
                dict(frame_id="enu_origin", topic="navsatfix_cpt7_ie_tc", description="fake"))
    write_topic(root, "gnss_raw_cpt7_ie_tc", dict(timestamp=tn, position_ecef=np.zeros((len(tn), 3)),
                                                  position_ecef_std=np.full((len(tn), 3), 0.02),
                                                  orientation_hrp=np.zeros((len(tn), 3)),
                                                  orientation_hrp_std=np.zeros((len(tn), 3)), sequence_id=np.arange(len(tn))),
                dict(frame_id="enu_origin", topic="gnss_raw_cpt7_ie_tc", description="fake"))
    # 脚のオドメトリ（base の姿勢を odom で表したもの。twist は base 座標系）
    i = pick(truth, 100)
    T_odom_enu = A.se3(rpy(0, 0, np.deg2rad(30)), [5.0, -3.0, 0.0])
    T_odom_base = np.stack([T_odom_enu @ T_enu_base[k] for k in i])
    leg = odometry_arrays(truth["t"][i], T_odom_base[:, :3, :3], T_odom_base[:, :3, 3], 0.01)
    write_topic(root, "anymal_state_odometry", leg, dict(frame_id="odom", topic="anymal_state_odometry",
                                                         description="ANYmal leg inertial odometry solution."))
    i = pick(truth, 10)
    dl = odometry_arrays(truth["t"][i], truth["R"][i], truth["p"][i], 0.1)
    write_topic(root, "dlio_map_odometry", dl, dict(frame_id="dlio_map", topic="dlio_map_odometry", description="fake DLIO"))
    # プリズム（トータルステーションの座標、20 Hz、ミッションの前半だけ）
    i = pick(truth, 20)
    i = i[: int(len(i) * prism_until)]
    T_station_enu = A.se3(rpy(0, 0, np.deg2rad(-50)), [100.0, 20.0, -3.0])
    pr = np.stack([(T_station_enu @ T_enu_base[k] @ np.append(PRISM_LEVER, 1.0))[:3] for k in i])
    pr += RNG.normal(0, 0.002, pr.shape)
    write_topic(root, "prism_position", dict(timestamp=truth["t"][i], point=pr, sequence_id=np.arange(len(i))),
                dict(frame_id="leica_total_station", topic="prism_position", description="fake prism"))
    # 静的 TF
    tf = {"box_base": tf_entry(T_BASE_BOXBASE, "base", "box_base"),
          "livox_lidar": tf_entry(T_BOXBASE_LIVOX, "box_base", "livox_lidar"),
          "adis16475_imu": tf_entry(T_BOXBASE_ADIS, "box_base", "adis16475_imu")}
    tf["cpt7_imu"] = tf_entry(np.eye(4), "box_base", "cpt7_imu")
    write_topic(root, "tf", dict(timestamp=np.zeros(1)), dict(tf=tf, description="static tf"))
    # Livox（動き補正済み。10 Hz の先頭 30 スキャン）
    i = pick(truth, 10)[:30]
    T_base_livox = T_BASE_BOXBASE @ T_BOXBASE_LIVOX
    n_max = 3000
    pts = np.zeros((len(i), n_max, 3), dtype=np.float32)
    inten = np.zeros((len(i), n_max), dtype=np.float32)
    valid = np.zeros((len(i), 1), dtype=np.int32)
    for j in range(len(i)):
        pb_ = scan_points(2500)
        pl = (pb_ - T_base_livox[:3, 3]) @ T_base_livox[:3, :3]  # base → livox
        pts[j, :len(pl)] = pl
        inten[j, :len(pl)] = RNG.uniform(0, 150, len(pl))
        valid[j, 0] = len(pl)
    livox_attrs = dict(frame_id="livox_lidar", topic="livox_points_undistorted", transform=tf["livox_lidar"],
                       description="10Hz - Livox Mid360 non-repetitive LiDAR (mounted up-side-down)- undistorted using Leg Intgerial Odometry")
    write_topic(root, "livox_points_undistorted", dict(timestamp=truth["t"][i], points=pts, intensity=inten, valid=valid,
                                                       line=np.zeros((len(i), n_max), dtype=np.uint8),
                                                       tag=np.zeros((len(i), n_max), dtype=np.uint8),
                                                       sequence_id=np.arange(len(i))), livox_attrs)
    # IMU（ADIS は m/s²、Livox 内蔵は g 単位）
    i = pick(truth, 200)
    for name, T_imu, scale, desc in [("adis_imu", T_BASE_BOXBASE @ T_BOXBASE_ADIS, 1.0, "Analog Devices ADIS16475-2 200Hz"),
                                     ("livox_imu", T_base_livox, 1.0 / G, "TDK ICM40609 200Hz (integrated in Livox Mid360)")]:
        g_imu = np.einsum("nji,j->ni", np.einsum("nij,jk->nik", truth["R"][i], T_imu[:3, :3]), [0, 0, G])
        write_topic(root, name, dict(timestamp=truth["t"][i], lin_acc=(g_imu + RNG.normal(0, 0.02, g_imu.shape)) * scale,
                                     ang_vel=RNG.normal(0, 0.001, g_imu.shape), orien=np.tile([0, 0, 0, 1.0], (len(i), 1)),
                                     lin_acc_cov=np.zeros((len(i), 9)), ang_vel_cov=np.zeros((len(i), 9)),
                                     orien_cov=np.zeros((len(i), 9)), sequence_id=np.arange(len(i))),
                    dict(frame_id=name, topic=name, description=desc, transform=tf["adis16475_imu"]))

    # Hugging Face と同じ形にまとめる: data/<topic>.tar、data/.zgroup、metadata/*.yaml
    dst = out_root / folder
    (dst / "data").mkdir(parents=True, exist_ok=True)
    (dst / "metadata").mkdir(parents=True, exist_ok=True)
    shutil.copy2(work / "data" / ".zgroup", dst / "data" / ".zgroup")
    for topic_dir in sorted((work / "data").iterdir()):
        if topic_dir.is_dir():
            with tarfile.open(dst / "data" / f"{topic_dir.name}.tar", "w") as tar:
                tar.add(topic_dir, arcname=topic_dir.name)
            (dst / "metadata" / f"{topic_dir.name}.yaml").write_text(f"topic: {topic_dir.name}\n")
    shutil.rmtree(work)


def main():
    from pyproj import Transformer
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    fwd = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    inv = Transformer.from_crs("EPSG:32632", "EPSG:4326", always_xy=True)
    eth = fwd.transform(8.5476, 47.3763)
    sbb = fwd.transform(8.6100, 47.4050)
    loop = [(0, 0), (40, 0), (40, 20), (0, 20), (0, 0)]
    make_mission(out, "2024-10-01-11-29-55", loop, eth, still=3.0, t0=1727774995.0, utm_inv=inv.transform,
                 gnss_gap=(40.0, 3.0))
    make_mission(out, "2024-10-01-12-00-49", [(40, 20), (40, 0), (0, 0), (0, -30)], eth, still=1.0,
                 t0=1727776849.0, utm_inv=inv.transform)
    make_mission(out, "2024-12-03-13-15-38", [(0, 0), (60, 0)], sbb, still=2.0, t0=1733231738.0, utm_inv=inv.transform)
    print(f"fake missions: {out}")


if __name__ == "__main__":
    main()
