#!/usr/bin/env python3
"""GrandTour のミッション（Hugging Face の Zarr）から、推定ノード用の ROS 2 bag（MCAP）を作る（検証計画 3.2 節）。

  python3 grandtour_to_bag.py $GLL_DATA/grandtour/<フォルダ名> $GLL_DATA/grandtour/bags/spx3.mcap \\
      [--init-error 0,0] [--start 0] [--end 0] [--odom-scale 1.0] [--odom-noise 0.0] [--drop-lidar 60:10 ...]

出力:
  <out>.mcap                  bag（ros2 bag play <out>.mcap --clock で再生する）
  <out>_groundtruth.csv       真値（base_link、UTM）。評価に使う
  <out>_params.yaml           推定ノードのパラメータ（config/localizer.yaml に、このミッション用の値を入れたもの）

トピック（frame）:
  /sensing/imu              sensor_msgs/Imu（base_link。IMU の値を base の向きに回し、m/s² にそろえる）
  /sensing/odom             nav_msgs/Odometry（odom → base_link。脚のオドメトリ。twist は base の座標系）
  /sensing/lidar/points     sensor_msgs/PointCloud2（livox_lidar。x, y, z, intensity。点ごとの時刻は無い）
  /tf_static                base_link → livox_lidar
  /initialpose              geometry_msgs/PoseWithCovarianceStamped（map = UTM。真値に --init-error の誤差を足したもの）
  /groundtruth/pose         geometry_msgs/PoseStamped（map = UTM。確認用。推定ノードは購読しない）

データの約束事（オドメトリの姿勢と速度、静的 TF の解釈）は、事前確認（inspect_grandtour.py）で確かめたものに合わせてある。
必要なパッケージ: numpy、zarr（3.x）、pyproj、PyYAML。ROS は要らない（MCAP は tools/tile_demo/make_bag.py の書き出しを使う）。
"""
import argparse
import csv
import heapq
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "tools" / "tile_demo"))
import analysis as A  # noqa: E402
import gt_zarr as Z  # noqa: E402
import make_bag as MB  # noqa: E402
import missions as M  # noqa: E402

G = 9.80665

# ------------------------------------------------------------------ メッセージの定義（make_bag.py の一覧に足す）
MB.MSGDEFS.update({
    "geometry_msgs/msg/PoseWithCovariance": "Pose pose\nfloat64[36] covariance",
    "geometry_msgs/msg/PoseWithCovarianceStamped": "std_msgs/Header header\nPoseWithCovariance pose",
    "geometry_msgs/msg/Twist": "Vector3 linear\nVector3 angular",
    "geometry_msgs/msg/TwistWithCovariance": "Twist twist\nfloat64[36] covariance",
    "nav_msgs/msg/Odometry": "std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\n"
                             "geometry_msgs/TwistWithCovariance twist",
    "sensor_msgs/msg/Imu": "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
                           "geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
                           "geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance",
})
_POSE = ["geometry_msgs/msg/Pose", "geometry_msgs/msg/Point", "geometry_msgs/msg/Quaternion"]
MB.DEPS.update({
    "sensor_msgs/msg/Imu": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/Quaternion",
                            "geometry_msgs/msg/Vector3"],
    "nav_msgs/msg/Odometry": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/PoseWithCovariance"]
                             + _POSE + ["geometry_msgs/msg/TwistWithCovariance", "geometry_msgs/msg/Twist",
                                        "geometry_msgs/msg/Vector3"],
    "geometry_msgs/msg/PoseWithCovarianceStamped": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time",
                                                    "geometry_msgs/msg/PoseWithCovariance"] + _POSE,
    "geometry_msgs/msg/PoseStamped": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time"] + _POSE,
})


def ns(t):
    return int(round(t * 1e9))


def _pose(c, p, q):
    for v in (*p, *q):
        c.f64(float(v))


def ser_imu(t, frame, gyro, acc):
    c = MB.Cdr()
    c.header(ns(t), frame)
    _pose(c, [], [0.0, 0.0, 0.0, 1.0])
    c.f64(-1.0)  # orientation_covariance[0] = -1: 姿勢は無い
    for _ in range(8):
        c.f64(0.0)
    for v in gyro:
        c.f64(float(v))
    for _ in range(9):
        c.f64(0.0)
    for v in acc:
        c.f64(float(v))
    for _ in range(9):
        c.f64(0.0)
    return c.bytes()


def ser_odom(t, frame, child, p, q, v, w):
    c = MB.Cdr()
    c.header(ns(t), frame)
    c.string(child)
    _pose(c, p, q)
    for _ in range(36):
        c.f64(0.0)
    for x in (*v, *w):
        c.f64(float(x))
    for _ in range(36):
        c.f64(0.0)
    return c.bytes()


def ser_pose_cov(t, frame, p, q, cov):
    c = MB.Cdr()
    c.header(ns(t), frame)
    _pose(c, p, q)
    for x in np.asarray(cov, dtype=float).reshape(36):
        c.f64(float(x))
    return c.bytes()


def ser_pose(t, frame, p, q):
    c = MB.Cdr()
    c.header(ns(t), frame)
    _pose(c, p, q)
    return c.bytes()


def ser_tf(t, parent, child, T):
    c = MB.Cdr()
    c.u32(1)
    c.header(ns(t), parent)
    c.string(child)
    _pose(c, T[:3, 3], A.rot_to_quat(T[:3, :3]))
    return c.bytes()


# ------------------------------------------------------------------ 変換
def parse_drop(items):
    out = []
    for s in items:
        a, b = s.split(":")
        out.append((float(a), float(b)))
    return out


def estimate_base_height(root, topic, T_base_lidar, idx):
    """スキャンの点を base に移し、周り（半径 1〜6 m）の低い方の点を地面とみなして、地面から base までの高さを求める。"""
    hs = []
    for i in idx:
        pts = Z.read_scan(root, topic, int(i))["points"]
        pb = pts @ T_base_lidar[:3, :3].T + T_base_lidar[:3, 3]
        r = np.hypot(pb[:, 0], pb[:, 1])
        ring = (r > 1.0) & (r < 6.0)
        if ring.sum() > 50:
            hs.append(-A.pct(pb[ring, 2], 5))
    return float(np.median(hs)) if hs else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission_dir", type=Path)
    ap.add_argument("out", type=Path, help="出力の .mcap")
    ap.add_argument("--imu", default="adis_imu", help="IMU のトピック（adis_imu / livox_imu / anymal_imu など）")
    ap.add_argument("--points", choices=["undistorted", "raw"], default="undistorted")
    ap.add_argument("--start", type=float, default=0.0, help="ミッションの最初からの開始時刻 [s]")
    ap.add_argument("--end", type=float, default=0.0, help="ミッションの最初からの終了時刻 [s]（0 なら最後まで）")
    ap.add_argument("--odom-rate", type=float, default=50.0, help="ODOM の周期 [Hz]（間引く）")
    ap.add_argument("--odom-scale", type=float, default=1.0, help="ODOM の速度に掛ける係数（劣化させる。V-G3）")
    ap.add_argument("--odom-noise", type=float, default=0.0, help="ODOM の速度に足す白色雑音の標準偏差 [m/s]（V-G3）")
    ap.add_argument("--drop-lidar", nargs="*", default=[], metavar="START:DURATION",
                    help="LiDAR を止める区間（bag の最初からの秒。V-G4）")
    ap.add_argument("--init-error", default="0,0", help="初期姿勢の誤差 '位置 [m],yaw [deg]'（V-G2）")
    ap.add_argument("--init-delay", type=float, default=0.5, help="静止初期化の後、初期姿勢を出すまでの時間 [s]")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--utm-zone", type=int, default=32)
    ap.add_argument("--map-config", default="", help="params の map.config_path に入れる maps.yaml")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    mdir = args.mission_dir.expanduser()
    code = M.code_of(mdir.name)
    root = Z.open_mission(mdir)
    have = Z.topics(root)
    ptopic = "livox_points_undistorted" if args.points == "undistorted" else "livox_points"
    for t_ in (args.imu, "anymal_state_odometry", ptopic, "tf", "cpt7_ie_tc_odometry", "navsatfix_cpt7_ie_tc"):
        if t_ not in have:
            sys.exit(f"{code}: トピック {t_} が無い（download.py --preset lidar で落とす）")
    tf = Z.tf_table(root)
    for t_ in (args.imu, ptopic):  # TF に無いフレームは、トピックの transform 属性（同じ形・同じ解釈）で補う
        fr, tr = root[t_].attrs.get("frame_id", t_), root[t_].attrs.get("transform")
        if fr not in tf and tr:
            tf[fr] = dict(tr)

    # ---- 真値
    gt = Z.ground_truth(root, args.utm_zone)
    fit = gt["fit"]
    print(f"{code}: ENU → UTM: 回転 {fit['theta_deg']:.3f}°、縮尺 {fit['s']:.6f}、当てはめの残差 {fit['rms'] * 100:.1f} cm")
    if fit["rms"] > 0.05:
        print("  注意: 残差が大きい（navsatfix と cpt7_ie_tc_odometry の基準点が違う可能性）")

    # ---- 時刻の範囲
    imu = Z.read_imu(root, args.imu)
    leg = Z.read_odometry(root, "anymal_state_odometry")
    t_mission0 = min(imu["t"][0], leg["t"][0])
    t0 = t_mission0 + args.start
    t1 = (t_mission0 + args.end) if args.end > 0 else min(imu["t"][-1], leg["t"][-1])
    t1 = min(t1, gt["t"][-1])

    # ---- 静止初期化の時間（bag の最初から止まっている時間）
    lt = leg["t_twist"]
    sel = (lt >= t0) & (lt <= t1)
    still = A.stationary_start(lt[sel], np.linalg.norm(leg["v"][sel, :2], axis=1), leg["w"][sel, 2])
    static_init = float(np.clip(still - 0.3, 0.2, 3.0))
    t_init = t0 + static_init + args.init_delay
    print(f"  最初の静止 {still:.1f} s → attitude.static_init_time = {static_init:.1f} s、初期姿勢は {t_init - t0:.1f} s に出す")

    # ---- IMU（base の向きへ、m/s² へ）
    R_base_imu = Z.static_transform(tf, "base", root[args.imu].attrs.get("frame_id", args.imu))[:3, :3]
    first = imu["t"] < imu["t"][0] + 1.0
    acc_norm = float(np.median(np.linalg.norm(imu["acc"][first], axis=1)))
    acc_scale = G if acc_norm < 3.0 else 1.0
    print(f"  IMU {args.imu}: 最初の 1 s の加速度の大きさ {acc_norm:.3f} → 係数 {acc_scale}")
    acc_b = imu["acc"] @ R_base_imu.T * acc_scale
    gyro_b = imu["gyro"] @ R_base_imu.T

    # ---- LiDAR の外部パラメータ
    lidar_frame = root[ptopic].attrs.get("frame_id", "livox_lidar")
    T_base_lidar = Z.static_transform(tf, "base", lidar_frame)
    rpy = np.rad2deg(A.rpy_of(T_base_lidar[:3, :3]))
    ts_scan = np.asarray(root[ptopic]["timestamp"][:], dtype=float)
    scan_idx = np.nonzero((ts_scan >= t0) & (ts_scan <= t1))[0]
    still_scans = scan_idx[ts_scan[scan_idx] < t0 + max(still, 0.5)][:5]
    base_h = estimate_base_height(root, ptopic, T_base_lidar, still_scans if len(still_scans) else scan_idx[:5])
    print(f"  LiDAR {lidar_frame}: 位置 {np.round(T_base_lidar[:3, 3], 4)} m、rpy {np.round(rpy, 3)} deg、"
          f"地面から base まで {base_h:.3f} m")

    drops = [(t0 + a, t0 + a + b) for a, b in parse_drop(args.drop_lidar)]

    # ---- 書き出し（時刻順に混ぜる）
    out = args.out.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    w = MB.McapWriter(str(out))
    qos = MB.qos_yaml(False, 100)
    ch = {}
    for topic, typ, q in [("/sensing/imu", "sensor_msgs/msg/Imu", qos), ("/sensing/odom", "nav_msgs/msg/Odometry", qos),
                          ("/sensing/lidar/points", "sensor_msgs/msg/PointCloud2", MB.qos_yaml(False, 10)),
                          ("/tf_static", "tf2_msgs/msg/TFMessage", MB.qos_yaml(True, 1)),
                          ("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", MB.qos_yaml(False, 1)),
                          ("/groundtruth/pose", "geometry_msgs/msg/PoseStamped", qos)]:
        ch[topic] = w.channel(w.schema(typ), topic, q)

    def gen_imu():
        for i in np.nonzero((imu["t"] >= t0) & (imu["t"] <= t1))[0]:
            yield ns(imu["t"][i]), ch["/sensing/imu"], ser_imu(imu["t"][i], "base_link", gyro_b[i], acc_b[i])

    def gen_odom():
        idx = A.subsample(lt, 1.0 / args.odom_rate)
        idx = idx[(lt[idx] >= t0) & (lt[idx] <= t1)]
        for i in idx:
            v = leg["v"][i].copy()
            v[:2] *= args.odom_scale
            wz = leg["w"][i].copy()
            if args.odom_noise > 0:
                v[:2] += rng.normal(0, args.odom_noise, 2)
                wz[2] += rng.normal(0, args.odom_noise * 0.2)
            j = int(np.clip(np.searchsorted(leg["t"], lt[i]), 0, len(leg["t"]) - 1))
            yield ns(lt[i]), ch["/sensing/odom"], ser_odom(lt[i], "odom", "base_link", leg["p"][j], leg["q"][j], v, wz)

    def gen_points():
        for i in scan_idx:
            t = ts_scan[i]
            if any(a <= t < b for a, b in drops):
                continue
            s = Z.read_scan(root, ptopic, int(i))
            inten = s.get("intensity", np.zeros(len(s["points"])))
            yield ns(t), ch["/sensing/lidar/points"], MB.ser_cloud(ns(t), lidar_frame, s["points"].astype(np.float32),
                                                                   inten.astype(np.float32))

    def gen_static():
        yield ns(t0), ch["/tf_static"], ser_tf(t0, "base_link", lidar_frame, T_base_lidar)

    # 初期姿勢（真値 + 誤差）
    e_xy, e_yaw = (float(x) for x in args.init_error.split(","))
    T_init = Z.interp_pose(gt["t"], gt["T_utm"], np.array([t_init]))[0]
    ang = rng.uniform(-math.pi, math.pi)
    yaw0 = A.yaw_of(T_init[:3, :3]) + math.radians(e_yaw) * (1 if rng.uniform() < 0.5 else -1)
    p_init = T_init[:3, 3] + np.array([e_xy * math.cos(ang), e_xy * math.sin(ang), 0.0])
    s_xy, s_yaw = max(e_xy, 0.5), math.radians(max(e_yaw, 5.0))
    cov = np.zeros((6, 6))
    cov[0, 0] = cov[1, 1] = s_xy ** 2
    cov[2, 2] = 0.25
    cov[3, 3] = cov[4, 4] = math.radians(2.0) ** 2
    cov[5, 5] = s_yaw ** 2

    def gen_init():
        yield ns(t_init), ch["/initialpose"], ser_pose_cov(t_init, "map", p_init, A.rot_to_quat(A.rot_z(yaw0)), cov)

    gsel = np.nonzero((gt["t"] >= t0) & (gt["t"] <= t1))[0]
    gsel = gsel[A.subsample(gt["t"][gsel], 0.05)]

    def gen_gt():
        for i in gsel:
            T = gt["T_utm"][i]
            yield ns(gt["t"][i]), ch["/groundtruth/pose"], ser_pose(gt["t"][i], "map", T[:3, 3], A.rot_to_quat(T[:3, :3]))

    n = 0
    for t, cid, data in heapq.merge(gen_static(), gen_init(), gen_imu(), gen_odom(), gen_points(), gen_gt(),
                                    key=lambda x: x[0]):
        w.message(cid, t, data)
        n += 1
    w.close()
    print(f"  {out}: {n} メッセージ、{t1 - t0:.1f} s（スキャン {len(scan_idx)}、止めた区間 {len(drops)}）")

    # ---- 真値の CSV（base_link、UTM）
    stem = out.with_suffix("")
    gcsv = Path(str(stem) + "_groundtruth.csv")
    sel = (gt["t"] >= t0) & (gt["t"] <= t1)
    with open(gcsv, "w", newline="") as fp:
        wr = csv.writer(fp)
        wr.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw", "sigma_h"])
        for i in np.nonzero(sel)[0]:
            T = gt["T_utm"][i]
            r, p, y = A.rpy_of(T[:3, :3])
            sh = gt["sigma_h"][i] if "sigma_h" in gt else float("nan")
            wr.writerow([f"{gt['t'][i]:.6f}", f"{T[0, 3]:.4f}", f"{T[1, 3]:.4f}", f"{T[2, 3]:.4f}",
                         f"{r:.6f}", f"{p:.6f}", f"{y:.6f}", f"{sh:.4f}"])
    print(f"  {gcsv}")

    # ---- パラメータ（config/localizer.yaml にこのミッション用の値を入れる）
    base = yaml.safe_load((REPO / "ros2" / "gll_ros2" / "config" / "localizer.yaml").read_text())
    prm = base["gll_localizer"]["ros__parameters"]
    prm["gnss"]["utm_zone"] = args.utm_zone
    prm["gnss"]["utm_north"] = True
    prm["imu"]["rotation_rpy_deg"] = [0.0, 0.0, 0.0]
    prm["imu"]["acc_scale"] = 1.0
    prm["attitude"]["static_init_time"] = round(static_init, 2)
    prm["lidar"]["extrinsic_xyz"] = [round(float(x), 5) for x in T_base_lidar[:3, 3]]
    prm["lidar"]["extrinsic_rpy_deg"] = [round(float(x), 4) for x in rpy]
    prm["lidar"]["time_field"] = ""
    prm["lidar"]["max_range"] = 40.0
    if np.isfinite(base_h):
        prm["lidar"]["base_link_height"] = round(base_h, 3)
    prm["map"]["config_path"] = args.map_config
    prm["debug_csv_path"] = str(Path(str(stem) + "_output.csv"))
    pfile = Path(str(stem) + "_params.yaml")
    header = (f"# {code}（{mdir.name}）用。tools/grandtour/grandtour_to_bag.py が config/localizer.yaml から作った。\n"
              f"# lidar.crop_box_* はまだ既定値（ロボット自身の点の範囲は inspect_grandtour.py のレポート 4 節を見て決める）。\n")
    pfile.write_text(header + yaml.safe_dump(base, sort_keys=False, allow_unicode=True))
    print(f"  {pfile}")


if __name__ == "__main__":
    main()
