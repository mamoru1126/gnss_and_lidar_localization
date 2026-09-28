#!/usr/bin/env python3
"""AWSIM で録った bag（ROS 2 Humble、MCAP）から、推定ノード用の ROS 2 bag（MCAP）を作る（検証計画 3 章）。

  python3 awsim_to_bag.py <録った bag（ディレクトリか .mcap）> <出力.mcap> \\
      [--lidar-extrinsic 0.9,0,2.0,0,0,0] [--gnss-off-time 120:30 ...] [--gnss-off-box x0,y0,x1,y1 ...]

入力（AWSIM の既定のトピック。--*-topic で変えられる）:
  /awsim/ground_truth/vehicle/pose        geometry_msgs/PoseStamped（真値。地図座標 = MGRS の区画の中の座標）
  /sensing/imu/tamagawa/imu_raw           sensor_msgs/Imu
  /vehicle/status/velocity_status         autoware_auto_vehicle_msgs/VelocityReport
  /sensing/lidar/top/pointcloud_raw       sensor_msgs/PointCloud2
出力:
  <out>.mcap                  /sensing/imu（base_link）、/sensing/odom、/sensing/gnss/fix、/sensing/lidar/points、
                              /tf_static、/groundtruth/pose（と、--initial-pose なら /initialpose）
  <out>_groundtruth.csv       真値（base_link、UTM）
  <out>_params.yaml           推定ノードのパラメータ（config/localizer.yaml にこの bag 用の値を入れたもの）
  <out>_maps.yaml             --tiles を指定したとき。地図グループ 1 つと、MGRS の原点から決めたアンカー（縮尺なし）

- GNSS は、AWSIM の出力（1 Hz の Pose）の代わりに、真値から F9P 相当の NavSatFix を作る（--gnss-rate、--gnss-sigma）。
  --gnss-off-time / --gnss-off-box の区間・範囲では RTK-FIX を外す（status = -1）。GNSS 区間と地図区間の切り替わりの検証用。
- IMU は、真値の加速度・角速度との当てはめで取り付けの向きを求め、base_link の向きに回して出す。
- 点群は、中身をそのまま出す（frame も元のまま）。base_link → 点群の frame は --lidar-extrinsic で与え、/tf_static と
  パラメータに入れる。値は check_lidar_extrinsic.py で地図と照らして確かめる。
- 地図座標 → UTM は、MGRS の区画の原点（--mgrs-origin。西新宿の 54SUE なら 300000,3900000）を足すだけ（縮尺なし）。
必要なパッケージ: numpy、pyproj、PyYAML（zstd で圧縮された bag なら zstandard）。ROS は要らない。
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
import make_bag as MB  # noqa: E402
import mcap_io as IO  # noqa: E402
import rigid as A  # noqa: E402

G = 9.80665

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
    "sensor_msgs/msg/NavSatStatus": "int8 STATUS_NO_FIX = -1\nint8 STATUS_FIX = 0\nint8 STATUS_SBAS_FIX = 1\n"
                                    "int8 STATUS_GBAS_FIX = 2\nuint16 SERVICE_GPS = 1\nuint16 SERVICE_GLONASS = 2\n"
                                    "uint16 SERVICE_COMPASS = 4\nuint16 SERVICE_GALILEO = 8\nint8 status\nuint16 service",
    "sensor_msgs/msg/NavSatFix": "std_msgs/Header header\nNavSatStatus status\nfloat64 latitude\nfloat64 longitude\n"
                                 "float64 altitude\nfloat64[9] position_covariance\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\n"
                                 "uint8 COVARIANCE_TYPE_APPROXIMATED = 1\nuint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\n"
                                 "uint8 COVARIANCE_TYPE_KNOWN = 3\nuint8 position_covariance_type",
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
    "sensor_msgs/msg/NavSatFix": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "sensor_msgs/msg/NavSatStatus"],
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
    c.f64(-1.0)
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


def ser_odom(t, v, w):
    c = MB.Cdr()
    c.header(ns(t), "odom")
    c.string("base_link")
    _pose(c, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    for _ in range(36):
        c.f64(0.0)
    for x in (*v, *w):
        c.f64(float(x))
    for _ in range(36):
        c.f64(0.0)
    return c.bytes()


def ser_navsat(t, status, lat, lon, alt, var_h, var_v):
    c = MB.Cdr()
    c.header(ns(t), "gnss_link")
    c.b += int(status).to_bytes(1, "little", signed=True)  # NavSatStatus.status（int8）
    c._align(2)
    c.b += (1).to_bytes(2, "little")                         # service（uint16）: GPS
    for v in (lat, lon, alt):
        c.f64(float(v))
    for v in (var_h, 0, 0, 0, var_h, 0, 0, 0, var_v):
        c.f64(float(v))
    c.u8(2)                                                  # COVARIANCE_TYPE_DIAGONAL_KNOWN
    return c.bytes()


def ser_pose(t, frame, p, q):
    c = MB.Cdr()
    c.header(ns(t), frame)
    _pose(c, p, q)
    return c.bytes()


def ser_pose_cov(t, frame, p, q, cov):
    c = MB.Cdr()
    c.header(ns(t), frame)
    _pose(c, p, q)
    for x in np.asarray(cov, dtype=float).reshape(36):
        c.f64(float(x))
    return c.bytes()


def ser_tf(t, parent, child, T):
    c = MB.Cdr()
    c.u32(1)
    c.header(ns(t), parent)
    c.string(child)
    _pose(c, T[:3, 3], A.rot_to_quat(T[:3, :3]))
    return c.bytes()


# ------------------------------------------------------------------ IMU の取り付けの向き
def estimate_imu_rotation(gt_t, gt_p, gt_R, t_imu, gyro, acc):
    """真値から求めた角速度・比力（base 座標系）と IMU の値を当てはめ、R_base_imu を求める。

    比力 f = R^T (a_world + g)、角速度 ω = log(R(t-h)^T R(t+h)) / 2h。どちらも R_base_imu · (IMU の値) に一致する。
    IMU の加速度に重力が入っていなければ、f から g を除いて当てはめる。
    """
    h = 0.05
    ok = (t_imu > gt_t[0] + 0.2) & (t_imu < gt_t[-1] - 0.2)
    ts = t_imu[ok]
    p0, p1, p2 = (A.interp(gt_t, gt_p, ts + d) for d in (-2 * h, 0.0, 2 * h))
    a_world = (p2 - 2 * p1 + p0) / (2 * h) ** 2
    Rm = A.quat_to_rot(A.quat_interp(gt_t, np.stack([A.rot_to_quat(r) for r in gt_R]), ts - h))
    Rp = A.quat_to_rot(A.quat_interp(gt_t, np.stack([A.rot_to_quat(r) for r in gt_R]), ts + h))
    R0 = A.quat_to_rot(A.quat_interp(gt_t, np.stack([A.rot_to_quat(r) for r in gt_R]), ts))
    w_base = A.rot_log(np.einsum("nji,njk->nik", Rm, Rp)) / (2 * h)
    still = np.linalg.norm(a_world, axis=1) < 0.05
    has_g = float(np.median(np.linalg.norm(acc[ok], axis=1))) > 3.0
    f_world = a_world + (np.array([0.0, 0.0, G]) if has_g else 0.0)
    f_base = np.einsum("nji,nj->ni", R0, f_world)
    src = np.vstack([acc[ok], 5.0 * gyro[ok]])
    dst = np.vstack([f_base, 5.0 * w_base])
    good = np.all(np.isfinite(dst), axis=1)
    R, _ = A.kabsch_origin(src[good], dst[good])
    res_acc = np.linalg.norm(acc[ok] @ R.T - f_base, axis=1)
    return R, dict(has_gravity=has_g, acc_rms=float(np.sqrt(np.nanmean(res_acc ** 2))), still=int(still.sum()))


def in_boxes(xy, boxes):
    m = np.zeros(len(xy), dtype=bool)
    for x0, y0, x1, y1 in boxes:
        m |= (xy[:, 0] >= min(x0, x1)) & (xy[:, 0] <= max(x0, x1)) & (xy[:, 1] >= min(y0, y1)) & (xy[:, 1] <= max(y0, y1))
    return m


def sorted_window(it, window=64):
    """だいたい時刻順に並んだ (t, ...) の列を、window 個の範囲で並べ直して時刻順にする。"""
    heap = []
    for k, x in enumerate(it):
        heapq.heappush(heap, (x[0], k, x))
        if len(heap) > window:
            yield heapq.heappop(heap)[2]
    while heap:
        yield heapq.heappop(heap)[2]


def floats(s, n):
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expects {n} comma-separated numbers: {s}")
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--gt-topic", default="/awsim/ground_truth/vehicle/pose")
    ap.add_argument("--imu-topic", default="/sensing/imu/tamagawa/imu_raw")
    ap.add_argument("--vel-topic", default="/vehicle/status/velocity_status")
    ap.add_argument("--points-topic", default="/sensing/lidar/top/pointcloud_raw")
    ap.add_argument("--lidar-extrinsic", default="0.9,0,2.0,0,0,0",
                    help="base_link → 点群の frame 'x,y,z,roll,pitch,yaw'（m、deg）。check_lidar_extrinsic.py で確かめる")
    ap.add_argument("--mgrs-origin", default="300000,3900000", help="地図座標の原点の UTM 'E,N'（54SUE: 300000,3900000）")
    ap.add_argument("--utm-zone", type=int, default=54)
    ap.add_argument("--alt-offset", type=float, default=0.0, help="地図の z に足して楕円体高にする値 [m]")
    ap.add_argument("--gnss-rate", type=float, default=10.0)
    ap.add_argument("--gnss-sigma", type=float, default=0.02, help="RTK-FIX の水平の標準偏差 [m]（高さは 2 倍）")
    ap.add_argument("--gnss-lever", default="0,0,1.5", help="base_link から見たアンテナの位置 'x,y,z' [m]")
    ap.add_argument("--gnss-off-time", nargs="*", default=[], metavar="START:DURATION",
                    help="RTK-FIX を外す時間（bag の最初からの秒）")
    ap.add_argument("--gnss-off-box", nargs="*", default=[], metavar="X0,Y0,X1,Y1",
                    help="RTK-FIX を外す範囲（地図座標の矩形）")
    ap.add_argument("--no-gnss", action="store_true", help="GNSS を出さない（地図だけでの検証）")
    ap.add_argument("--initial-pose", default="", metavar="ERR_XY,ERR_YAW_DEG",
                    help="/initialpose を出す（真値にこの誤差を足す。地図だけでの初期化の検証）")
    ap.add_argument("--odom-scale", type=float, default=1.0)
    ap.add_argument("--odom-noise", type=float, default=0.0)
    ap.add_argument("--drop-lidar", nargs="*", default=[], metavar="START:DURATION")
    ap.add_argument("--static-init", type=float, default=0.0, help="attitude.static_init_time（0 なら最初の停止から決める）")
    ap.add_argument("--map-config", default="", help="map.config_path に入れる maps.yaml（--tiles と一緒には使わない）")
    ap.add_argument("--tiles", type=Path, default=None,
                    help="地図のタイルのフォルダ（tiled_pcd_map_tiler の -o）。<out>_maps.yaml を作って map.config_path に入れる")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    ex = floats(args.lidar_extrinsic, 6)
    E0, N0 = floats(args.mgrs_origin, 2)
    lever = np.array(floats(args.gnss_lever, 3))

    # ---- 1 回目: 点群以外を読む
    small = {args.gt_topic: [], args.imu_topic: [], args.vel_topic: []}
    frames, pcount, ptopic_seen = {}, 0, False
    for topic, typ, t_log, data in IO.read_messages(args.bag, set(small) | {args.points_topic}):
        if topic == args.points_topic:
            if not ptopic_seen:
                _, pframe, names, npts = IO.cloud_header(data)
                frames["points"] = (pframe, names, npts)
                ptopic_seen = True
            pcount += 1
            continue
        if topic == args.gt_topic:
            t, _f, p, q = IO.pose_stamped(data)
            small[topic].append((t, *p, *q))
        elif topic == args.imu_topic:
            t, f, gyro, acc = IO.imu(data)
            frames["imu"] = f
            small[topic].append((t, *gyro, *acc))
        elif topic == args.vel_topic:
            t, _f, vx, vy, wz = IO.velocity_report(data)
            small[topic].append((t, vx, vy, wz))
    for k, v in small.items():
        if not v:
            sys.exit(f"{k} のメッセージが無い（トピック名は --*-topic で変えられる）")
    if not pcount:
        sys.exit(f"{args.points_topic} のメッセージが無い")
    gt = np.array(sorted(small[args.gt_topic]))
    imu = np.array(sorted(small[args.imu_topic]))
    vel = np.array(sorted(small[args.vel_topic]))
    gt_t, gt_p = gt[:, 0], gt[:, 1:4]
    gt_R = A.quat_to_rot(gt[:, 4:8])
    t0 = max(gt_t[0], imu[0, 0], vel[0, 0])
    t1 = min(gt_t[-1], imu[-1, 0], vel[-1, 0])
    print(f"真値 {len(gt)}（{len(gt) / (gt_t[-1] - gt_t[0]):.0f} Hz）、IMU {len(imu)}、速度 {len(vel)}、点群 {pcount}"
          f"（frame {frames['points'][0]}、列 {frames['points'][1]}、{frames['points'][2]} 点）、{t1 - t0:.1f} s")

    # ---- 最初の停止（静止初期化）
    sp = np.hypot(vel[:, 1], vel[:, 2])
    moving = (sp > 0.05) | (np.abs(vel[:, 3]) > 0.02)
    sel = vel[:, 0] >= t0
    first_move = vel[sel][np.argmax(moving[sel]), 0] if moving[sel].any() else t1
    still = first_move - t0
    static_init = args.static_init or float(np.clip(still - 0.5, 0.2, 3.0))
    print(f"最初の停止 {still:.1f} s → attitude.static_init_time = {static_init:.1f} s")
    if still < 1.0:
        print("  注意: 最初にほとんど止まっていない。走り出す前に数秒止まるように録る（静止初期化のため）")

    # ---- IMU の取り付けの向き
    R_bi, info = estimate_imu_rotation(gt_t, gt_p, gt_R, imu[:, 0], imu[:, 1:4], imu[:, 4:7])
    rpy = np.rad2deg(A.rpy_of(R_bi))
    print(f"IMU（{frames['imu']}）→ base_link の向き: rpy {np.round(rpy, 2)} deg、当てはめの残差 {info['acc_rms']:.3f} m/s²、"
          f"重力 {'あり' if info['has_gravity'] else 'なし（足して出す）'}")
    gyro_b = imu[:, 1:4] @ R_bi.T
    acc_b = imu[:, 4:7] @ R_bi.T
    if not info["has_gravity"]:
        Rg = A.quat_to_rot(A.quat_interp(gt_t, gt[:, 4:8], imu[:, 0]))
        acc_b += np.einsum("nji,j->ni", Rg, [0.0, 0.0, G])

    # ---- LiDAR の外部パラメータ
    T_bl = A.se3(A.rpy_to_rot(*np.deg2rad(ex[3:])), ex[:3])
    pframe = frames["points"][0]

    # ---- 書き出し
    out = args.out.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    stem = out.with_suffix("")
    w = MB.McapWriter(str(out))
    q100 = MB.qos_yaml(False, 100)
    ch = {}
    for topic, typ, q in [("/sensing/imu", "sensor_msgs/msg/Imu", q100), ("/sensing/odom", "nav_msgs/msg/Odometry", q100),
                          ("/sensing/gnss/fix", "sensor_msgs/msg/NavSatFix", q100),
                          ("/sensing/lidar/points", "sensor_msgs/msg/PointCloud2", MB.qos_yaml(False, 10)),
                          ("/tf_static", "tf2_msgs/msg/TFMessage", MB.qos_yaml(True, 1)),
                          ("/groundtruth/pose", "geometry_msgs/msg/PoseStamped", q100),
                          ("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", MB.qos_yaml(False, 1))]:
        if topic == "/sensing/gnss/fix" and args.no_gnss:
            continue
        if topic == "/initialpose" and not args.initial_pose:
            continue
        ch[topic] = w.channel(w.schema(typ), topic, q)

    msgs = [(ns(t0), ch["/tf_static"], ser_tf(t0, "base_link", pframe, T_bl))]
    for r, g, a in zip(imu, gyro_b, acc_b):
        if t0 <= r[0] <= t1:
            msgs.append((ns(r[0]), ch["/sensing/imu"], ser_imu(r[0], "base_link", g, a)))
    for r in vel:
        if t0 <= r[0] <= t1:
            v = np.array([r[1], r[2], 0.0]) * args.odom_scale
            wz = r[3]
            if args.odom_noise > 0:
                v[:2] += rng.normal(0, args.odom_noise, 2)
                wz += rng.normal(0, args.odom_noise * 0.2)
            msgs.append((ns(r[0]), ch["/sensing/odom"], ser_odom(r[0], v, (0.0, 0.0, wz))))
    gsel = np.nonzero((gt_t >= t0) & (gt_t <= t1))[0]
    for i in gsel[A.subsample(gt_t[gsel], 0.05)]:
        msgs.append((ns(gt_t[i]), ch["/groundtruth/pose"],
                     ser_pose(gt_t[i], "map", gt_p[i] + [E0, N0, 0.0], gt[i, 4:8])))

    n_fix = n_off = 0
    if not args.no_gnss:
        from pyproj import Transformer
        inv = Transformer.from_crs(f"EPSG:{32600 + args.utm_zone}", "EPSG:4326", always_xy=True)
        tg = np.arange(t0, t1, 1.0 / args.gnss_rate)
        pg = A.interp(gt_t, gt_p, tg)
        Rg = A.quat_to_rot(A.quat_interp(gt_t, gt[:, 4:8], tg))
        ant = pg + np.einsum("nij,j->ni", Rg, lever)
        noise = rng.normal(0, args.gnss_sigma, (len(tg), 3)) * [1, 1, 2]
        off = in_boxes(ant[:, :2], [floats(b, 4) for b in args.gnss_off_box])
        for s in args.gnss_off_time:
            a, b = (float(x) for x in s.split(":"))
            off |= (tg >= t0 + a) & (tg < t0 + a + b)
        lon, lat = inv.transform(ant[:, 0] + noise[:, 0] + E0, ant[:, 1] + noise[:, 1] + N0)
        alt = ant[:, 2] + noise[:, 2] + args.alt_offset
        for k in range(len(tg)):
            st = -1 if off[k] else 2
            sig = args.gnss_sigma if st == 2 else 5.0
            msgs.append((ns(tg[k]), ch["/sensing/gnss/fix"], ser_navsat(tg[k], st, lat[k], lon[k], alt[k], sig ** 2, (2 * sig) ** 2)))
        n_off = int(off.sum())
        n_fix = len(tg) - n_off
        print(f"GNSS: {len(tg)} 件（RTK-FIX {n_fix}、外した {n_off}）、{args.gnss_rate:.0f} Hz、σ {args.gnss_sigma} m")

    if args.initial_pose:
        e_xy, e_yaw = floats(args.initial_pose, 2)
        ti = t0 + static_init + 0.5
        pi = A.interp(gt_t, gt_p, np.array([ti]))[0] + [E0, N0, 0.0]
        yaw = A.yaw_of(A.quat_to_rot(A.quat_interp(gt_t, gt[:, 4:8], np.array([ti])))[0]) + math.radians(e_yaw)
        ang = rng.uniform(-math.pi, math.pi)
        pi[:2] += e_xy * np.array([math.cos(ang), math.sin(ang)])
        cov = np.diag([max(e_xy, 0.5) ** 2, max(e_xy, 0.5) ** 2, 0.25, 0.0003, 0.0003, math.radians(max(e_yaw, 5.0)) ** 2])
        msgs.append((ns(ti), ch["/initialpose"], ser_pose_cov(ti, "map", pi, A.rot_to_quat(A.rot_z(yaw)), cov)))
    msgs.sort(key=lambda x: x[0])

    drops = [(t0 + float(a), t0 + float(a) + float(b)) for a, b in (s.split(":") for s in args.drop_lidar)]

    def points():
        for topic, _typ, _tl, data in IO.read_messages(args.bag, {args.points_topic}):
            t, _f, _n, _c = IO.cloud_header(data)
            if t0 <= t <= t1 and not any(a <= t < b for a, b in drops):
                yield ns(t), ch["/sensing/lidar/points"], data

    n = 0
    for tns, cid, data in heapq.merge(iter(msgs), sorted_window(points()), key=lambda x: x[0]):
        w.message(cid, tns, data)
        n += 1
    w.close()
    print(f"{out}: {n} メッセージ")

    # ---- 真値の CSV（base_link、UTM）
    gcsv = Path(str(stem) + "_groundtruth.csv")
    with open(gcsv, "w", newline="") as fp:
        wr = csv.writer(fp)
        wr.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw"])
        for i in gsel:
            r, p, y = A.rpy_of(gt_R[i])
            wr.writerow([f"{gt_t[i]:.6f}", f"{gt_p[i, 0] + E0:.4f}", f"{gt_p[i, 1] + N0:.4f}", f"{gt_p[i, 2]:.4f}",
                         f"{r:.6f}", f"{p:.6f}", f"{y:.6f}"])
    print(f"  {gcsv}")

    # ---- パラメータ
    base = yaml.safe_load((REPO / "ros2" / "gll_ros2" / "config" / "localizer.yaml").read_text())
    prm = base["gll_localizer"]["ros__parameters"]
    prm["gnss"]["utm_zone"] = args.utm_zone
    prm["gnss"]["utm_north"] = True
    prm["gnss"]["lever_arm"] = [float(x) for x in lever]
    prm["imu"]["rotation_rpy_deg"] = [0.0, 0.0, 0.0]
    prm["attitude"]["static_init_time"] = round(static_init, 2)
    prm["lidar"]["extrinsic_xyz"] = [float(x) for x in ex[:3]]
    prm["lidar"]["extrinsic_rpy_deg"] = [float(x) for x in ex[3:]]
    prm["lidar"]["base_link_height"] = 0.0
    prm["map"]["config_path"] = args.map_config
    if args.tiles is not None:
        # 地図座標 = UTM − MGRS の原点（回転なし・縮尺なし）。アンカー点は走り始めの位置にする
        tidx = (args.tiles / "tile_index.yaml").resolve()
        if not tidx.exists():
            sys.exit(f"{tidx} が無い")
        mp = gt_p[gsel[0]]
        mfile = Path(str(stem) + "_maps.yaml")
        mfile.write_text(
            "# tools/awsim/awsim_to_bag.py が作った。地図座標（MGRS の区画の中の座標）= UTM − MGRS の原点\n"
            f"utm: {{zone: {args.utm_zone}, hemisphere: north}}\n"
            "map_groups:\n"
            "  - id: awsim\n"
            f"    tile_index: {tidx}\n"
            f"    anchor: {{map_point: [{mp[0]:.3f}, {mp[1]:.3f}, {mp[2]:.3f}], easting: {mp[0] + E0:.3f}, "
            f"northing: {mp[1] + N0:.3f}, ellipsoid_height: {mp[2] + args.alt_offset:.3f}, grid_heading_deg: 90.0, "
            "use_scale_factor: false, stddev_xy: 0.01, stddev_yaw_deg: 0.05}\n")
        prm["map"]["config_path"] = str(mfile.resolve())
        print(f"  {mfile}")
    prm["debug_csv_path"] = str(Path(str(stem) + "_output.csv"))
    pfile = Path(str(stem) + "_params.yaml")
    pfile.write_text(f"# {args.bag} から tools/awsim/awsim_to_bag.py が作った（config/localizer.yaml に値を入れたもの）\n"
                     + yaml.safe_dump(base, sort_keys=False, allow_unicode=True))
    print(f"  {pfile}")


if __name__ == "__main__":
    main()
