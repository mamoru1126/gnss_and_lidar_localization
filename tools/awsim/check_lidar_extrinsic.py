#!/usr/bin/env python3
"""LiDAR の取り付け位置（base_link → 点群の frame）を、地図と照らして確かめ、合うように直す（検証計画 3.3 節）。

  python3 check_lidar_extrinsic.py <録った bag> <地図のタイルのフォルダ（tile_index.yaml のある所）> \\
      [--extrinsic 0.9,0,2.0,0,0,0] [--mgrs-origin 0,0] [--scans 40]

真値の姿勢と取り付け位置で、いくつかのスキャンを地図座標に移し、地図の点がある格子（--voxel）に入る点の割合を数える。
取り付け位置の 6 つの値を 1 つずつ動かして、割合が最も大きくなる値を探す（粗い → 細かい）。最初の値と見つけた値、
それぞれの割合を出す。見つけた値は awsim_to_bag.py の --lidar-extrinsic に渡す。
地図のタイルは tiled_pcd_map_tiler の出力（地図座標。真値と同じ座標系）。必要なパッケージ: numpy、PyYAML。
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "tools" / "tile_demo"))
import mcap_io as IO  # noqa: E402
import rigid as A  # noqa: E402
from make_bag import read_tile  # noqa: E402

PARAMS = ["x", "y", "z", "roll", "pitch", "yaw"]


def voxel_keys(p, res):
    k = np.floor(p / res).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def score(ex, scans, occ, res):
    T = A.se3(A.rpy_to_rot(*np.deg2rad(ex[3:])), ex[:3])
    hit = tot = 0
    for R, p, pts in scans:
        pm = (pts @ T[:3, :3].T + T[:3, 3]) @ R.T + p
        k = voxel_keys(pm, res)
        i = np.clip(np.searchsorted(occ, k), 0, len(occ) - 1)
        hit += int(np.sum(occ[i] == k))
        tot += len(k)
    return hit / max(tot, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("tiles", type=Path)
    ap.add_argument("--extrinsic", default="0.9,0,2.0,0,0,0", help="最初の値 'x,y,z,roll,pitch,yaw'（m、deg）")
    ap.add_argument("--gt-topic", default="/awsim/ground_truth/vehicle/pose")
    ap.add_argument("--points-topic", default="/sensing/lidar/top/pointcloud_raw")
    ap.add_argument("--mgrs-origin", default="0,0", help="真値に足して地図のタイルの座標にする値 'E,N'（ふつうは 0,0）")
    ap.add_argument("--scans", type=int, default=40, help="使うスキャンの数（動いている間から等間隔に選ぶ）")
    ap.add_argument("--points", type=int, default=4000, help="1 スキャンから使う点の数")
    ap.add_argument("--voxel", type=float, default=0.3)
    ap.add_argument("--no-search", action="store_true", help="割合を出すだけで、値を探さない")
    args = ap.parse_args()
    ex0 = np.array([float(v) for v in args.extrinsic.split(",")])
    off = np.array([float(v) for v in args.mgrs_origin.split(",")] + [0.0])
    rng = np.random.default_rng(0)

    gt = []
    for _t, _y, _tl, data in IO.read_messages(args.bag, {args.gt_topic}):
        t, _f, p, q = IO.pose_stamped(data)
        gt.append((t, *p, *q))
    gt = np.array(sorted(gt))
    if len(gt) < 10:
        sys.exit(f"{args.gt_topic} が無い")
    speed = np.r_[0, np.linalg.norm(np.diff(gt[:, 1:3], axis=0), axis=1) / np.maximum(np.diff(gt[:, 0]), 1e-6)]

    heads = []
    for _t, _y, _tl, data in IO.read_messages(args.bag, {args.points_topic}):
        heads.append(IO.cloud_header(data)[0])
    ts = np.array(heads)
    moving = np.interp(ts, gt[:, 0], speed) > 0.3
    cand = np.nonzero(moving & (ts > gt[0, 0]) & (ts < gt[-1, 0]))[0]
    if len(cand) == 0:
        cand = np.nonzero((ts > gt[0, 0]) & (ts < gt[-1, 0]))[0]
    pick = set(cand[np.linspace(0, len(cand) - 1, min(args.scans, len(cand))).astype(int)].tolist())

    scans = []
    for k, (_t, _y, _tl, data) in enumerate(IO.read_messages(args.bag, {args.points_topic})):
        if k not in pick:
            continue
        t, _f, xyz = IO.cloud_xyz(data)
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(r > 2.0) & (r < 60.0)]
        if len(xyz) > args.points:
            xyz = xyz[rng.choice(len(xyz), args.points, replace=False)]
        p = A.interp(gt[:, 0], gt[:, 1:4], np.array([t]))[0] + off
        R = A.quat_to_rot(A.quat_interp(gt[:, 0], gt[:, 4:8], np.array([t])))[0]
        scans.append((R, p, xyz.astype(float)))
    print(f"スキャン {len(scans)} 個（各 {args.points} 点まで）")

    index = yaml.safe_load((args.tiles / "tile_index.yaml").read_text())
    tsz = float(index.get("tile_size", 20))
    need = set()
    for _R, p, _x in scans:
        ix, iy = int(np.floor(p[0] / tsz)), int(np.floor(p[1] / tsz))
        need |= {(ix + a, iy + b) for a in range(-4, 5) for b in range(-4, 5)}
    pts = [read_tile(args.tiles / t["file"]) for t in index["tiles"] if (t["ix"], t["iy"]) in need]
    if not pts:
        sys.exit("真値の近くにタイルが無い（--mgrs-origin か、地図と真値の座標系を確かめる）")
    occ = np.unique(voxel_keys(np.vstack(pts), args.voxel))
    print(f"地図: タイル {len(pts)} 枚、格子 {len(occ)} 個（{args.voxel} m）")

    s0 = score(ex0, scans, occ, args.voxel)
    print(f"最初の値 {np.round(ex0, 3).tolist()}: 地図の格子に入る点の割合 {s0 * 100:.1f} %")
    if args.no_search:
        return
    ex, best = ex0.copy(), s0
    for steps in ([0.5, 0.5, 0.5, 2.0, 2.0, 5.0], [0.1, 0.1, 0.1, 0.5, 0.5, 1.0], [0.02, 0.02, 0.02, 0.1, 0.1, 0.2]):
        for _ in range(3):
            improved = False
            for j, st in enumerate(steps):
                for sgn in (1, -1):
                    while True:
                        cand_ex = ex.copy()
                        cand_ex[j] += sgn * st
                        s = score(cand_ex, scans, occ, args.voxel)
                        if s > best + 1e-4:
                            ex, best, improved = cand_ex, s, True
                        else:
                            break
            if not improved:
                break
    print(f"見つけた値 {np.round(ex, 3).tolist()}: 割合 {best * 100:.1f} %")
    print(f"  --lidar-extrinsic {','.join(f'{v:.3f}' for v in ex)}")
    if best < 0.6:
        print("注意: 割合が低い。地図と真値の座標系（--mgrs-origin）や、最初の値が大きく違っていないかを確かめる")


if __name__ == "__main__":
    main()
