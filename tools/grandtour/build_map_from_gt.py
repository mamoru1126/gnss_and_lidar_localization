#!/usr/bin/env python3
"""案 A の点群地図: 地図用ミッションの Livox の点群を、真値の姿勢で重ねる（検証計画 4.3 節）。

  python3 build_map_from_gt.py $GLL_DATA/grandtour/<地図用のフォルダ名> $GLL_DATA/grandtour/maps/spx2 \\
      --near $GLL_DATA/grandtour/<照合用のフォルダ名>
  tiled_pcd_map_tiler -i $GLL_DATA/grandtour/maps/spx2/map.pcd -o $GLL_DATA/grandtour/maps/spx2/tiles \\
      --tile-size 20 --voxel-size 0.2

出力（<out_dir>）:
  map.pcd      地図（binary。x, y, z, intensity。地図座標 = 地図用ミッションの最初の base の位置を原点にした ENU、実距離）
  maps.yaml    地図グループ 1 つとアンカー（UTM）。tile_index は tiles/tile_index.yaml（上の tiled_pcd_map_tiler の -o）

地図座標と UTM の関係（アンカー）は、真値の ENU → UTM の当てはめ（gt_zarr.ground_truth）から決まるので、
アンカーの誤差は無い（案 A）。--near を付けると、照合用ミッションの経路の近く（水平 --near-radius、高さ --z-below〜--z-above）の
点だけを地図に入れる（階段の上の階など、照合で通らない場所を除くため）。
必要なパッケージ: numpy、zarr（3.x）、pyproj。
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analysis as A  # noqa: E402
import gt_zarr as Z  # noqa: E402
import missions as M  # noqa: E402


def voxel_merge(pts, inten, res):
    """ボクセル（一辺 res）ごとに 1 点（最初の点）を残す。"""
    if len(pts) == 0:
        return pts, inten
    k = np.floor(pts / res).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    i = np.sort(i)
    return pts[i], inten[i]


def write_pcd(path, pts, inten):
    n = len(pts)
    arr = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    arr["x"], arr["y"], arr["z"], arr["intensity"] = pts[:, 0], pts[:, 1], pts[:, 2], inten
    head = ("# .PCD v0.7 - tools/grandtour/build_map_from_gt.py\nVERSION 0.7\nFIELDS x y z intensity\n"
            "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    with open(path, "wb") as fp:
        fp.write(head.encode())
        fp.write(arr.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission_dir", type=Path, help="地図用のミッション")
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--points", choices=["undistorted", "raw"], default="undistorted")
    ap.add_argument("--near", nargs="*", type=Path, default=[], help="照合用のミッション（この経路の近くの点だけを残す）")
    ap.add_argument("--near-radius", type=float, default=30.0, help="照合用の経路からの水平距離 [m]")
    ap.add_argument("--z-below", type=float, default=1.5, help="照合用の経路の base の高さより下に残す範囲 [m]")
    ap.add_argument("--z-above", type=float, default=8.0, help="照合用の経路の base の高さより上に残す範囲 [m]")
    ap.add_argument("--min-range", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--self-radius", type=float, default=0.8, help="base から水平にこの距離以内の点は、ロボット自身として除く [m]")
    ap.add_argument("--max-sigma", type=float, default=0.05, help="真値の水平 σ がこれを超える時刻のスキャンは使わない [m]")
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--every", type=int, default=1, help="スキャンを n 個に 1 個使う")
    ap.add_argument("--utm-zone", type=int, default=32)
    ap.add_argument("--group-id", default="", help="地図グループの id（省略するとミッションの略称）")
    args = ap.parse_args()

    mdir = args.mission_dir.expanduser()
    code = M.code_of(mdir.name)
    root = Z.open_mission(mdir)
    topic = "livox_points_undistorted" if args.points == "undistorted" else "livox_points"
    gt = Z.ground_truth(root, args.utm_zone)
    fit = gt["fit"]
    tf = Z.tf_table(root)
    T_base_lidar = Z.static_transform(tf, "base", root[topic].attrs.get("frame_id", "livox_lidar"))

    # 地図座標: 地図用ミッションの ENU（実距離）を、最初の base の位置だけずらしたもの
    p0 = gt["T_enu"][0, :3, 3].copy()

    def enu_to_utm(p):
        e, n = fit["s"] * fit["R"] @ p[:2] + fit["t"]
        return float(e), float(n), float(p[2] + fit["dz"])

    # 照合用ミッションの経路（地図座標）
    near_xyz = []
    for nd in args.near:
        g2 = Z.ground_truth(Z.open_mission(nd.expanduser()), args.utm_zone)
        u = g2["T_utm"][:, :3, 3][::20]
        en = (u[:, :2] - fit["t"]) @ (fit["R"] / fit["s"])  # UTM → 地図用ミッションの ENU（R^T / s）
        near_xyz.append(np.column_stack([en - p0[:2], u[:, 2] - fit["dz"] - p0[2]]))
    near_xyz = np.vstack(near_xyz) if near_xyz else None

    ts = np.asarray(root[topic]["timestamp"][:], dtype=float)
    idx = np.nonzero((ts >= gt["t"][0]) & (ts <= gt["t"][-1]))[0][:: max(args.every, 1)]
    T_scan = Z.interp_pose(gt["t"], gt["T_enu"], ts[idx])
    sig = A.interp(gt["t"], gt["sigma_h"], ts[idx]) if "sigma_h" in gt else np.zeros(len(idx))

    pts_all = np.zeros((0, 3))
    int_all = np.zeros(0)
    buf_p, buf_i = [], []
    used = skipped = 0
    for k, i in enumerate(idx):
        if not np.all(np.isfinite(T_scan[k])) or (np.isfinite(sig[k]) and sig[k] > args.max_sigma):
            skipped += 1
            continue
        s = Z.read_scan(root, topic, int(i))
        pl = s["points"]
        inten = s.get("intensity", np.zeros(len(pl)))
        r = np.linalg.norm(pl, axis=1)
        keep = (r >= args.min_range) & (r <= args.max_range)
        pb = pl[keep] @ T_base_lidar[:3, :3].T + T_base_lidar[:3, 3]
        keep2 = np.hypot(pb[:, 0], pb[:, 1]) > args.self_radius
        pb = pb[keep2]
        pm = pb @ T_scan[k][:3, :3].T + T_scan[k][:3, 3] - p0
        buf_p.append(pm)
        buf_i.append(inten[keep][keep2])
        used += 1
        if len(buf_p) >= 50:
            pts_all, int_all = voxel_merge(np.vstack([pts_all] + buf_p), np.concatenate([int_all] + buf_i), args.voxel)
            buf_p, buf_i = [], []
        if (k + 1) % 500 == 0:
            print(f"  {k + 1}/{len(idx)} scans, {len(pts_all)} points", flush=True)
    if buf_p:
        pts_all, int_all = voxel_merge(np.vstack([pts_all] + buf_p), np.concatenate([int_all] + buf_i), args.voxel)

    n_before = len(pts_all)
    if near_xyz is not None and len(pts_all):
        d, j = A.nearest(near_xyz[:, :2], pts_all[:, :2])
        dz = pts_all[:, 2] - near_xyz[j, 2]
        keep = (d <= args.near_radius) & (dz >= -args.z_below) & (dz <= args.z_above)
        pts_all, int_all = pts_all[keep], int_all[keep]
        print(f"  照合用の経路の近くの点だけを残した: {n_before} → {len(pts_all)}")

    out = args.out_dir.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    write_pcd(out / "map.pcd", pts_all, int_all)

    e0, n0, h0 = enu_to_utm(p0)
    heading = (90.0 - fit["theta_deg"]) % 360.0  # 地図の x 軸（ENU の東）の、グリッド北から時計回りの角度
    gid = args.group_id or code.lower().replace("-", "")
    (out / "maps.yaml").write_text(
        f"# {code}（{mdir.name}）の案 A の地図。tools/grandtour/build_map_from_gt.py が作った。\n"
        f"# 地図座標 = {code} の ENU（実距離）を、最初の base の位置を原点にずらしたもの。\n"
        f"# ENU → UTM の当てはめ: 回転 {fit['theta_deg']:.4f}°、縮尺 {fit['s']:.6f}、残差 {fit['rms'] * 100:.1f} cm\n"
        f"utm: {{zone: {args.utm_zone}, hemisphere: north}}\n"
        f"map_groups:\n"
        f"  - id: {gid}\n"
        f"    tile_index: tiles/tile_index.yaml\n"
        f"    anchor: {{map_point: [0.0, 0.0, 0.0], easting: {e0:.4f}, northing: {n0:.4f}, "
        f"ellipsoid_height: {h0:.3f}, grid_heading_deg: {heading:.6f}}}\n")
    ext = pts_all.max(0) - pts_all.min(0) if len(pts_all) else np.zeros(3)
    print(f"{code}: スキャン {used} 個を使った（真値が無い・σ が大きいので {skipped} 個を除いた）。"
          f"点 {len(pts_all)}、範囲 {np.round(ext, 1)} m")
    print(f"  {out / 'map.pcd'}\n  {out / 'maps.yaml'}")
    print(f"次に: tiled_pcd_map_tiler -i {out / 'map.pcd'} -o {out / 'tiles'} --tile-size 20 --voxel-size 0.2")
    if math.isnan(h0):
        print("注意: 楕円体高が求まらなかった")


if __name__ == "__main__":
    main()
