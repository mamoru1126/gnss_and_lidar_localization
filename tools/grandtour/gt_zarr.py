"""GrandTour の Zarr（Hugging Face 版）の読み込みと座標変換。

配列名と属性は、GrandTour のサンプル（github.com/leggedrobotics/grand_tour_dataset の
examples_hugging_face。MIT ライセンス）に合わせてある。必要なパッケージ: numpy、zarr（3.x）、pyproj。
"""
from pathlib import Path

import numpy as np

import analysis as A
from analysis import inv_se3, quat_to_rot, se3


def open_mission(mission_dir):
    import zarr
    return zarr.open_group(store=str(Path(mission_dir) / "data"), mode="r")


def topics(root):
    return set(root.keys())


def _sorted_finite(t, *arrays):
    t = np.asarray(t, dtype=float).reshape(-1)
    ok = np.isfinite(t)
    for a in arrays:
        a = np.asarray(a, dtype=float)
        ok &= np.all(np.isfinite(a.reshape(len(t), -1)), axis=1)
    order = np.argsort(t[ok], kind="stable")
    return [t[ok][order]] + [np.asarray(a, dtype=float)[ok][order] for a in arrays]


def read_odometry(root, topic):
    """pose_pos / pose_orien（x, y, z, w）/ twist_lin / twist_ang。frame_id は属性。"""
    g = root[topic]
    have = set(g.keys())
    t, p, q = _sorted_finite(g["timestamp"][:], g["pose_pos"][:], g["pose_orien"][:])
    out = dict(t=t, p=p, q=q, frame_id=g.attrs.get("frame_id", ""))
    if {"twist_lin", "twist_ang"} <= have:
        t2, v, w = _sorted_finite(g["timestamp"][:], g["twist_lin"][:], g["twist_ang"][:])
        out.update(t_twist=t2, v=v, w=w)
    return out


def read_navsatfix(root, topic="navsatfix_cpt7_ie_tc"):
    """lat / long / alt / cov（3×3 を 9 要素に並べたもの）。"""
    g = root[topic]
    n = int(g["timestamp"].shape[0])
    cols = [np.asarray(g[k][:], dtype=float).reshape(n, -1) for k in ("lat", "long", "alt")]
    has_cov = "cov" in set(g.keys())
    if has_cov:
        cov = np.asarray(g["cov"][:], dtype=float).reshape(n, -1)
        has_cov = cov.shape[1] == 9
    arrays = cols + ([cov] if has_cov else [])
    res = _sorted_finite(g["timestamp"][:], *arrays)
    out = dict(t=res[0], lat=res[1][:, 0], lon=res[2][:, 0], alt=res[3][:, 0], frame_id=g.attrs.get("frame_id", ""))
    if has_cov:
        out["cov"] = res[4]
    return out


def read_prism(root, topic="prism_position"):
    g = root[topic]
    t, p = _sorted_finite(g["timestamp"][:], g["point"][:])
    return dict(t=t, p=p, frame_id=g.attrs.get("frame_id", ""))


def read_imu(root, topic):
    g = root[topic]
    t, acc, gyro = _sorted_finite(g["timestamp"][:], g["lin_acc"][:], g["ang_vel"][:])
    return dict(t=t, acc=acc, gyro=gyro, frame_id=g.attrs.get("frame_id", ""),
                description=g.attrs.get("description", ""))


def point_fields(root, topic):
    return sorted(root[topic].keys())


def read_scan(root, topic, i):
    """i 番目のスキャン（points と、あれば intensity）。valid が有効な点の数。"""
    g = root[topic]
    n = int(np.asarray(g["valid"][i]).reshape(-1)[0])
    pts = np.asarray(g["points"][i, :n], dtype=float)
    out = dict(t=float(g["timestamp"][i]), points=pts)
    if "intensity" in set(g.keys()):
        out["intensity"] = np.asarray(g["intensity"][i, :n], dtype=float).reshape(-1)
    return out


def scan_count(root, topic):
    return int(root[topic]["timestamp"].shape[0])


# ------------------------------------------------------------------ 静的 TF
def tf_table(root):
    return dict(root["tf"].attrs["tf"])


def _pq(entry):
    tr, rot = entry["translation"], entry["rotation"]
    return se3(quat_to_rot([rot["x"], rot["y"], rot["z"], rot["w"]]), [tr["x"], tr["y"], tr["z"]])


def static_transform(tf, parent, child):
    """T_parent_child（p_parent = T p_child）。

    GrandTour のサンプルの get_static_transform（zarr_transforms.py）と同じ解釈で計算する。
    tf の各項目は base か box_base を基準にしている（どちらかは base_frame_id で分かる）。
    """
    T_boxbase_to_base = _pq(tf["box_base"])

    def to_base(frame):
        if frame == "base":
            return np.eye(4)
        e = tf[frame]
        T = _pq(e)
        if e.get("base_frame_id") == "box_base":
            T = T @ T_boxbase_to_base
        return T

    return to_base(parent) @ inv_se3(to_base(child))


# ------------------------------------------------------------------ UTM
def to_utm(lat, lon, zone=32, north=True):
    """緯度経度 → UTM（E, N）[m]。"""
    from pyproj import Transformer
    epsg = (32600 if north else 32700) + zone
    tr = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    e, n = tr.transform(np.asarray(lon), np.asarray(lat))
    return np.asarray(e), np.asarray(n)


def meridian_convergence_deg(lat, lon, zone=32, north=True):
    """子午線収差 [deg]（真北からグリッド北への角度。pyproj の定義）。"""
    from pyproj import Proj
    proj = Proj(proj="utm", zone=zone, ellps="WGS84", south=not north)
    return np.asarray(proj.get_factors(np.asarray(lon), np.asarray(lat)).meridian_convergence)


# ------------------------------------------------------------------ 真値（base の姿勢）
def ground_truth(root, zone=32, north=True):
    """真値: base の姿勢を ENU（cpt7_ie_tc の enu_origin。実距離）と UTM（グリッド）で返す。

    cpt7_ie_tc_odometry は box_base を enu_origin で表した姿勢（ROS の Odometry と同じ約束事。事前確認で確かめた）。
    ENU → UTM は、同じ解の緯度経度（navsatfix_cpt7_ie_tc）への 2 次元の相似変換の当てはめで求める
    （回転に子午線収差、縮尺に UTM の縮尺係数が入る。どちらも仮定しないで済む）。
    静的 TF は GrandTour のサンプルと同じ解釈（事前確認で確かめた）。

    戻り値: t、T_enu（N×4×4、ENU で表した base）、T_utm（N×4×4、UTM で表した base。z は楕円体高）、
    sigma_h（水平の標準偏差）、fit（相似変換 s・R・t・theta_deg・rms）。
    """
    tc = read_odometry(root, "cpt7_ie_tc_odometry")
    nav = read_navsatfix(root)
    tf = tf_table(root)
    T_box_base = static_transform(tf, "box_base", "base")
    t = tc["t"]
    R_enu_box = quat_to_rot(tc["q"])
    T_enu_box = np.tile(np.eye(4), (len(t), 1, 1))
    T_enu_box[:, :3, :3] = R_enu_box
    T_enu_box[:, :3, 3] = tc["p"]
    T_enu = T_enu_box @ T_box_base

    e, n = to_utm(nav["lat"], nav["lon"], zone, north)
    en = A.interp(nav["t"], np.column_stack([e, n, nav["alt"]]), t)
    ok = np.all(np.isfinite(en), axis=1)
    if ok.sum() < 10:
        raise ValueError("navsatfix と cpt7_ie_tc_odometry の時刻が重ならない")
    s, R2, t2, rms = A.similarity_2d(tc["p"][ok, :2], en[ok, :2])
    theta = float(np.arctan2(R2[1, 0], R2[0, 0]))
    dz = float(np.median(en[ok, 2] - tc["p"][ok, 2]))
    G = np.eye(4)  # ENU → UTM（水平は相似変換、高さはずらすだけ）
    G[:2, :2] = s * R2
    G[:2, 3] = t2
    G[2, 3] = dz
    T_utm = np.einsum("ij,njk->nik", G, T_enu)
    # 回転は縮尺を含まない形にそろえる
    T_utm[:, :3, :3] = np.einsum("ij,njk->nik", A.rot_z(theta), T_enu[:, :3, :3])
    out = dict(t=t, T_enu=T_enu, T_utm=T_utm, fit=dict(s=s, R=R2, t=t2, dz=dz, theta_deg=float(np.rad2deg(theta)), rms=rms))
    if "cov" in nav:
        sh = np.sqrt(np.maximum(nav["cov"][:, 0], nav["cov"][:, 4]))
        out["sigma_h"] = A.interp(nav["t"], sh, t)
    return out


def interp_pose(t_src, T_src, t_dst):
    """姿勢の列（N×4×4）を時刻 t_dst に補間する（位置は線形、回転は正規化線形補間）。範囲外は NaN。"""
    p = A.interp(t_src, T_src[:, :3, 3], t_dst)
    q_src = np.stack([A.rot_to_quat(R) for R in T_src[:, :3, :3]])
    q = A.quat_interp(t_src, q_src, t_dst)
    T = np.tile(np.eye(4), (len(t_dst), 1, 1))
    T[:, :3, :3] = quat_to_rot(q)
    T[:, :3, 3] = p
    return T
