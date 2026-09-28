"""GrandTour の Zarr（Hugging Face 版）の読み込みと座標変換。

配列名と属性は、GrandTour のサンプル（github.com/leggedrobotics/grand_tour_dataset の
examples_hugging_face。MIT ライセンス）に合わせてある。必要なパッケージ: numpy、zarr（3.x）、pyproj。
"""
from pathlib import Path

import numpy as np

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
