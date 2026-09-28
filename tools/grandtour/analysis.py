"""GrandTour の事前確認で使う計算（numpy だけに依存する。Zarr の読み込みは gt_zarr.py）。

姿勢は、位置 p（N×3）とクォータニオン q（N×4、並びは x, y, z, w。GrandTour の Zarr と同じ）で扱う。
T_A_B は「フレーム B の姿勢を A で表したもの」（p_A = T_A_B p_B）。
"""
import numpy as np


# ------------------------------------------------------------------ 回転
def quat_to_rot(q):
    """クォータニオン（x, y, z, w）→ 回転行列。q は (4,) か (N, 4)。"""
    q = np.asarray(q, dtype=float)
    single = q.ndim == 1
    q = np.atleast_2d(q)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R[0] if single else R


def rot_to_quat(R):
    """回転行列 → クォータニオン（x, y, z, w）。"""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def yaw_of(R):
    """回転行列（N×3×3 か 3×3）の yaw [rad]。"""
    R = np.asarray(R)
    return np.arctan2(R[..., 1, 0], R[..., 0, 0])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def se3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_se3(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


# ------------------------------------------------------------------ 時系列
def gaps(t, max_gap):
    """時刻の間隔が max_gap を超えた所の (開始時刻, 長さ) の並び。"""
    t = np.asarray(t)
    if len(t) < 2:
        return []
    d = np.diff(t)
    idx = np.nonzero(d > max_gap)[0]
    return [(float(t[i]), float(d[i])) for i in idx]


def interp(t_src, x_src, t_dst):
    """列ごとの線形補間。範囲外は NaN。"""
    x_src = np.asarray(x_src, dtype=float)
    one_d = x_src.ndim == 1
    x2 = x_src[:, None] if one_d else x_src
    out = np.empty((len(t_dst), x2.shape[1]))
    for c in range(x2.shape[1]):
        out[:, c] = np.interp(t_dst, t_src, x2[:, c], left=np.nan, right=np.nan)
    return out[:, 0] if one_d else out


def subsample(t, period):
    """時刻がおよそ period ごとになるように間引いた添字。"""
    t = np.asarray(t)
    if len(t) == 0:
        return np.array([], dtype=int)
    grid = np.arange(t[0], t[-1] + 1e-9, period)
    idx = np.unique(np.clip(np.searchsorted(t, grid), 0, len(t) - 1))
    return idx


def quat_interp(t, q, tq):
    """クォータニオンの列を時刻 tq に補間する（隣り合う 2 つの正規化線形補間。範囲外は端の値）。"""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float).copy()
    # 隣どうしで符号をそろえる（q と -q は同じ回転）
    flip = np.cumsum(np.concatenate([[0], (np.sum(q[1:] * q[:-1], axis=1) < 0).astype(int)])) % 2 == 1
    q[flip] *= -1
    tq = np.clip(np.asarray(tq, dtype=float), t[0], t[-1])
    j = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
    i = j - 1
    w = ((tq - t[i]) / np.maximum(t[j] - t[i], 1e-12))[:, None]
    out = (1 - w) * q[i] + w * q[j]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def pct(x, q):
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if len(x) else float("nan")


# ------------------------------------------------------------------ 経路
def path_length(xy):
    xy = np.asarray(xy)
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def resample_path(xy, step):
    """経路を step [m] ごとの点に並べ直す。(点 M×2, 進行方向の角度 M) を返す。"""
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        return xy.copy(), np.zeros(len(xy))
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < step:
        return xy[[0, -1]], np.full(2, np.arctan2(*(xy[-1] - xy[0])[::-1]))
    si = np.arange(0.0, s[-1], step)
    pts = np.column_stack([np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])])
    d = np.gradient(pts, axis=0)
    head = np.arctan2(d[:, 1], d[:, 0])
    return pts, head


def nearest(a_pts, b_pts, chunk=2000):
    """b の各点に最も近い a の点の (距離, 添字)。"""
    a_pts = np.asarray(a_pts)
    b_pts = np.asarray(b_pts)
    dist = np.empty(len(b_pts))
    idx = np.empty(len(b_pts), dtype=int)
    for i in range(0, len(b_pts), chunk):
        d2 = ((b_pts[i:i + chunk, None, :] - a_pts[None, :, :]) ** 2).sum(-1)
        j = np.argmin(d2, axis=1)
        idx[i:i + chunk] = j
        dist[i:i + chunk] = np.sqrt(d2[np.arange(len(j)), j])
    return dist, idx


def overlap(a_pts, a_head, b_pts, b_head, radius, step):
    """b の経路のうち、a の経路から radius 以内にある部分の割合・長さと、そのときの向き。

    向きは、a の最も近い点の進行方向との差で、45° 以内を同じ向き、135° 以上を逆向きとする。
    """
    if len(a_pts) == 0 or len(b_pts) == 0:
        return dict(frac=0.0, length=0.0, same=0.0, opposite=0.0)
    dist, idx = nearest(a_pts, b_pts)
    near = dist <= radius
    dh = np.abs(wrap(b_head - a_head[idx]))
    return dict(
        frac=float(near.mean()),
        length=float(near.sum() * step),
        same=float((near & (dh < np.deg2rad(45))).sum() * step),
        opposite=float((near & (dh > np.deg2rad(135))).sum() * step),
    )


def group_sites(paths, link_dist):
    """経路どうしが link_dist 以内に近づくものを同じ場所としてまとめる。{名前: 経路} → [[名前, ...], ...]。"""
    names = list(paths)
    parent = {n: n for n in names}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    for i, a in enumerate(names):
        for b in names[i + 1:]:
            pa, pb = paths[a], paths[b]
            if len(pa) == 0 or len(pb) == 0:
                continue
            d, _ = nearest(pa[::10] if len(pa) > 10 else pa, pb[::10] if len(pb) > 10 else pb)
            if d.min() <= link_dist:
                parent[find(a)] = find(b)
    groups = {}
    for n in names:
        groups.setdefault(find(n), []).append(n)
    return list(groups.values())


# ------------------------------------------------------------------ 静止
def stationary_start(t, speed, yaw_rate, v_th=0.05, w_th=0.05):
    """最初から止まっている時間 [s]（speed < v_th かつ |yaw_rate| < w_th が続く間）。"""
    t = np.asarray(t)
    moving = (np.asarray(speed) >= v_th) | (np.abs(np.asarray(yaw_rate)) >= w_th)
    if len(t) == 0:
        return 0.0
    i = np.argmax(moving) if moving.any() else len(t) - 1
    return float(t[i] - t[0])


# ------------------------------------------------------------------ オドメトリの約束事の確認
def _body_displacements(t, pos, R, dt, min_move):
    j = np.clip(np.searchsorted(t, t + dt), 0, len(t) - 1)
    d = pos[j] - pos
    dh = np.linalg.norm(d[:, :2], axis=1)
    keep = (dh > min_move) & (j > np.arange(len(t)))
    d_body = np.einsum("nji,nj->ni", R[keep], d[keep])  # R^T d
    return d_body, np.linalg.norm(d[keep], axis=1)


def pose_semantics_score(t, p, q, dt=0.5, min_move=0.05):
    """姿勢の解釈の確からしさ。

    脚のロボットは、機体に対してほぼ決まった向き（ふつうは前）に歩く。姿勢の解釈が正しければ、機体座標系で見た
    移動の向きがそろう。どの軸が前かは分からない（真値のフレームが後ろ向きのこともある）ので、向きのそろい方
    （単位ベクトルの平均の長さ、0〜1）で比べる。旋回があるほど、正しくない解釈との差が大きくなる。

    standard: 姿勢 = 子フレーム（機体）を親フレームで表したもの（ROS の Odometry と同じ）
    inverted: 姿勢 = 親フレームを子フレームで表したもの（逆向きに格納されている場合）
    *_dir は、そろった向きの角度 [deg]（機体の +x から反時計回り。0 なら前に歩いている）。
    """
    t = np.asarray(t, dtype=float)
    p = np.asarray(p, dtype=float)
    R = quat_to_rot(q)
    out = {}
    for name, (pos, Rb) in {
        "standard": (p, R),
        "inverted": (-np.einsum("nji,nj->ni", R, p), np.transpose(R, (0, 2, 1))),
    }.items():
        d_body, _ = _body_displacements(t, pos, Rb, dt, min_move)
        h = np.linalg.norm(d_body[:, :2], axis=1)
        ok = h > 1e-9
        if ok.sum() == 0:
            out[name], out[name + "_dir"] = float("nan"), float("nan")
        else:
            u = (d_body[ok, :2] / h[ok, None]).mean(0)
            out[name] = float(np.linalg.norm(u))
            out[name + "_dir"] = float(np.rad2deg(np.arctan2(u[1], u[0])))
        out[name + "_samples"] = int(ok.sum())
    return out


def twist_frame_residual(t, p, q, v, period=0.1):
    """twist の並進速度が、子フレーム（機体）と親フレームのどちらで表されているか。

    位置を数値微分した速度（親フレーム）と、R v（子フレームなら一致）・v（親フレームなら一致）の差の RMS [m/s]。
    姿勢は standard の解釈とする。
    """
    t = np.asarray(t, dtype=float)
    idx = subsample(t, period)
    if len(idx) < 5:
        return dict(child=float("nan"), parent=float("nan"))
    ts, ps = t[idx], np.asarray(p, dtype=float)[idx]
    R = quat_to_rot(np.asarray(q)[idx])
    vs = np.asarray(v, dtype=float)[idx]
    v_fd = np.gradient(ps, ts, axis=0)
    child = np.einsum("nij,nj->ni", R, vs)
    return dict(
        child=float(np.sqrt(np.mean(np.sum((v_fd - child) ** 2, axis=1)))),
        parent=float(np.sqrt(np.mean(np.sum((v_fd - vs) ** 2, axis=1)))),
    )


def standardize_pose(p, q, semantics):
    """姿勢を standard の解釈（機体を親フレームで表したもの）にそろえる。"""
    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    if semantics == "inverted":
        R = quat_to_rot(q)
        return -np.einsum("nji,nj->ni", R, p), q * np.array([-1.0, -1.0, -1.0, 1.0])
    return p, q


def odom_convention(t, p, q, v=None, decisive=0.5):
    """オドメトリの約束事（姿勢の解釈と、速度の座標系）を決める。

    速度（twist）があれば、姿勢の解釈 2 通り × 速度の座標系 2 通りのそれぞれで、位置を微分した速度との差を求め、
    最も小さい組を選ぶ（2 番目の decisive 倍より小さいとき「速度で決定」）。決まらなければ、機体座標系で見た移動の
    向きのそろい方（pose_semantics_score）で姿勢の解釈だけを決める。
    """
    sc = pose_semantics_score(t, p, q)
    out = dict(score_standard=sc["standard"], score_inverted=sc["inverted"], residual={})
    if v is not None and len(v) == len(t):
        for sem in ("standard", "inverted"):
            ps, qs = standardize_pose(p, q, sem)
            r = twist_frame_residual(t, ps, qs, v)
            out["residual"][f"{sem}/child"] = r["child"]
            out["residual"][f"{sem}/parent"] = r["parent"]
    res = {k: x for k, x in out["residual"].items() if np.isfinite(x)}
    if len(res) >= 2:
        order = sorted(res, key=res.get)
        best, second = order[0], order[1]
        if res[best] < decisive * res[second]:
            out["semantics"], out["twist_frame"] = best.split("/")
            out["decided_by"] = "twist"
    if "semantics" not in out:
        out["semantics"] = "standard" if not (sc["inverted"] > sc["standard"]) else "inverted"
        out["twist_frame"] = None
        out["decided_by"] = "direction"
    out["walk_dir_deg"] = sc[out["semantics"] + "_dir"]
    return out


def rot_log(R):
    """回転行列（N×3×3）→ 回転ベクトル（N×3）。"""
    R = np.asarray(R, dtype=float)
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    ang = np.arccos(c)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], axis=1)
    s = np.sin(ang)
    k = np.where(s > 1e-9, ang / (2 * np.where(s > 1e-9, s, 1.0)), 0.5)
    return v * k[:, None]


def angle_between(Ra, Rb):
    """2 つの回転の差の角度 [deg]。"""
    return float(np.rad2deg(np.linalg.norm(rot_log((Ra.T @ Rb)[None])[0])))


def handeye_rotation(t_a, q_a, t_b, q_b, dt=0.37, period=0.23, min_angle=0.02):
    """2 つの姿勢の列 R_a(t) = C R_b(t) X（C, X は定数）から、X（b の機体 → a の機体の向き）を求める。

    相対回転 A = R_a(i)^T R_a(j)、B = R_b(i)^T R_b(j) の回転ベクトルは b_vec = X a_vec を満たす。
    歩行の揺れで roll / pitch の回転もあるので、3 軸とも決まることが多い。dt と period は、歩行の周期と
    そろって回転が打ち消されないように、半端な値にしてある。
    """
    t_a, t_b = np.asarray(t_a), np.asarray(t_b)
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1]) - dt
    if t1 <= t0:
        return None
    ts = np.arange(t0, t1, period)
    Ra0, Ra1 = quat_to_rot(quat_interp(t_a, q_a, ts)), quat_to_rot(quat_interp(t_a, q_a, ts + dt))
    Rb0, Rb1 = quat_to_rot(quat_interp(t_b, q_b, ts)), quat_to_rot(quat_interp(t_b, q_b, ts + dt))
    A = np.einsum("nji,njk->nik", Ra0, Ra1)
    B = np.einsum("nji,njk->nik", Rb0, Rb1)
    a, b = rot_log(A), rot_log(B)
    keep = np.linalg.norm(b, axis=1) > min_angle
    if keep.sum() < 10:
        return None
    X, _ = kabsch_origin(a[keep], b[keep])
    res = np.linalg.norm(a[keep] @ X.T - b[keep], axis=1)
    return dict(X=X, rms_deg=float(np.rad2deg(np.sqrt(np.mean(res ** 2)))), n=int(keep.sum()))


def lever_from_odometry(t_a, p_a, q_a, t_b, p_b, q_b, X, dt=2.0, period=0.5, min_turn_deg=5.0):
    """2 つの軌跡から、機体 b から見た機体 a の原点の位置 l を求める（静的 TF の並進の確認）。

    a: 真値（box_base の姿勢、地球座標）、b: 脚のオドメトリ（base の姿勢、odom）。X = R_b_a（b → a の向き）。
    モデル: p_a(t) = C p_b(t) + d + C R_b(t) l（C = R_a X^T R_b^T。オドメトリの yaw のずれを含むので窓ごとに求める）。
    短い窓（dt）ごとに C^T Δp_a - Δp_b = ΔR_b l を並べ、最小二乗で解く。向きが変わる窓（min_turn_deg 以上）だけを使う。
    水平（x, y）は旋回で決まる。高さ（z）は roll / pitch の変化でしか決まらないので、あまり当てにならない。
    """
    t_a, t_b = np.asarray(t_a, dtype=float), np.asarray(t_b, dtype=float)
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1]) - dt
    if t1 <= t0:
        return None
    ts = np.arange(t0, t1, period)
    pa0, pa1 = interp(t_a, p_a, ts), interp(t_a, p_a, ts + dt)
    pb0, pb1 = interp(t_b, p_b, ts), interp(t_b, p_b, ts + dt)
    Ra0 = quat_to_rot(quat_interp(t_a, q_a, ts))
    Rb0, Rb1 = quat_to_rot(quat_interp(t_b, q_b, ts)), quat_to_rot(quat_interp(t_b, q_b, ts + dt))
    turn = np.rad2deg(np.linalg.norm(rot_log(np.einsum("nji,njk->nik", Rb0, Rb1)), axis=1))
    ok = (turn >= min_turn_deg) & np.all(np.isfinite(pa0) & np.isfinite(pa1) & np.isfinite(pb0) & np.isfinite(pb1), axis=1)
    if ok.sum() < 10:
        return None
    C = np.einsum("nij,kj,nlk->nil", Ra0[ok], X, Rb0[ok])  # Ra X^T Rb^T
    rhs = np.einsum("nji,nj->ni", C, pa1[ok] - pa0[ok]) - (pb1[ok] - pb0[ok])
    M = Rb1[ok] - Rb0[ok]
    l, *_ = np.linalg.lstsq(M.reshape(-1, 3), rhs.reshape(-1), rcond=None)
    res = np.einsum("nij,j->ni", M, l) - rhs
    return dict(l=l, rms=float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))), n=int(ok.sum()))


def kabsch_origin(src, dst):
    """dst ≈ R src（並進なし）。"""
    H = np.asarray(src).T @ np.asarray(dst)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, None


# ------------------------------------------------------------------ 剛体の当てはめ
def similarity_2d(src, dst):
    """dst ≈ s R src + t（2 次元の相似変換。Umeyama の方法）。(s, R, t, 残差の RMS) を返す。"""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    ms, md = src.mean(0), dst.mean(0)
    X, Y = src - ms, dst - md
    U, S, Vt = np.linalg.svd(X.T @ Y / len(X))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, d])
    R = Vt.T @ D @ U.T
    s = float(np.trace(np.diag(S) @ D) / np.mean(np.sum(X ** 2, axis=1)))
    t = md - s * R @ ms
    res = np.linalg.norm(src @ (s * R).T + t - dst, axis=1)
    return s, R, t, float(np.sqrt(np.mean(res ** 2)))


def rpy_of(R):
    """回転行列 → (roll, pitch, yaw)（R = Rz(yaw) Ry(pitch) Rx(roll)。gll_ros2 の rpyToMatrix と同じ順）。"""
    return (float(np.arctan2(R[2, 1], R[2, 2])), float(np.arcsin(np.clip(-R[2, 0], -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def kabsch(src, dst):
    """dst ≈ R src + t となる (R, t)（最小二乗）。"""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def fit_station(prism, p_world, R_world_body, iters=30):
    """トータルステーションの座標で測ったプリズムの位置を、真値の軌跡に合わせる。

    モデル: R_s prism + t_s = p_world + R_world_body l（l は機体座標系でのプリズムの位置。未知）。
    R_s を Kabsch で、(t_s, l) を線形の最小二乗で、交互に解く。残差の RMS・p95・最大値と l を返す。
    """
    prism = np.asarray(prism, dtype=float)
    p_world = np.asarray(p_world, dtype=float)
    Rb = np.asarray(R_world_body, dtype=float)
    n = len(prism)
    l = np.zeros(3)
    M = np.zeros((3 * n, 6))
    M[:, :3] = np.tile(np.eye(3), (n, 1))
    M[:, 3:] = -Rb.reshape(3 * n, 3)
    for _ in range(iters):
        Rs, ts = kabsch(prism, p_world + np.einsum("nij,j->ni", Rb, l))
        rhs = (p_world - prism @ Rs.T).reshape(-1)
        x, *_ = np.linalg.lstsq(M, rhs, rcond=None)
        ts, l = x[:3], x[3:]
    res = prism @ Rs.T + ts - (p_world + np.einsum("nij,j->ni", Rb, l))
    e = np.linalg.norm(res, axis=1)
    yaw = np.unwrap(yaw_of(Rb))
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), max=float(e.max()), p95=pct(e, 95),
                lever=l, R=Rs, t=ts, n=int(n), yaw_span_deg=float(np.rad2deg(yaw.max() - yaw.min())))
