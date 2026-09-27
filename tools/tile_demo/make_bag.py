#!/usr/bin/env python3
"""tiled_pcd_map_demo の記録とタイルから、RViz で再生できる rosbag（ROS 2、MCAP）を作る（README.md 参照）。

  python3 make_bag.py <タイルのディレクトリ> <frames.json> <出力.mcap>
      [--map-res 0.5] [--tile-res 0.5] [--start-time 1767225600]

frames.json は tiled_pcd_map_demo で --record-interval 0.1 を付けて作る（tf を 10 Hz で出すため）。
ROS も外部の rosbag ライブラリも使わない（MCAP と CDR をこのスクリプトで書く。numpy と PyYAML が必要）。

トピック（frame はすべて map。座標は地図座標のまま）:
  /tf                     tf2_msgs/msg/TFMessage             map → base_link（記録の間隔ごと）
  /tile_demo/map          sensor_msgs/msg/PointCloud2        地図の全体（map-res に間引き。intensity = 地面からの高さ）
  /tile_demo/route        nav_msgs/msg/Path                  走る経路
  /tile_demo/vehicle      visualization_msgs/msg/MarkerArray 車の矢印と load / unload の円（base_link に固定）
  /tile_demo/tiles        visualization_msgs/msg/MarkerArray 読み込み中のタイル（読んだら追加、捨てたら削除する差分）
map・route・vehicle は最初に 1 回だけ出し、QoS を transient local にしてある（後から起動した RViz にも届く）。
"""
import argparse
import json
import math
import pathlib
import struct
import zlib

import numpy as np
import yaml

# ------------------------------------------------------------------ メッセージの定義（ROS 2 Jazzy の .msg からコメントを除いたもの）
MSGDEFS = {
    "builtin_interfaces/msg/Time": "int32 sec\nuint32 nanosec",
    "builtin_interfaces/msg/Duration": "int32 sec\nuint32 nanosec",
    "std_msgs/msg/Header": "builtin_interfaces/Time stamp\nstring frame_id",
    "std_msgs/msg/ColorRGBA": "float32 r\nfloat32 g\nfloat32 b\nfloat32 a",
    "geometry_msgs/msg/Vector3": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/msg/Quaternion": "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1",
    "geometry_msgs/msg/Point": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/msg/Pose": "Point position\nQuaternion orientation",
    "geometry_msgs/msg/PoseStamped": "std_msgs/Header header\nPose pose",
    "geometry_msgs/msg/Transform": "Vector3 translation\nQuaternion rotation",
    "geometry_msgs/msg/TransformStamped": "std_msgs/Header header\nstring child_frame_id\nTransform transform",
    "tf2_msgs/msg/TFMessage": "geometry_msgs/TransformStamped[] transforms",
    "sensor_msgs/msg/PointCloud2": "std_msgs/Header header\nuint32 height\nuint32 width\nPointField[] fields\n"
                                   "bool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense",
    "sensor_msgs/msg/PointField": "uint8 INT8 = 1\nuint8 UINT8 = 2\nuint8 INT16 = 3\nuint8 UINT16 = 4\nuint8 INT32 = 5\n"
                                  "uint8 UINT32 = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\nstring name\nuint32 offset\n"
                                  "uint8 datatype\nuint32 count",
    "sensor_msgs/msg/CompressedImage": "std_msgs/Header header\nstring format\nuint8[] data",
    "nav_msgs/msg/Path": "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",
    "visualization_msgs/msg/MarkerArray": "Marker[] markers",
    "visualization_msgs/msg/Marker": (
        "int32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\n"
        "int32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\n"
        "int32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n"
        "std_msgs/Header header\nstring ns\nint32 id\nint32 type\nint32 action\ngeometry_msgs/Pose pose\n"
        "geometry_msgs/Vector3 scale\nstd_msgs/ColorRGBA color\nbuiltin_interfaces/Duration lifetime\nbool frame_locked\n"
        "geometry_msgs/Point[] points\nstd_msgs/ColorRGBA[] colors\nstring texture_resource\n"
        "sensor_msgs/CompressedImage texture\nUVCoordinate[] uv_coordinates\nstring text\nstring mesh_resource\n"
        "MeshFile mesh_file\nbool mesh_use_embedded_materials"),
    "visualization_msgs/msg/UVCoordinate": "float32 u\nfloat32 v",
    "visualization_msgs/msg/MeshFile": "string filename\nuint8[] data",
}
DEPS = {  # 依存する型（rosbag2 と同じく、深さ優先で最初に現れた順）
    "tf2_msgs/msg/TFMessage": ["geometry_msgs/msg/TransformStamped", "std_msgs/msg/Header", "builtin_interfaces/msg/Time",
                               "geometry_msgs/msg/Transform", "geometry_msgs/msg/Vector3", "geometry_msgs/msg/Quaternion"],
    "sensor_msgs/msg/PointCloud2": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "sensor_msgs/msg/PointField"],
    "nav_msgs/msg/Path": ["std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/PoseStamped",
                          "geometry_msgs/msg/Pose", "geometry_msgs/msg/Point", "geometry_msgs/msg/Quaternion"],
    "visualization_msgs/msg/MarkerArray": [
        "visualization_msgs/msg/Marker", "std_msgs/msg/Header", "builtin_interfaces/msg/Time", "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Point", "geometry_msgs/msg/Quaternion", "geometry_msgs/msg/Vector3", "std_msgs/msg/ColorRGBA",
        "builtin_interfaces/msg/Duration", "sensor_msgs/msg/CompressedImage", "visualization_msgs/msg/UVCoordinate",
        "visualization_msgs/msg/MeshFile"],
}


def full_definition(t):
    """MCAP のスキーマ（ros2msg）: 本体の定義に、依存する型の定義を区切り線付きで続ける。"""
    s = MSGDEFS[t]
    for d in DEPS[t]:
        s += "\n" + "=" * 80 + "\nMSG: " + d + "\n" + MSGDEFS[d]
    return s


# ------------------------------------------------------------------ CDR（リトルエンディアン）
class Cdr:
    def __init__(self):
        self.b = bytearray(b"\x00\x01\x00\x00")  # カプセル化ヘッダ（CDR_LE）

    def _align(self, n):
        pad = (-(len(self.b) - 4)) % n  # 位置はヘッダの後ろから数える
        self.b += b"\x00" * pad

    def u8(self, v):
        self.b += struct.pack("<B", v)

    def boolean(self, v):
        self.u8(1 if v else 0)

    def i32(self, v):
        self._align(4)
        self.b += struct.pack("<i", v)

    def u32(self, v):
        self._align(4)
        self.b += struct.pack("<I", v)

    def f32(self, v):
        self._align(4)
        self.b += struct.pack("<f", v)

    def f64(self, v):
        self._align(8)
        self.b += struct.pack("<d", v)

    def string(self, s):
        e = s.encode() + b"\x00"
        self.u32(len(e))
        self.b += e

    def u8seq(self, data):
        self.u32(len(data))
        self.b += data

    def time(self, ns):
        self.i32(ns // 1_000_000_000)
        self.u32(ns % 1_000_000_000)

    def header(self, ns, frame):
        self.time(ns)
        self.string(frame)

    def bytes(self):
        return bytes(self.b)


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def ser_tf(ns, parent, child, x, y, z, yaw):
    c = Cdr()
    c.u32(1)
    c.header(ns, parent)
    c.string(child)
    for v in (x, y, z, *quat_yaw(yaw)):
        c.f64(v)
    return c.bytes()


def ser_cloud(ns, frame, xyz, intensity):
    n = len(xyz)
    pts = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    pts["x"], pts["y"], pts["z"], pts["intensity"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], intensity
    c = Cdr()
    c.header(ns, frame)
    c.u32(1)       # height
    c.u32(n)       # width
    c.u32(4)       # fields
    for i, name in enumerate(("x", "y", "z", "intensity")):
        c.string(name)
        c.u32(4 * i)
        c.u8(7)    # FLOAT32
        c.u32(1)
    c.boolean(False)
    c.u32(16)
    c.u32(16 * n)
    c.u8seq(pts.tobytes())
    c.boolean(True)
    return c.bytes()


def ser_path(ns, frame, poses):
    c = Cdr()
    c.header(ns, frame)
    c.u32(len(poses))
    for (x, y, z, yaw) in poses:
        c.header(ns, frame)
        for v in (x, y, z, *quat_yaw(yaw)):
            c.f64(v)
    return c.bytes()


ARROW, LINE_STRIP, POINTS = 0, 4, 8
ADD, DELETE = 0, 2


def marker(c, ns, frame, mns, mid, mtype=POINTS, action=ADD, pose=(0, 0, 0, 0), scale=(1, 1, 1),
           color=(1, 1, 1, 1), lifetime=0.0, frame_locked=False, points=None):
    c.header(ns, frame)
    c.string(mns)
    c.i32(mid)
    c.i32(mtype)
    c.i32(action)
    x, y, z, yaw = pose
    for v in (x, y, z, *quat_yaw(yaw)):
        c.f64(v)
    for v in scale:
        c.f64(v)
    for v in color:
        c.f32(v)
    c.i32(int(lifetime))
    c.u32(int(round((lifetime - int(lifetime)) * 1e9)))
    c.boolean(frame_locked)
    pts = np.zeros((0, 3)) if points is None else np.asarray(points, dtype="<f8")
    c.u32(len(pts))
    if len(pts):
        c._align(8)
        c.b += np.ascontiguousarray(pts, dtype="<f8").tobytes()
    c.u32(0)            # colors
    c.string("")        # texture_resource
    c.header(0, "")     # texture.header
    c.string("")        # texture.format
    c.u8seq(b"")        # texture.data
    c.u32(0)            # uv_coordinates
    c.string("")        # text
    c.string("")        # mesh_resource
    c.string("")        # mesh_file.filename
    c.u8seq(b"")        # mesh_file.data
    c.boolean(False)    # mesh_use_embedded_materials


def ser_markers(items):
    c = Cdr()
    c.u32(len(items))
    for kw in items:
        marker(c, **kw)
    return c.bytes()


# ------------------------------------------------------------------ MCAP（チャンク・索引・統計つき。圧縮はしない）
MAGIC = b"\x89MCAP0\r\n"


def rec(op, body):
    return struct.pack("<BQ", op, len(body)) + body


def s_str(s):
    e = s.encode()
    return struct.pack("<I", len(e)) + e


def s_map(d):
    body = b"".join(s_str(k) + s_str(v) for k, v in d.items())
    return struct.pack("<I", len(body)) + body


class McapWriter:
    def __init__(self, path, chunk_size=4 << 20):
        self.f = open(path, "wb")
        self.chunk_size = chunk_size
        self.f.write(MAGIC)
        self.f.write(rec(0x01, s_str("ros2") + s_str("gnss_and_lidar_localization tools/tile_demo/make_bag.py")))
        self.schemas, self.channels = [], []
        self.chunk_indexes = []
        self.counts = {}
        self.start = self.end = None
        self.n = 0
        self._reset_chunk()

    def _reset_chunk(self):
        self.buf = bytearray()
        self.idx = {}
        self.c_start = self.c_end = None

    def schema(self, name):
        sid = len(self.schemas) + 1
        r = rec(0x03, struct.pack("<H", sid) + s_str(name) + s_str("ros2msg") + struct.pack("<I", len(full_definition(name).encode()))
                + full_definition(name).encode())
        self.schemas.append(r)
        self.f.write(r)
        return sid

    def channel(self, sid, topic, qos):
        cid = len(self.channels) + 1
        r = rec(0x04, struct.pack("<HH", cid, sid) + s_str(topic) + s_str("cdr") + s_map({"offered_qos_profiles": qos}))
        self.channels.append(r)
        self.f.write(r)
        self.counts[cid] = 0
        return cid

    def message(self, cid, t, data):
        off = len(self.buf)
        self.buf += rec(0x05, struct.pack("<HIQQ", cid, self.counts[cid], t, t) + data)
        self.idx.setdefault(cid, []).append((t, off))
        self.counts[cid] += 1
        self.n += 1
        self.c_start = t if self.c_start is None else min(self.c_start, t)
        self.c_end = t if self.c_end is None else max(self.c_end, t)
        self.start = t if self.start is None else min(self.start, t)
        self.end = t if self.end is None else max(self.end, t)
        if len(self.buf) >= self.chunk_size:
            self._flush()

    def _flush(self):
        if not self.buf:
            return
        records = bytes(self.buf)
        body = (struct.pack("<QQQI", self.c_start, self.c_end, len(records), zlib.crc32(records)) + s_str("")
                + struct.pack("<Q", len(records)) + records)
        chunk = rec(0x06, body)
        chunk_start = self.f.tell()
        self.f.write(chunk)
        idx_start = self.f.tell()
        offsets = {}
        for cid in sorted(self.idx):
            offsets[cid] = self.f.tell()
            entries = b"".join(struct.pack("<QQ", t, o) for t, o in self.idx[cid])
            self.f.write(rec(0x07, struct.pack("<H", cid) + struct.pack("<I", len(entries)) + entries))
        idx_len = self.f.tell() - idx_start
        omap = b"".join(struct.pack("<HQ", k, v) for k, v in offsets.items())
        self.chunk_indexes.append(rec(0x08, struct.pack("<QQQQ", self.c_start, self.c_end, chunk_start, len(chunk))
                                      + struct.pack("<I", len(omap)) + omap + struct.pack("<Q", idx_len) + s_str("")
                                      + struct.pack("<QQ", len(records), len(records))))
        self._reset_chunk()

    def close(self):
        self._flush()
        self.f.write(rec(0x0F, struct.pack("<I", 0)))  # Data End（CRC は計算しない = 0）
        summary_start = self.f.tell()
        groups = []

        def group(op, records):
            if records:
                s = self.f.tell()
                for r in records:
                    self.f.write(r)
                groups.append((op, s, self.f.tell() - s))

        group(0x03, self.schemas)
        group(0x04, self.channels)
        cm = b"".join(struct.pack("<HQ", k, v) for k, v in sorted(self.counts.items()))
        stats = rec(0x0B, struct.pack("<QHIIII", self.n, len(self.schemas), len(self.channels), 0, 0, len(self.chunk_indexes))
                    + struct.pack("<QQ", self.start or 0, self.end or 0) + struct.pack("<I", len(cm)) + cm)
        group(0x0B, [stats])
        group(0x08, self.chunk_indexes)
        offset_start = self.f.tell()
        for op, s, ln in groups:
            self.f.write(rec(0x0E, struct.pack("<BQQ", op, s, ln)))
        self.f.write(rec(0x02, struct.pack("<QQI", summary_start, offset_start, 0)))
        self.f.write(MAGIC)
        self.f.close()


def qos_yaml(transient_local, depth):
    """rosbag2（Jazzy）の offered_qos_profiles の書き方。"""
    def zero():  # 期限なし（既定値）。別々の dict にして、YAML のアンカー（&id001）が出ないようにする
        return {"sec": 0, "nsec": 0}

    return yaml.safe_dump([{
        "history": "keep_last", "depth": depth, "reliability": "reliable",
        "durability": "transient_local" if transient_local else "volatile",
        "deadline": zero(), "lifespan": zero(), "liveliness": "automatic", "liveliness_lease_duration": zero(),
        "avoid_ros_namespace_conventions": False}], sort_keys=False)


# ------------------------------------------------------------------ 地図
def read_tile(path):
    b = pathlib.Path(path).read_bytes()
    if b[:8] not in (b"TPCMTIL1", b"GLLTILE1"):  # GLLTILE1 は名前を変える前の形式
        raise ValueError(f"{path}: not a tiled_pcd_map tile file")
    n, _flags = struct.unpack("<QI", b[8:20])
    return np.frombuffer(b[20:20 + 12 * n], dtype="<f4").reshape(-1, 3).astype(np.float64)


def voxel_first(p, res):
    """ボクセル（一辺 res）ごとに最初の点を残す（表示用の間引き）。"""
    if res <= 0 or len(p) == 0:
        return p
    k = np.floor(p / res).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return p[np.sort(i)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tile_dir", type=pathlib.Path)
    ap.add_argument("frames", type=pathlib.Path)
    ap.add_argument("out", type=pathlib.Path)
    ap.add_argument("--map-res", type=float, default=0.5, help="地図の全体を間引く大きさ [m]")
    ap.add_argument("--tile-res", type=float, default=0.5, help="読み込み中のタイルの表示を間引く大きさ [m]")
    ap.add_argument("--start-time", type=float, default=1767225600.0, help="bag の最初の時刻（UNIX 時刻 [s]）")
    a = ap.parse_args()

    index = yaml.safe_load((a.tile_dir / "tile_index.yaml").read_text())
    rec_ = json.loads(a.frames.read_text())
    frames = rec_["frames"]
    if len(frames) < 2:
        raise SystemExit("frames.json has too few frames")
    dt_rec = frames[1]["t"] - frames[0]["t"]
    if dt_rec > 0.15:
        print(f"note: frames are recorded every {dt_rec:.1f} s; use tiled_pcd_map_demo --record-interval 0.1 for smooth tf")
    # frames の tiles は、demo の記録の tiles（= 索引の順）の添字
    ts = float(index.get("tile_size", 20))
    tiles_meta = index["tiles"]
    assert len(tiles_meta) == len(rec_["tiles"]), "frames.json was made from a different tile index"

    # タイルごとの点（地図座標）、地面の高さ（点の z の 5% 点）
    pts, ground = [], []
    for t in tiles_meta:
        p = read_tile(a.tile_dir / t["file"])
        pts.append(p)
        ground.append(float(np.percentile(p[:, 2], 5)) if len(p) else float("nan"))
    ground = np.array(ground)
    key = {(t["ix"], t["iy"]): i for i, t in enumerate(tiles_meta)}

    def ground_at(x, y):
        i = key.get((math.floor(x / ts), math.floor(y / ts)))
        return ground[i] if i is not None and not math.isnan(ground[i]) else float("nan")

    # 自己位置（z は足元のタイルの地面の高さをならしたもの、yaw もならす）
    xs = np.array([f["x"] for f in frames])
    ys = np.array([f["y"] for f in frames])
    zs = np.array([ground_at(x, y) for x, y in zip(xs, ys)])
    good = ~np.isnan(zs)
    zs = np.interp(np.arange(len(zs)), np.flatnonzero(good), zs[good]) if good.any() else np.zeros(len(zs))
    k = 15
    zs = np.convolve(np.pad(zs, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), mode="valid")
    yaws = np.unwrap([f["yaw"] for f in frames])
    k = 3
    yaws = np.convolve(np.pad(yaws, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), mode="valid")

    t0 = int(round(a.start_time * 1e9))

    def ns(t):
        return t0 + int(round(t * 1e9))

    w = McapWriter(a.out)
    s_tf = w.schema("tf2_msgs/msg/TFMessage")
    s_pc = w.schema("sensor_msgs/msg/PointCloud2")
    s_path = w.schema("nav_msgs/msg/Path")
    s_mk = w.schema("visualization_msgs/msg/MarkerArray")
    c_tf = w.channel(s_tf, "/tf", qos_yaml(False, 100))
    c_map = w.channel(s_pc, "/tile_demo/map", qos_yaml(True, 1))
    c_route = w.channel(s_path, "/tile_demo/route", qos_yaml(True, 1))
    c_veh = w.channel(s_mk, "/tile_demo/vehicle", qos_yaml(True, 1))
    c_tiles = w.channel(s_mk, "/tile_demo/tiles", qos_yaml(False, 100))

    # 最初に 1 回だけ出すもの
    T0 = ns(frames[0]["t"])
    allp = np.concatenate([voxel_first(p, a.map_res) for p in pts if len(p)])
    allg = np.concatenate([np.full(len(voxel_first(p, a.map_res)), g) for p, g in zip(pts, ground) if len(p)])
    w.message(c_map, T0, ser_cloud(T0, "map", allp, (allp[:, 2] - allg).astype(np.float32)))
    step = max(1, int(round(1.0 / max(dt_rec, 1e-3))))
    route = [(xs[i], ys[i], zs[i], yaws[i]) for i in range(0, len(frames), step)]
    w.message(c_route, T0, ser_path(T0, "map", route))
    L, U = rec_["load_radius"], rec_["unload_radius"]
    ahead = rec_["lookahead_time"] * rec_["speed"]

    def circle(r, cx=0.0, n=120):
        return [(cx + r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), 0.3) for i in range(n + 1)]

    veh = [
        dict(ns=0, frame="base_link", mns="vehicle", mid=0, mtype=ARROW, pose=(-2.0, 0, 1.0, 0), scale=(4.5, 1.8, 1.8),
             color=(1.0, 0.71, 0.28, 1.0), frame_locked=True),
        dict(ns=0, frame="base_link", mns="load_radius", mid=0, mtype=LINE_STRIP, scale=(0.5, 0, 0),
             color=(0.27, 0.85, 0.54, 1.0), frame_locked=True, points=circle(L)),
        dict(ns=0, frame="base_link", mns="load_radius_ahead", mid=0, mtype=LINE_STRIP, scale=(0.25, 0, 0),
             color=(0.27, 0.85, 0.54, 0.5), frame_locked=True, points=circle(L, ahead)),
        dict(ns=0, frame="base_link", mns="unload_radius", mid=0, mtype=LINE_STRIP, scale=(0.4, 0, 0),
             color=(0.6, 0.6, 0.6, 1.0), frame_locked=True, points=circle(U)),
    ]
    w.message(c_veh, T0, ser_markers(veh))

    # タイルの表示用の点と枠
    GREEN, RED = (0.27, 0.85, 0.54, 1.0), (1.0, 0.48, 0.4, 1.0)
    disp = {}

    def tile_markers(i, tns):
        if i not in disp:
            disp[i] = voxel_first(pts[i], a.tile_res)
        t = tiles_meta[i]
        x0, y0 = t["ix"] * ts, t["iy"] * ts
        z = ground[i] if not math.isnan(ground[i]) else 0.0
        box = [(x0, y0, z), (x0 + ts, y0, z), (x0 + ts, y0 + ts, z), (x0, y0 + ts, z), (x0, y0, z)]
        return [dict(ns=tns, frame="map", mns="points", mid=i, mtype=POINTS, scale=(0.35, 0.35, 0), color=GREEN,
                     points=disp[i]),
                dict(ns=tns, frame="map", mns="outline", mid=i, mtype=LINE_STRIP, scale=(0.3, 0, 0), color=GREEN,
                     points=box)], box

    prev = set()
    n_add = n_del = 0
    for j, f in enumerate(frames):
        tns = ns(f["t"])
        w.message(c_tf, tns, ser_tf(tns, "map", "base_link", xs[j], ys[j], zs[j], yaws[j]))
        cur = set(f["tiles"])
        if cur != prev:
            items = []
            for i in sorted(cur - prev):
                items += tile_markers(i, tns)[0]
                n_add += 1
            for i in sorted(prev - cur):
                _, box = tile_markers(i, tns)
                items += [dict(ns=tns, frame="map", mns="points", mid=i, action=DELETE),
                          dict(ns=tns, frame="map", mns="outline", mid=i, action=DELETE),
                          dict(ns=tns, frame="map", mns="dropped", mid=i, mtype=LINE_STRIP, scale=(0.5, 0, 0),
                               color=RED, lifetime=1.5, points=box)]
                n_del += 1
            w.message(c_tiles, tns, ser_markers(items))
            prev = cur
    w.close()
    dur = frames[-1]["t"] - frames[0]["t"]
    print(f"{len(frames)} frames ({dur:.0f} s), map {len(allp)} points ({a.map_res} m), "
          f"tiles added {n_add} / removed {n_del}, {w.n} messages, {a.out.stat().st_size / 1e6:.1f} MB -> {a.out}")


if __name__ == "__main__":
    main()
