#!/usr/bin/env python3
"""make_bag.py で作った bag を、ROS 2 そのもの（rosbag2_py・rclpy）で確かめる（CI の ros2 ジョブで使う）。

  python3 check_bag_ros2.py <bag.mcap>

1. rosbag2 で bag を開き、トピック・型・件数・QoS（durability）を読む。
2. 全メッセージを rclpy の型サポートで直列化の逆変換（deserialize）し、中身を確かめる。
3. ros2 bag play で再生し、購読して全メッセージが届くこと、transient local のトピックは
   再生の途中から購読しても届くことを確かめる。
"""
import subprocess
import sys
import time

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

EXPECTED = {
    "/tf": ("tf2_msgs/msg/TFMessage", "volatile"),
    "/tile_demo/map": ("sensor_msgs/msg/PointCloud2", "transient_local"),
    "/tile_demo/route": ("nav_msgs/msg/Path", "transient_local"),
    "/tile_demo/vehicle": ("visualization_msgs/msg/MarkerArray", "transient_local"),
    "/tile_demo/tiles": ("visualization_msgs/msg/MarkerArray", "volatile"),
}


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


def durability(profiles):
    """TopicMetadata.offered_qos_profiles（版によって文字列か QoS の一覧）から durability を読む。"""
    if isinstance(profiles, str):
        return "transient_local" if "transient_local" in profiles else "volatile"
    found = []
    for p in profiles:
        d = getattr(p, "durability", None)
        if callable(d):  # rosbag2_py（Jazzy）では rclcpp::QoS のバインディングで、durability() はメソッド
            d = d()
        s = str(d).upper()
        if "TRANSIENT_LOCAL" in s:
            return "transient_local"
        if "VOLATILE" in s:
            found.append("volatile")
            continue
        try:
            if int(d) == 1:  # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                return "transient_local"
            found.append("volatile")
        except (TypeError, ValueError):
            fail(f"cannot read durability from {d!r}")
    return "volatile" if found else "unknown"


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    if set(topics) != set(EXPECTED):
        fail(f"topics {sorted(topics)}")
    for name, (typ, dur) in EXPECTED.items():
        t = topics[name]
        if t.type != typ:
            fail(f"{name}: type {t.type}")
        got = durability(t.offered_qos_profiles)
        if got != dur:
            fail(f"{name}: durability {got} (offered_qos_profiles = {t.offered_qos_profiles!r})")
    counts = {n: 0 for n in EXPECTED}
    shown = set()
    while reader.has_next():
        name, data, stamp = reader.read_next()
        msg = deserialize_message(data, get_message(EXPECTED[name][0]))
        counts[name] += 1
        if name == "/tf":
            tr = msg.transforms[0]
            if tr.header.frame_id != "map" or tr.child_frame_id != "base_link":
                fail("tf frames")
            if tr.header.stamp.sec * 10**9 + tr.header.stamp.nanosec != stamp:
                fail("tf stamp != bag time")
        elif name == "/tile_demo/map":
            if [f.name for f in msg.fields] != ["x", "y", "z", "intensity"] or \
                    len(msg.data) != msg.width * msg.point_step or msg.width == 0:
                fail("map cloud layout")
        elif name == "/tile_demo/route":
            if len(msg.poses) < 2:
                fail("route")
        elif name == "/tile_demo/vehicle":
            if sorted(m.ns for m in msg.markers) != ["load_radius", "load_radius_ahead", "unload_radius", "vehicle"]:
                fail("vehicle markers")
        elif name == "/tile_demo/tiles":
            for m in msg.markers:
                if m.ns == "points":
                    if m.action == m.ADD:
                        if m.type != m.POINTS or not m.points:
                            fail("tile marker")
                        shown.add(m.id)
                    elif m.action == m.DELETE:
                        shown.discard(m.id)
    print("rosbag2 read + rclpy deserialize OK:", counts, f"tiles shown at the end: {len(shown)}")
    return counts


class Counter(Node):
    def __init__(self):
        super().__init__("tile_demo_bag_check")
        qos = QoSProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.counts = {n: 0 for n in EXPECTED}
        self.late = {}
        for name, (typ, _) in EXPECTED.items():
            self.create_subscription(get_message(typ), name, lambda m, n=name: self._inc(n), qos)

    def _inc(self, name):
        self.counts[name] += 1

    def subscribe_late(self):
        """再生の途中から transient local で購読する（最初に 1 回だけ出たメッセージが届くか）。"""
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, (typ, dur) in EXPECTED.items():
            if dur == "transient_local":
                self.late[name] = 0
                self.create_subscription(get_message(typ), name,
                                         lambda m, n=name: self.late.__setitem__(n, self.late[n] + 1), qos)


def play_and_listen(path, expected, rate):
    rclpy.init()
    node = Counter()
    play = subprocess.Popen(["ros2", "bag", "play", path, "--rate", str(rate), "--delay", "2",
                             "--disable-keyboard-controls"])
    start = time.time()
    subscribed_late = False
    end = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not subscribed_late and time.time() - start > 4.0 and play.poll() is None:
            node.subscribe_late()
            subscribed_late = True
        if play.poll() is not None and end is None:
            end = time.time()
        if end is not None and time.time() - end > 2.0:
            break
        if time.time() - start > 300:
            play.kill()
            fail("ros2 bag play did not finish")
    if play.returncode != 0:
        fail(f"ros2 bag play exited with {play.returncode}")
    print("received during play:", node.counts)
    print("late transient-local subscribers received:", node.late)
    node.destroy_node()
    rclpy.shutdown()
    for n, c in expected.items():
        if node.counts[n] != c:
            fail(f"{n}: received {node.counts[n]} of {c}")
    if not subscribed_late:
        fail("playback ended before the late subscription (use a longer bag or a lower rate)")
    for n, c in node.late.items():
        if c < 1:
            fail(f"{n}: late transient-local subscriber got nothing")


def main():
    path = sys.argv[1]
    expected = read_bag(path)
    play_and_listen(path, expected, rate=5.0)
    print("OK")


if __name__ == "__main__":
    main()
