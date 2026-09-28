#!/usr/bin/env python3
"""grandtour_to_bag.py で作った bag を ROS 2 そのもの（rosbag2_py・rclpy）で読み、全メッセージを deserialize できるかを確かめる。

  python3 check_bag_ros2.py <bag.mcap>

CI の ros2 ジョブで、ROS 2 と colcon のワークスペースを source した状態で使う（run_tests.sh から呼ばれる）。
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

EXPECTED = {
    "/sensing/imu": "sensor_msgs/msg/Imu",
    "/sensing/odom": "nav_msgs/msg/Odometry",
    "/sensing/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/groundtruth/pose": "geometry_msgs/msg/PoseStamped",
}


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
if types != EXPECTED:
    fail(f"topics {types}")
counts = {k: 0 for k in EXPECTED}
last = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if t < last:
        fail(f"not time ordered at {topic}")
    last = t
    m = deserialize_message(data, get_message(types[topic]))
    counts[topic] += 1
    if topic == "/sensing/imu" and counts[topic] == 1:
        if m.header.frame_id != "base_link" or abs(m.linear_acceleration.z - 9.81) > 0.1:
            fail(f"imu {m.header.frame_id} {m.linear_acceleration}")
    if topic == "/sensing/lidar/points":
        names = [f.name for f in m.fields]
        if names != ["x", "y", "z", "intensity"] or m.width == 0 or m.header.frame_id != "livox_lidar":
            fail(f"points {names} {m.width} {m.header.frame_id}")
    if topic == "/tf_static":
        tr = m.transforms[0]
        if (tr.header.frame_id, tr.child_frame_id) != ("base_link", "livox_lidar"):
            fail(f"tf_static {tr.header.frame_id} -> {tr.child_frame_id}")
    if topic == "/sensing/odom" and (m.header.frame_id, m.child_frame_id) != ("odom", "base_link"):
        fail(f"odom frames {m.header.frame_id} {m.child_frame_id}")
    if topic == "/initialpose" and m.header.frame_id != "map":
        fail("initialpose frame")
print("counts:", counts)
if min(counts.values()) == 0:
    fail("a topic has no messages")
print("ros2 bag check passed")
