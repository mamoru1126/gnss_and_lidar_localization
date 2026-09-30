#!/usr/bin/env python3
"""awsim_to_bag.py で作った bag を ROS 2 そのもの（rosbag2_py・rclpy）で読み、全メッセージを deserialize できるかを確かめる。

  python3 check_bag_ros2.py <bag のフォルダ> [--no-gnss] [--initial-pose]
  python3 check_bag_ros2.py <replay.sh --record で録った bag> --recorded

bag のフォルダは metadata.yaml ごと rosbag2 で開く（metadata.yaml が正しいかも確かめる）。

CI の ros2 ジョブで、ROS 2 を source した状態で使う（run_localizer_check.sh から呼ばれる）。
"""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--no-gnss", action="store_true")
ap.add_argument("--initial-pose", action="store_true")
ap.add_argument("--recorded", action="store_true", help="replay.sh --record で録った bag（推定の様子のトピック）")
args = ap.parse_args()

EXPECTED = {
    "/sensing/imu": "sensor_msgs/msg/Imu",
    "/sensing/odom": "nav_msgs/msg/Odometry",
    "/sensing/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/groundtruth/pose": "geometry_msgs/msg/PoseStamped",
}
EXPECTED["/map/points"] = "sensor_msgs/msg/PointCloud2"  # --tiles で入る全体の地図
if not args.no_gnss:
    EXPECTED["/sensing/gnss/fix"] = "sensor_msgs/msg/NavSatFix"
if args.initial_pose:
    EXPECTED["/initialpose"] = "geometry_msgs/msg/PoseWithCovarianceStamped"
if args.recorded:
    # 録った bag: 推定の出力と、地図・スキャンの表示用の点群（入力の IMU・ODOM・生の点群は録らない）
    EXPECTED = {
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/map/points": "sensor_msgs/msg/PointCloud2",
        "/groundtruth/pose": "geometry_msgs/msg/PoseStamped",
        "/gll_localizer/output/pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
        "/gll_localizer/debug/lidar_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
        "/gll_localizer/debug/map_points": "sensor_msgs/msg/PointCloud2",
        "/gll_localizer/debug/scan_points": "sensor_msgs/msg/PointCloud2",
    }


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
if args.recorded:
    missing = {k: v for k, v in EXPECTED.items() if types.get(k) != v}
    if missing:
        fail(f"missing topics {missing}（録れたもの: {types}）")
elif types != EXPECTED:
    fail(f"topics {types}")
counts = {k: 0 for k in types}
status = set()
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
    if topic == "/sensing/gnss/fix":
        status.add(m.status.status)
        if m.header.frame_id != "gnss_link" or m.position_covariance_type != 2 or not 35 < m.latitude < 36:
            fail(f"gnss {m.header.frame_id} {m.position_covariance_type} {m.latitude}")
    if topic == "/sensing/lidar/points":
        names = [f.name for f in m.fields]
        if names[:3] != ["x", "y", "z"] or m.width == 0 or m.header.frame_id != "velodyne_top":
            fail(f"points {names} {m.width} {m.header.frame_id}")
    if topic == "/tf_static" and not args.recorded:
        pairs = [(tr.header.frame_id, tr.child_frame_id) for tr in m.transforms]
        if pairs != [("base_link", "velodyne_top"), ("map", "map_local")]:
            fail(f"tf_static {pairs}")
    if topic in ("/map/points", "/gll_localizer/debug/map_points", "/gll_localizer/debug/scan_points"):
        if m.header.frame_id != "map_local" or m.width == 0:
            fail(f"{topic}: frame {m.header.frame_id}, {m.width} points")
    if topic == "/sensing/odom" and (m.header.frame_id, m.child_frame_id) != ("odom", "base_link"):
        fail(f"odom frames {m.header.frame_id} {m.child_frame_id}")
    if topic == "/initialpose" and m.header.frame_id != "map":
        fail("initialpose frame")
print("counts:", counts, "gnss status:", sorted(status))
if min(counts[k] for k in EXPECTED) == 0:
    fail("a topic has no messages")
if not args.no_gnss and not args.recorded and status != {-1, 2}:
    fail(f"gnss status {status}（2 と、--gnss-off-time の -1 があるはず）")
print("ros2 bag check passed")
