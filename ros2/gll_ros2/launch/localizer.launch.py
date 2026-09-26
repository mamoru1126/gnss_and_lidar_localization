"""gll_localizer を起動する。

例:
  ros2 launch gll_ros2 localizer.launch.py \
      imu_topic:=/sensing/imu odom_topic:=/sensing/odom gnss_topic:=/sensing/gnss/fix

LiDAR と地図を使う場合（Phase 2）:
  ros2 launch gll_ros2 localizer.launch.py points_topic:=/sensing/lidar/points map_config:=/data/maps/maps.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params = [
        LaunchConfiguration("params_file").perform(context),
        {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"},
    ]
    # map_config を指定したときだけ、パラメータファイルの map.config_path を上書きする
    map_config = LaunchConfiguration("map_config").perform(context)
    if map_config:
        params.append({"map.config_path": map_config})
    node = Node(
        package="gll_ros2",
        executable="localizer_node",
        name="gll_localizer",
        output="screen",
        parameters=params,
        remappings=[
            ("~/input/imu", LaunchConfiguration("imu_topic")),
            ("~/input/odom", LaunchConfiguration("odom_topic")),
            ("~/input/gnss/fix", LaunchConfiguration("gnss_topic")),
            ("~/input/gnss/velocity", LaunchConfiguration("gnss_velocity_topic")),
            ("~/input/initial_pose", LaunchConfiguration("initial_pose_topic")),
            ("~/input/points", LaunchConfiguration("points_topic")),
        ],
    )
    return [node]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("gll_ros2"), "config", "localizer.yaml")
    args = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("imu_topic", default_value="/sensing/imu"),
        DeclareLaunchArgument("odom_topic", default_value="/sensing/odom"),
        DeclareLaunchArgument("gnss_topic", default_value="/sensing/gnss/fix"),
        DeclareLaunchArgument("gnss_velocity_topic", default_value="/sensing/gnss/velocity"),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("points_topic", default_value="/sensing/lidar/points"),
        DeclareLaunchArgument("map_config", default_value="", description="maps.yaml（空ならパラメータファイルの値）"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
