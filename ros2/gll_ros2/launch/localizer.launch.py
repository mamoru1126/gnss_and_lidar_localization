"""gll_localizer を起動する。

例:
  ros2 launch gll_ros2 localizer.launch.py \
      imu_topic:=/sensing/imu odom_topic:=/sensing/odom gnss_topic:=/sensing/gnss/fix
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("gll_ros2"), "config", "localizer.yaml")
    args = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("imu_topic", default_value="/sensing/imu"),
        DeclareLaunchArgument("odom_topic", default_value="/sensing/odom"),
        DeclareLaunchArgument("gnss_topic", default_value="/sensing/gnss/fix"),
        DeclareLaunchArgument("gnss_velocity_topic", default_value="/sensing/gnss/velocity"),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    node = Node(
        package="gll_ros2",
        executable="localizer_node",
        name="gll_localizer",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("~/input/imu", LaunchConfiguration("imu_topic")),
            ("~/input/odom", LaunchConfiguration("odom_topic")),
            ("~/input/gnss/fix", LaunchConfiguration("gnss_topic")),
            ("~/input/gnss/velocity", LaunchConfiguration("gnss_velocity_topic")),
            ("~/input/initial_pose", LaunchConfiguration("initial_pose_topic")),
        ],
    )
    return LaunchDescription(args + [node])
