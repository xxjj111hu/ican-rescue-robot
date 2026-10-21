"""环形靶射击节点启动（D-01 瞄准 / D-02 环数）。

示例：
  # 只瞄准不实弹（默认，安全）
  ros2 launch ican_shoot ring_shoot.launch.py

  # 实弹
  ros2 launch ican_shoot ring_shoot.launch.py dry_run:=false

  # 同时开调试图（录制演示视频时用）
  ros2 launch ican_shoot ring_shoot.launch.py publish_debug_image:=true

前提：仿真 + Nav2 已启动，机器人已停在①号任务点、车头正对环形靶。
触发：
  ros2 service call /ican_shoot/fire_ring std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("ican_shoot")
    default_params = os.path.join(share, "config", "ring_shoot_params.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="射击节点参数文件"),
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true=只瞄准不实弹（默认）；false=实弹"),
        DeclareLaunchArgument("publish_debug_image", default_value="false",
                              description="是否发布调试图（录演示视频时开）"),
        DeclareLaunchArgument("tol_px", default_value="8.0",
                              description="瞄准容差（px），约 1 环 = 8.2 px"),
    ]

    # ParameterValue 强制类型：命令行传 "false"/"8" 时避免被推断成错误类型
    overrides = {
        "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
        "publish_debug_image": ParameterValue(
            LaunchConfiguration("publish_debug_image"), value_type=bool),
        "tol_px": ParameterValue(LaunchConfiguration("tol_px"), value_type=float),
    }

    node = Node(
        package="ican_shoot",
        executable="ring_shooter_node",
        name="ring_shooter_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
    )

    return LaunchDescription(args + [node])
