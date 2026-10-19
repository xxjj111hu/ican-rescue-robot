"""启动识别节点（含距离门控参数）。

常用覆盖：
  ros2 launch ican_recognize recognize.launch.py gate_enable:=false
  ros2 launch ican_recognize recognize.launch.py gate_min_dist:=0.60 gate_max_dist:=1.00
  ros2 launch ican_recognize recognize.launch.py mode:=classifier

模型路径请改 config/recognize_params.yaml，或用：
  ros2 run ican_recognize recognize_node --ros-args --params-file <你的yaml>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ican_recognize")
    default_params = os.path.join(share, "config", "recognize_params.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="参数文件路径"),
        DeclareLaunchArgument("mode", default_value="yolo",
                              description="yolo | classifier"),
        DeclareLaunchArgument("gate_enable", default_value="true",
                              description="是否启用距离门控"),
        DeclareLaunchArgument("gate_min_dist", default_value="0.60",
                              description="识别窗口近端（m）"),
        DeclareLaunchArgument("gate_max_dist", default_value="0.90",
                              description="识别窗口远端（m）"),
        DeclareLaunchArgument("detect_timeout_sec", default_value="5.0",
                              description="窗口内无发布多久判为识别失败"),
        DeclareLaunchArgument("publish_debug_image", default_value="false",
                              description="是否发布调试图（开启会显著拖慢主循环，实测 25Hz→7Hz）"),
        DeclareLaunchArgument("debug_every", default_value="10",
                              description="调试图每 N 帧发一次"),
    ]

    # 只覆盖这几个；model_path 等仍以参数文件为准，避免空串覆盖
    overrides = {
        "mode": LaunchConfiguration("mode"),
        "gate_enable": LaunchConfiguration("gate_enable"),
        "gate_min_dist": LaunchConfiguration("gate_min_dist"),
        "gate_max_dist": LaunchConfiguration("gate_max_dist"),
        "detect_timeout_sec": LaunchConfiguration("detect_timeout_sec"),
        "publish_debug_image": LaunchConfiguration("publish_debug_image"),
        "debug_every": LaunchConfiguration("debug_every"),
    }

    node = Node(
        package="ican_recognize",
        executable="recognize_node",
        name="ican_recognize_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
    )

    return LaunchDescription(args + [node])
