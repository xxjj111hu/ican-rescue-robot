"""裁判通信监控节点启动（E-01 配套 / E-04 现场验证工具）。

用法：
  ros2 launch ican_referee referee_monitor.launch.py
  ros2 launch ican_referee referee_monitor.launch.py echo_payload:=false

Ctrl-C 退出时会打印本场「裁判接口小结」：
  - 每个话题收到多少条
  - 载荷格式是否符合官方表 1
  - 声明顺序是否正确（识别话术先于射击声明）
时间线同时写入 log_file，可直接作为验收证据。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("ican_referee")
    default_params = os.path.join(share, "config", "referee_params.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="监控节点参数文件"),
        DeclareLaunchArgument("echo_payload", default_value="true",
                              description="是否在终端回显每条裁判消息"),
        DeclareLaunchArgument(
            "log_file", default_value="~/.ros/ican_referee/timeline.txt",
            description="时间线落盘路径"),
    ]

    node = Node(
        package="ican_referee",
        executable="referee_monitor_node",
        name="referee_monitor_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "echo_payload": ParameterValue(
                    LaunchConfiguration("echo_payload"), value_type=bool),
                "log_file": ParameterValue(
                    LaunchConfiguration("log_file"), value_type=str),
            },
        ],
    )

    return LaunchDescription(args + [node])
