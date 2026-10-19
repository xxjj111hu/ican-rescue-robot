"""仅启动速度监控节点（任务12 验证记录，可独立挂接）。

  ros2 launch ican_nav_tasks speed_monitor.launch.py
  ros2 launch ican_nav_tasks speed_monitor.launch.py report_file:=~/.ros/speed.txt
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration("topic")
    max_linear = LaunchConfiguration("max_linear_speed")
    report_file = LaunchConfiguration("report_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.5"),
            DeclareLaunchArgument(
                "report_file",
                default_value=os.path.expanduser(
                    "~/.ros/ican_nav_tasks/speed_report.txt"
                ),
                description="验证报告输出路径；置空则不写文件",
            ),
            Node(
                package="ican_nav_tasks",
                executable="speed_monitor_node",
                name="speed_monitor_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": topic,
                        "max_linear_speed": max_linear,
                        "report_file": report_file,
                    }
                ],
            ),
        ]
    )
