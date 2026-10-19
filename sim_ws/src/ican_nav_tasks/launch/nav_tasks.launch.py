"""导航任务启动：速度监控 + 顺序导航（识别位 → 进入位 → 返航停车）。

通常配合 ican_example_navigation 的 navigation.launch.py（提供 Nav2/AMCL）一起使用。

示例：
  ros2 launch ican_example_navigation navigation.launch.py rviz:=false
  ros2 launch ican_nav_tasks nav_tasks.launch.py
  ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true
  ros2 launch ican_nav_tasks nav_tasks.launch.py params_file:=/绝对路径/waypoints.yaml

参数集中在 config/waypoints.yaml（含两套位姿、识别等待、裁判话题）。
此处只做少量可覆盖项，避免同一参数在两个地方各写一遍。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ican_nav_tasks")
    default_params = os.path.join(share, "config", "waypoints.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="导航任务参数文件（含两套位姿）"),
        DeclareLaunchArgument("start_delay_sec", default_value="20.0",
                              description="启动后等待定位就绪的时间"),
        DeclareLaunchArgument("enable_shooting", default_value="false",
                              description="是否实弹射击（瞄准算法未接入前建议 false）"),
        DeclareLaunchArgument("recognize_wait_sec", default_value="8.0",
                              description="停在识别位等待话术的最长时间"),
    ]

    # 只覆盖这几个；其余（含 waypoints / standoff_waypoints）以参数文件为准
    mission_overrides = {
        "start_delay_sec": LaunchConfiguration("start_delay_sec"),
        "enable_shooting": LaunchConfiguration("enable_shooting"),
        "recognize_wait_sec": LaunchConfiguration("recognize_wait_sec"),
    }

    return LaunchDescription(
        args
        + [
            TimerAction(
                period=0.0,
                actions=[
                    Node(
                        package="ican_nav_tasks",
                        executable="speed_monitor_node",
                        name="speed_monitor_node",
                        output="screen",
                        parameters=[LaunchConfiguration("params_file")],
                    )
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="ican_nav_tasks",
                        executable="sequential_mission_node",
                        name="sequential_mission_node",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("params_file"),
                            mission_overrides,
                        ],
                    )
                ],
            ),
        ]
    )
