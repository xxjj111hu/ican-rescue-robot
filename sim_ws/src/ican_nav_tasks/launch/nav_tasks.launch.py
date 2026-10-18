"""导航任务启动：速度监控 + 顺序导航（1-4 -> 返航停车）。

通常配合 ican_example_navigation 的 navigation.launch.py（提供 Nav2/AMCL）一起使用。
示例：
  ros2 launch ican_example_navigation navigation.launch.py rviz:=false
  ros2 launch ican_nav_tasks nav_tasks.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_delay = LaunchConfiguration("start_delay_sec")
    enable_shooting = LaunchConfiguration("enable_shooting")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("enable_shooting", default_value="false"),
            TimerAction(
                period=0.0,
                actions=[
                    Node(
                        package="ican_nav_tasks",
                        executable="speed_monitor_node",
                        name="speed_monitor_node",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "topic": "/cmd_vel",
                                "max_linear_speed": 0.5,
                                "report_file": "/home/ican/Ican_fly/demo_shoot_ws/logs/speed_report.txt",
                            }
                        ],
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
                            {
                                "use_sim_time": True,
                                "start_delay_sec": start_delay,
                                "enable_shooting": enable_shooting,
                                "result_file": "/home/ican/Ican_fly/demo_shoot_ws/logs/mission_result.txt",
                            }
                        ],
                    )
                ],
            ),
        ]
    )
