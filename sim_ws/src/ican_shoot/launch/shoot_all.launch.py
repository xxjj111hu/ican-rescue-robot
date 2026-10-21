"""一键启动两个射击节点（D-05 全流程联调用）。

为什么需要这个：四个任务点用到两套不同的瞄准逻辑
  ①环形靶  -> /ican_shoot/fire_ring    （ring_shooter_node，视觉检测十环靶纸）
  ②旋转靶  -> /ican_shoot/fire_wheel   （target_shooter_node，读轮子转角做时序）
  ③移动靶3 -> /ican_shoot/fire_moving_3
  ④移动靶4 -> /ican_shoot/fire_moving_4
所以两个节点都要在跑。

示例：
  # 只瞄准不实弹（默认，安全）
  ros2 launch ican_shoot shoot_all.launch.py

  # 实弹
  ros2 launch ican_shoot shoot_all.launch.py dry_run:=false

配合任务节点：
  ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("ican_shoot")
    ring_launch = os.path.join(share, "launch", "ring_shoot.launch.py")
    target_launch = os.path.join(share, "launch", "target_shoot.launch.py")

    args = [
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true=只瞄准不实弹（默认）；false=实弹"),
        DeclareLaunchArgument("rotate_target", default_value="3",
                              description="旋转靶目标编号 1~5"),
        DeclareLaunchArgument("moving_target", default_value="7",
                              description="移动靶目标编号 6/7/8"),
    ]

    ring = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ring_launch),
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
        }.items(),
    )
    target = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(target_launch),
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "rotate_target": LaunchConfiguration("rotate_target"),
            "moving_target": LaunchConfiguration("moving_target"),
        }.items(),
    )

    return LaunchDescription(args + [ring, target])
