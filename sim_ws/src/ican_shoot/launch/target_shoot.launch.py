"""旋转靶 / 移动靶射击节点启动（D-03、D-04）。

示例：
  # 只瞄准不实弹（默认，安全）
  ros2 launch ican_shoot target_shoot.launch.py

  # 实弹
  ros2 launch ican_shoot target_shoot.launch.py dry_run:=false

  # 指定旋转靶编号（识别节点没发 /ican_recognize/target 时用）
  ros2 launch ican_shoot target_shoot.launch.py rotate_target:=2

触发：
  ros2 service call /ican_shoot/fire_wheel    std_srvs/srv/Trigger   # D-03
  ros2 service call /ican_shoot/fire_moving_3 std_srvs/srv/Trigger   # D-04 ③点
  ros2 service call /ican_shoot/fire_moving_4 std_srvs/srv/Trigger   # D-04 ④点

前提：机器人已停在对应任务点、车头大致朝向靶子。
本节点会自动做车头精调（转 ±3.33° 对准叶片）。
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
    default_params = os.path.join(share, "config", "target_shoot_params.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="靶位射击节点参数文件"),
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true=只瞄准不实弹（默认）；false=实弹"),
        DeclareLaunchArgument("rotate_target", default_value="3",
                              description="旋转靶目标编号 1~5（识别节点会覆盖）"),
        DeclareLaunchArgument("moving_target", default_value="7",
                              description="移动靶目标编号 6/7/8（识别节点会覆盖）"),
        DeclareLaunchArgument("aim_timeout_sec", default_value="20.0",
                              description="单个靶位最长瞄准时间"),
    ]

    overrides = {
        "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
        "default_rotate_target": ParameterValue(
            LaunchConfiguration("rotate_target"), value_type=int),
        "default_moving_target": ParameterValue(
            LaunchConfiguration("moving_target"), value_type=int),
        "aim_timeout_sec": ParameterValue(
            LaunchConfiguration("aim_timeout_sec"), value_type=float),
    }

    node = Node(
        package="ican_shoot",
        executable="target_shooter_node",
        name="target_shooter_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
    )

    return LaunchDescription(args + [node])
