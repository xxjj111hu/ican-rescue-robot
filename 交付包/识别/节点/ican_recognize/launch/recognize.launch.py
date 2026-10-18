from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ican_recognize",
            executable="recognize_node",
            name="ican_recognize_node",
            output="screen",
            parameters=[{
                "image_topic": "/camera/image",
                "detections_topic": "/referee/detections",
                "publish_debug_image": True,
            }],
        ),
    ])
