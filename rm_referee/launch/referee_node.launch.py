from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='rm_referee',
                executable='referee_node',
                output='screen',
                parameters=[{
                    "enable_normal": False,
                    "enable_vt": True,
                    "new_vt": False,
                    "normal_tty_device": "/dev/ttyUSB0",
                    "vt_tty_device": "/dev/ttyUSB0",
                }],
            )
        ]
    )
