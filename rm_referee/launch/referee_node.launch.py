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
                    "enable_normal": False,                 # 启用电源管理常规链路
                    "enable_vt": True,                      # 启用图传链路
                    "new_vt": False,                        # 如果是VT03新图传，把这一项设置为True，串口波特率会调整为921600，否则为115200
                    "normal_tty_device": "/dev/ttyUSB0",    # 常规链路对应的串口设备
                    "vt_tty_device": "/dev/ttyUSB0",        # 图传链路对应的串口设备
                }],
            )
        ]
    )
