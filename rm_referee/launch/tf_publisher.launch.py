from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_referee',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[
                {'robot_pos_topic': '/rm_referee/robot_pos'},
                {'ground_robot_position_topic': '/rm_referee/ground_robot_position'},
                {'root_frame_id': 'map'},
                {'child_frame_prefix': 'robot_'}
            ]
        )
    ])