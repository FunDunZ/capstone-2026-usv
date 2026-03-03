from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_autonomy',
            executable='detection_node',
            name='detection_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='usv_autonomy',
            executable='mission_node',
            name='mission_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
