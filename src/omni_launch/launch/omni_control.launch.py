from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nodes to be launched
        Node(
            package='omni_control_base',
            executable='box_publisher_node',
            name='box_publisher_node'
        ),
        Node(
            package='omni_control_base',
            executable='pid_control_node',
            name='pid_control_node'
        ),
        Node(
            package='omni_control_base',
            executable='motor_cmd_sender_node',
            name='motor_cmd_sender_node'
        ),
    ])
