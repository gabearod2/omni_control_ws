from launch import LaunchDescription
from launch_ros.actions import Node


# TODO: Create launch file for object capture
# TODO: Create launch file for person following 
# TODO: Create launch file for Nav2 controlling

def generate_launch_description():
    return LaunchDescription([
        # Nodes to be launched
        # Box publisher is not necessary. Done by jetson inference.
        # Need to launch the jetson-inference node.
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
