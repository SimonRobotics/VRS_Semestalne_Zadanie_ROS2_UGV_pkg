from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tank_control',
            executable='tank_control',
            name='tank_control',
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
        ),

    ])