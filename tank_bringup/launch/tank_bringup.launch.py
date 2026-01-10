from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # config_cam = os.path.join(
    #     get_package_share_directory('tank_bringup'),
    #     'config',
    #     'camera_config.yaml'
    # )

    return LaunchDescription([
        Node(
            package='tank_control',
            executable='tank_control',
            namespace='tank_control',
            name='tank_control',
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
        )
    ])