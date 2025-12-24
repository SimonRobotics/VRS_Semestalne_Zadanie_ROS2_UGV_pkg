from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():




    config = os.path.join(
        get_package_share_directory('your_package'),
        'config',
        'mavros_uart.yaml'
    )

    return LaunchDescription([
        Node(
            package='tank_control',
            executable='tank_control',
            name='tank_control',
        ),
        
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[config]
        )
    ])