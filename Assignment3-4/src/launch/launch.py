import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    sequence_controller_node = Node(
        package = 'sequence_controller',
        name = 'sequence_controller',
        executable = 'sequence_controller'#,
        # parameters=[{"tau": 0.1}]
    )
    
    return LaunchDescription([
        sequence_controller_node,
    ])