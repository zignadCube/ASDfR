import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
        
    cam2image_node = Node(
        package = 'image_tools',
        name = 'cam2image',
        executable = 'cam2image'
        )
    
    object_position_node = Node(
        package = 'camera_processing',
        name = 'object_position',
        executable = 'object_position',
        parameters = [{"debug_object_position": True}]
    )
    
    sequence_controller_node = Node(
        package = 'sequence_controller',
        name = 'sequence_controller',
        executable = 'sequence_controller'#,
        # parameters=[{"tau": 0.1}]
    )
    
    return LaunchDescription([
        cam2image_node,
        light_position_node,
        sequence_controller_node,
    ])