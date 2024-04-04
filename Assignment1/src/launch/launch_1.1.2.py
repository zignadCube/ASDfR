import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        'cam2image.yaml'
        )
        
    cam2image_node = Node(
        package = 'image_tools_sdfr',
        name = 'cam2image',
        executable = 'cam2image',
        parameters = [config]
        )
    
    brightness_node = Node(
        package = 'camera_processing',
        name = 'brightness_node',
        executable = 'brightness_node'
    )
    
    return LaunchDescription([
        cam2image_node,
        brightness_node,
    ])