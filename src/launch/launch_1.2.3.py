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
    
    RELbot_node = Node(
        package = 'RELbot_simulator',
        name = 'RELbot_simulator',
        executable = 'RELbot_simulator'
    )
    
    light_position_node = Node(
        package = 'camera_processing',
        name = 'light_position',
        executable = 'light_position',
        parameters = [{"threshold": 250}, {"debug_light_position": True}, {"image_topic": "/output/moving_camera"}]
    )
    
    sequence_controller_node = Node(
        package = 'sequence_controller',
        name = 'sequence_controller_moving',
        executable = 'sequence_controller_moving'#,
        # parameters=[{"tau": 0.1}]
    )
    
    return LaunchDescription([
        cam2image_node,
        RELbot_node,
        light_position_node,
        sequence_controller_node,
    ])