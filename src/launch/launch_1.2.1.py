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
    
    setpoint_generator_node = Node(
        package = 'sequence_controller',
        name = 'setpoint_generator',
        executable = 'setpoint_generator'
    )
    
    return LaunchDescription([
        cam2image_node,
        RELbot_node,
        setpoint_generator_node,
    ])