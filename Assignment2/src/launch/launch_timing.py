import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # Uncomment and change if using a config file
    # config = os.path.join(
    #     'cam2image.yaml'
    #     )
        
    seq_node = Node(
        package = 'timing',
        name = 'seq_node',
        executable = 'seq_node',
        parameters = [{"history": "keep_last"}, {"depth": 10}]
        )
    
    loop_node = Node(
        package = 'timing',
        name = 'loop_node',
        executable = 'loop_node',
        parameters = [{"history": "keep_last"}, {"depth": 10}]
    )
    
    return LaunchDescription([
        seq_node,
        loop_node,
    ])