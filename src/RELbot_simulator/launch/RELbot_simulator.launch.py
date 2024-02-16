from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, TimerAction

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
        # -------- Launched actions --------

    # Launch argument for Twist command mode
    use_twist_cmd_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_twist_cmd",
        default_value="false",
        description="Whether to use Twist command mode. If set to false, uses individual motor command mode.",
    )

    return LaunchDescription([
        use_twist_cmd_arg,
        Node(
            package='RELbot_simulator',
            # namespace='jiwy_simulator',
            executable='RELbot_simulator',
            name='RELbot_simulator',
                parameters=[
                    # Set the command mode based on the launch argument
                    {"use_twist_cmd": LaunchConfiguration(use_twist_cmd_arg.name)},
                ],
        ),
    ])