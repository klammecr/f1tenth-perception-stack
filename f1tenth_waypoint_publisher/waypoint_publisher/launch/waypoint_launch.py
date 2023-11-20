import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    ld = LaunchDescription()

    # Whereto find config
    config = os.path.join(
        get_package_share_directory('waypoint_publisher'),
        'config',
        'parameters.yaml'
        )
    
    # Load the parameters specific to your ComposableNode
    with open(config, 'r') as f:
        params = yaml.safe_load(f)

    node = Node(
        package='waypoint_publisher',
        executable='waypoint_publisher_node.py',
        output='screen',
        parameters=[config])
    
    ld.add_action(node)
    return ld