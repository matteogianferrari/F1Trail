from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('tracking_proc'), 'config', 'tracking.yaml')

    node = Node(
        package='tracking_proc',
        executable='tracking',
        name='tracking_module',
        parameters=[config]
    )
    
    ld.add_action(node)
    
    return ld