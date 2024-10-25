from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('sl_stereo_proc'), 'config', 'split_stereo.yaml')

    node = Node(
        package='sl_stereo_proc',
        executable='split_stereo',
        name='zed_stereo_split',
        parameters=[config]
    )
    
    ld.add_action(node)
    
    return ld