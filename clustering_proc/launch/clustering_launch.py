from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('clustering_proc'), 'config', 'clustering_node.yaml')

    clustering = Node(
        package='clustering_proc',
        executable='clustering',
        name='scan_clustering',
        parameters=[config]
    )

    ld.add_action(clustering)
    
    return ld