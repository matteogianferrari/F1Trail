from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('aruco_loc'), 'config', 'detection_node.yaml')

    detector = Node(
        package='aruco_loc',
        executable='detection_node',
        name='target_detector',
        parameters=[config]
    )

    ld.add_action(detector)
    
    return ld
