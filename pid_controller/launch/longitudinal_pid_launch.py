from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('longitudinal_pid'), 'config', 'longitudinal_pid.yaml')

    longitudinal_pid = Node(
        package='longitudinal_pid',
        executable='longitudinal_pid_node',
        name='longitudinal_pid',
        parameters= [config]
    )
    
    ld.add_action(longitudinal_pid)
    
    return ld