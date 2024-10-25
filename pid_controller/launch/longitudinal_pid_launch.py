from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    longitudinal_pid = Node(
        package='longitudinal_pid',
        executable='longitudinal_pid_node',
        name='longitudinal_pid'
    )
    
    ld.add_action(longitudinal_pid)
    
    return ld