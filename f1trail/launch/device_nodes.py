from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
        ]),
        launch_arguments={'camera_model': 'zed'}.items()
    )

    urg_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('urg_node'), 'launch', 'urg_node_launch.py'])
        ]),
        launch_arguments={'ip_address': '192.168.0.10',
        'laser_frame_id': 'lidar_link'}.items()
    )

    return LaunchDescription([
        zed_camera,
        urg_lidar
    ])