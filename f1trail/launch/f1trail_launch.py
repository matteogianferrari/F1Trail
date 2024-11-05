from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():
    device_nodes_enable_config_arg = DeclareLaunchArgument(
        'device_nodes_enable', default_value=TextSubstitution(text='false'),
        description='Enables the launch of device nodes for camera and lidar if set to \"true\"'
    )

    dev_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('f1trail'), 'launch', 'device_nodes.py'])
            ]),
        condition=LaunchConfigurationEquals('device_nodes_enable', 'true')
    )

    sl_proc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sl_stereo_proc'), 'launch', 'sl_stereo_proc_launch.py'])
        ])
    )

    aruco_loc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('aruco_loc'), 'launch', 'aruco_loc_launch.py'])
        ]),
        launch_arguments={'transform': '0.27 0 -0.04 -1.5707963267948966 0 -1.5707963267948966'}
    )

    clustering_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('clustering_proc'), 'launch', 'clustering_launch.py'])
        ]),
        launch_arguments={'transform':'0.15 0 0 0 0 0'}.items()
    )

    tracking_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('tracking_proc'), 'launch', 'tracking_proc_launch.py'])
        ])
    )

    pid_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('pid_controller'), 'launch', 'longitudinal_pid_launch.py'])
        ])
    )

    dev_nodes_start_event = RegisterEventHandler(
        OnExecutionComplete(
            target_action=dev_nodes,
            on_completion=[sl_proc_node, aruco_loc_node, clustering_node, tracking_node]
        )
    )

    return LaunchDescription([
        device_nodes_enable_config_arg,
        dev_nodes_start_event,
        dev_nodes,
        sl_proc_node,
        aruco_loc_node,
        clustering_node,
        tracking_node,
        pid_node
    ])
