from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def setup_launch(context, *args, **kwargs):
    config = PathJoinSubstitution([FindPackageShare('aruco_loc'), 'config', 'detection_node.yaml'])

    # define launch configuration to store the transform argument for the static_transform node
    # LaunchConfiguration substitutions allow us to acquire the value of the launch argument in any part of the launch description.
    transform_config = LaunchConfiguration('transform')

    detector = Node(
        package='aruco_loc',
        executable='detection_node',
        name='target_detector',
        parameters=[config]
    )

    # transform_config argument will be returned as a string here
    tf_args = transform_config.perform(context).split() + ['camera_link', 'base_link']
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_static_transform_publisher',
        arguments = tf_args
    )

    return [detector, static_transform]

def generate_launch_description():
    # define launch argument that can be passed form the console (or above launch files)
    transform_config_launch_arg = DeclareLaunchArgument(
        'transform',
        default_value='0 0 0 0 0 0',
        description='Static transform from camera_link to base_link. Format: x y z roll pitch yaw'
    )
    return LaunchDescription([
        transform_config_launch_arg,
        OpaqueFunction(function=setup_launch)
    ])
