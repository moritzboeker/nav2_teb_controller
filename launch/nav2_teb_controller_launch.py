"""Launch file."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    # Construct the path to the config files
    nav2_config = PathJoinSubstitution(
        [FindPackageShare('nav2_teb_controller'), 'config', 'teb_controller_params.yaml']
    )
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            # namespace='',
            output='screen',
            # autostart=True,
            parameters=[nav2_config],
            ros_arguments=['--log-level', 'controller_server:=debug'],
            # prefix=callgrind_prefix,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base_link',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_template_to_base_link',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ]
        ),
    ])
