from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/uav_geotag'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'gpos_topic',
            default_value='/fmu/out/vehicle_global_position',
        ),
        DeclareLaunchArgument('photo_interval_s', default_value='5.0'),

        Node(
            package='uav_geotag_recorder',
            executable='geotag_recorder',
            name='geotag_recorder',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'image_topic': LaunchConfiguration('image_topic'),
                'gpos_topic': LaunchConfiguration('gpos_topic'),
                'photo_interval_s': LaunchConfiguration('photo_interval_s'),
            }],
            output='screen',
        ),
    ])
