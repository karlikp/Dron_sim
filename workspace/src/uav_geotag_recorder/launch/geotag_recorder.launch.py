from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/uav_geotag'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('gpos_topic',  default_value='/fmu/out/vehicle_global_position'),
        DeclareLaunchArgument('save_every_n', default_value='5'),

        Node(
            package='uav_geotag_recorder',
            executable='geotag_recorder',
            name='geotag_recorder',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'image_topic': LaunchConfiguration('image_topic'),
                'gpos_topic': LaunchConfiguration('gpos_topic'),
                'save_every_n': LaunchConfiguration('save_every_n'),
            }],
            output='screen'
        )
    ])
