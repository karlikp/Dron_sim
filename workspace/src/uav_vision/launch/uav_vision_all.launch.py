from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():

    venv_python = '/home/karol/venv/hitnet_gpu/bin/python3'

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_stop_controller',
            default_value='true',
            description='Start node that can send PX4 HOLD/POSCTL commands on /obstacle_stop.',
        ),
        
        Node(
            package='uav_vision',
            executable='depth_stop_node',
            name='depth_stop_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_sim_time': True,
                'enable_px4_reaction': False,
                'enable_measurement_map_markers': True,
                'measurement_map_active': False,
                'measurement_map_active_topic': '/measurement_map_active',
                'measurement_gpos_source': 'px4',
                'measurement_gpos_topic': '/fmu/out/vehicle_global_position',
                'measurement_map_host': '127.0.0.1',
                'measurement_map_port': 8765,
            }],
        ),

        ExecuteProcess(
            cmd=[
                venv_python, '-m', 'uav_vision.disparity',
                '--ros-args', '-r', '__node:=disparity'
            ],
            output='screen'
        ),

        Node(
            package='uav_vision',
            executable='stop_controller_node',
            name='stop_controller_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_stop_controller')),
            parameters=[{
                'use_sim_time': True,
            }],
        ),
        Node(
            package='uav_vision',
            executable='uav_camera_det',
            name='uav_camera_det',
            output='log',
            arguments=['--ros-args', '--log-level', 'ERROR'],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
