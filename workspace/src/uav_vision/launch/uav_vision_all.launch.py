from launch import LaunchDescription
<<<<<<< HEAD
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    venv_python = '/home/karol/venv/hitnet_gpu/bin/python3'

    return LaunchDescription([
        
=======
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

VENV_PYTHON = '/home/karol/venv/hitnet_gpu/bin/python3'

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_stop_controller',
            default_value='true',
            description='Start node that can send PX4 HOLD/POSCTL commands on /obstacle_stop.',
        ),

>>>>>>> Singapore_model
        Node(
            package='uav_vision',
            executable='depth_stop_node',
            name='depth_stop_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
<<<<<<< HEAD
        ),

        ExecuteProcess(
            cmd=[
                venv_python, '-m', 'uav_vision.disparity',
                '--ros-args', '-r', '__node:=disparity'
            ],
            output='screen'
=======
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

        # disparity – needs TensorFlow/HitNet from the venv
        ExecuteProcess(
            cmd=[
                VENV_PYTHON, '-m', 'uav_vision.disparity',
                '--ros-args', '-r', '__node:=disparity'
            ],
            output='screen',
>>>>>>> Singapore_model
        ),

        Node(
            package='uav_vision',
            executable='stop_controller_node',
            name='stop_controller_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
<<<<<<< HEAD
        ),
        Node(
            package='uav_vision',
            executable='uav_camera_det',
            name='uav_camera_det',
            output='log',
            arguments=['--ros-args', '--log-level', 'ERROR'],
            respawn=True,
            respawn_delay=2.0,
=======
            condition=IfCondition(LaunchConfiguration('enable_stop_controller')),
            parameters=[{
                'use_sim_time': True,
            }],
        ),

        # uav_camera_det – needs torch/ultralytics from the venv
        ExecuteProcess(
            cmd=[
                VENV_PYTHON, '-m', 'uav_vision.uav_camera_det',
                '--ros-args', '-r', '__node:=uav_camera_det', '--log-level', 'ERROR'
            ],
            output='log',
>>>>>>> Singapore_model
        ),
    ])
