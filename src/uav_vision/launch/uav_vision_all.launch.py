from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    venv_python = '/home/karol/venv/hitnet_gpu/bin/python3'

    return LaunchDescription([
        
        Node(
            package='uav_vision',
            executable='depth_stop_node',
            name='depth_stop_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        # Node(
        #     package='uav_vision',
        #     executable='disparity',
        #     name='disparity',
        #     output='screen',
        #     respawn=True,
        #     respawn_delay=2.0,
        # ),

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
        ),
        Node(
            package='uav_vision',
            executable='uav_camera_det',
            name='uav_camera_det',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
