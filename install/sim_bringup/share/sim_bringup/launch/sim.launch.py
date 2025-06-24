import os
import math
from pathlib import Path

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.actions import TimerAction


def generate_launch_description():
    px4_path = Path(os.environ.get("PX4_PATH"))

    px4_models_path = Path(px4_path, 'Tools', 'simulation', 'gz')
    px4 = px4_path / "build" / "px4_sitl_default" / "bin" / "px4"

    package_share_path = get_package_share_path('sim_bringup')
    world = package_share_path / "worlds" / "main_2.sdf"

    resource_paths = [
        px4_models_path / "models",
        px4_path / "Tools" / "simulation" / "gz" / "models",
        package_share_path / "models"
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world.as_posix()),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(path.as_posix() for path in resource_paths)),

        SetParameter(name='use_sim_time', value=True),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
            output='screen',
        ),

        TimerAction(
        period=10.0,  # delay
        actions=[
            ExecuteProcess(
                additional_env={
                    "PX4_SYS_AUTOSTART": "4001",
                    "PX4_SIM_MODEL": "x500_oak",
                    "PX4_GZ_MODEL_POSE": f"57.4 40.95 0.17696 0 0 {math.radians(-180)}",
                },
                cmd=[
                    px4.as_posix(),
                ],
                output='screen',
            )
        ]
    ),

    

        ExecuteProcess(
            cmd=[f'{Path.home().as_posix()}/QGroundControl.AppImage'],
        ),

        ExecuteProcess(name='uxrce_dds', cmd=['MicroXRCEAgent', 'udp4', '-p', '8888']),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            output='screen',
        ),
])
