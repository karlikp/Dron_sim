import os
import platform
import subprocess
from pathlib import Path

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _get_multiarch():
    try:
        return subprocess.check_output(
            ['dpkg-architecture', '-qDEB_HOST_MULTIARCH'],
            stderr=subprocess.DEVNULL,
        ).decode().strip()
    except Exception:
        arch_map = {
            'x86_64': 'x86_64-linux-gnu',
            'aarch64': 'aarch64-linux-gnu',
            'armv7l': 'arm-linux-gnueabihf',
        }
        return arch_map.get(platform.machine(), f'{platform.machine()}-linux-gnu')


def generate_launch_description():
    px4_path = Path(os.environ.get("PX4_PATH"))

    px4_models_path = Path(px4_path, 'Tools', 'simulation', 'gz')
    px4 = px4_path / "build" / "px4_sitl_default" / "bin" / "px4"
    bringup_share_path = get_package_share_path('uav_bringup')
    package_share_path = get_package_share_path('uav_sim')
    world = package_share_path / "worlds" / "Singapore_river" / "model2.sdf"
    px4_params_path = bringup_share_path / "config"

    resource_paths = [
        px4_models_path / "models",
        px4_path / "Tools" / "simulation" / "gz" / "models",
        package_share_path / "models",
        package_share_path / "worlds",
    ]
    _multiarch = _get_multiarch()
    gz_sim_plugin_path = f'/usr/lib/{_multiarch}/gz-sim-7/plugins'
    gz_sim_gui_plugin_path = f'/usr/lib/{_multiarch}/gz-sim-7/plugins/gui'
    gz_gui_plugin_path = f'/usr/lib/{_multiarch}/gz-gui-7/plugins'
    
    gui_plugin_paths = ':'.join([
        gz_sim_gui_plugin_path,
        gz_gui_plugin_path,
    ])
    system_plugin_paths = gz_sim_plugin_path

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world.as_posix()),
        DeclareLaunchArgument('photo_interval_s', default_value='5.0'),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            ':'.join(path.as_posix() for path in resource_paths),
        ),
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', gui_plugin_paths),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugin_paths),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', system_plugin_paths),

        # NVIDIA dGPU via PRIME offload (works on Optimus laptops and in containers with --gpus=all).
        # Unset or override these if running on a non-NVIDIA system.
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),

        SetParameter(name='use_sim_time', value=True),

        # Physics server only – no Qt GUI dependency, won't crash on missing OpenGL context.
        ExecuteProcess(
            name='gz_server',
            cmd=['gz', 'sim', '-r', '-s', LaunchConfiguration('world')],
            output='screen',
        ),

        # GUI is optional; crash here does not stop the simulation.
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    name='gz_gui',
                    cmd=['gz', 'sim', '-g'],
                    output='screen',
                ),
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    additional_env={
                        "PX4_SYS_AUTOSTART": "4001",
                        "PX4_SIM_MODEL": "x500_oak",
                        "PX4_GZ_MODEL_NAME": "x500_oak_0",
                        "PATH": f"{px4_params_path.as_posix()}{os.pathsep}{os.environ.get('PATH', '')}",
                        #"PX4_GZ_MODEL_POSE": "0.805 2.041 1.740 -0.002 0.0 -3.141",
                    },
                    cmd=[
                        px4.as_posix(),
                    ],
                    output='screen',
                )
            ]
        ),

        ExecuteProcess(
            cmd=[
                (Path.home() / f'QGroundControl-{platform.machine()}.AppImage').as_posix(),
                '--config-dir',
                str(bringup_share_path.parent.parent.parent.parent / '.qgc-config'),
            ],
        ),

        # ROS2-PX4 communication
        ExecuteProcess(name='uxrce_dds', cmd=['MicroXRCEAgent', 'udp4', '-p', '8888']),

        # Launch node creating bridge between ROS2 and GZ.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            # Bridge list, '[' means that the message is forwarded to ROS2.
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_front/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera_front/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_front/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera_front/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/gimbal/roll_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/gimbal/pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/gimbal/yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='uav_geotag_recorder',
            executable='gimbal_photo_controller',
            name='gimbal_photo_controller',
            parameters=[{
                'use_sim_time': True,
                'roll_offset_rad': 0.0,
                'pitch_offset_rad': 0.0,
                'yaw_offset_rad': 0.0,
                'compensate_yaw': True,
                'command_rate_hz': 30.0,
            }],
            output='screen',
        ),

        Node(
            package='uav_geotag_recorder',
            executable='geotag_recorder',
            name='geotag_recorder',
            parameters=[{
                'use_sim_time': True,
                'output_dir': '/tmp/uav_geotag',
                'image_topic': '/camera/image_raw',
                'gpos_topic': '/fmu/out/vehicle_global_position',
                'photo_interval_s': LaunchConfiguration('photo_interval_s'),
            }],
            output='screen',
        ),
    ])
