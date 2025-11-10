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
    px4_path = Path(os.environ.get("PX4_PATH")) #os.environ to slownik zawirajacy wszystkie zmienne srodowiskowe

    px4_models_path = Path(px4_path, 'Tools', 'simulation', 'gz') # px4_path/Tools/simulatio/gz
    px4 = px4_path / "build" / "px4_sitl_default" / "bin" / "px4" #sitl = software in the loop, sposob symulowania Flight Controler 

    package_share_path = get_package_share_path('uav_sim') #znajduje zainstalowany pakiet ros2 w systemie
    world = package_share_path / "worlds" / "main_2.sdf"

    resource_paths = [                  #tworzy liste sciezek do katalogow z modelami 
        px4_models_path / "models",
        px4_path / "Tools" / "simulation" / "gz" / "models",
        package_share_path / "models"
    ]


#zwracana wartosc funkcji w ktorej jest tworzony obiekt LaunchDescription

    return LaunchDescription([                                        

        #deklaruje argument 'world' o domyślnej wartości world.as_posix(), gdzie as_posix() zamienia Path na string w stylu UNIX  
        DeclareLaunchArgument('world', default_value=world.as_posix()), 

        #GZ_SIM_RESOURCE_PATH informuje symulator gz gdzie szukać zasobow
        #  ':'.join() łączy wszystkie ścieżki w jeden długi string, oddzielony dwukropkiem
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(path.as_posix() for path in resource_paths)),

        #Ustawia globalny parametr 'use_sim_time' na wartość true
        SetParameter(name='use_sim_time', value=True),

        #pozwala uruchomić dowolny proces systemowy z poziomu pliku launch (tak jakbyś wpisał komende w terminalu)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
            output='screen',
        ),

        TimerAction(
        period=5.0,  # delay
            actions=[ #lista akcji które zostaną wykonane po opóźnieniu
                ExecuteProcess(
                    additional_env={
                        #określa numer konfiguracji startowej (airframe). 4001 to gotowy preset dla modelu drone x500
                        "PX4_SYS_AUTOSTART": "4001",
                        #nazwa modelu drona, którego symulacja ma być uruchomiona (musi odpowiadać nazwie modelu w plikach Gazebo)
                        "PX4_SIM_MODEL": "x500_oak",
                        #pozycja startowa drona w świecie gz: x,y,z,roll,pitch,yaw.
                        #wartość yaw jest przeliczana na radiany
                        "PX4_GZ_MODEL_POSE": f"57.4 40.95 0.17696 0 0 {math.radians(-180)}",
                    },
                    cmd=[
                        px4.as_posix(),
                    ],
                    output='screen',
                )
            ]
        ),

    
        #uruchomienie QGroundControl
        ExecuteProcess(
            cmd=[f'{Path.home().as_posix()}/QGroundControl-x86_64.AppImage'],
        ),

        #uruchomienie Micro XRCE-DDS Agent, potrzebny do komunikacji między PX4 a ROS2
        ExecuteProcess(name='uxrce_dds', cmd=['MicroXRCEAgent', 'udp4', '-p', '8888']),

        #uruchania węzeł ROS2 tworzący bridge pomiędzy gz (GZ Transport) a ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge', #program uruchamiany powyższego pakietu

            #lista mostów które węzeł ma utworzyć, '[' oznacza że wiadomość przekazywana jest do ros2
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
])
