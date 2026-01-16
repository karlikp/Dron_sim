from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import shutil

def xterm(cmd, title):
    return ExecuteProcess(
        cmd=['xterm', '-T', title, '-e', 'bash', '-lc', cmd],
        output='screen'
    )

def wrap_ros(cmd):
    return (
        'source /opt/ros/humble/setup.bash && '
        'source ~/ws/install/setup.bash && '
        f'{cmd}; '
        'exec bash'
    )

def respawn_loop(cmd, delay_sec=2):
    return (
        f'while true; do {cmd}; '
        f'echo "[respawn] restart za {delay_sec}s"; sleep {delay_sec}; '
        'done'
    )

def with_cd_models(cmd):
    return f'cd ~/ws/src/uav_vision/models && {cmd}'

def generate_launch_description():
    if shutil.which('xterm') is None:
        raise RuntimeError("Brak xterm. Zainstaluj: sudo apt-get install -y xterm")

    if not os.environ.get('DISPLAY'):
        raise RuntimeError("Brak DISPLAY (brak sesji X/GUI). Uzyj wariantu tmux zamiast xterm.")

    venv_python = '/home/karol/venv/hitnet_gpu/bin/python3'

    depth_cmd = wrap_ros(respawn_loop('ros2 run uav_vision depth_stop_node', 2))
    disp_cmd  = wrap_ros(respawn_loop(with_cd_models(
        f'{venv_python} -m uav_vision.disparity --ros-args -r __node:=disparity'
    ), 2))
    stop_cmd  = wrap_ros(respawn_loop('ros2 run uav_vision stop_controller_node', 2))
    det_cmd   = wrap_ros(respawn_loop(with_cd_models('ros2 run uav_vision uav_camera_det'), 2))

    return LaunchDescription([
        xterm(depth_cmd, 'depth_stop_node'),
        xterm(disp_cmd,  'disparity (venv)'),
        xterm(stop_cmd,  'stop_controller_node'),
        xterm(det_cmd,   'uav_camera_det'),
    ])
