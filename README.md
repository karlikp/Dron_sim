# dron_sim
Simulation of UAV in Gazebo with ROS2, PX4 Autopilot and QGroundControl.

### Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** (recommended) with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

  - **Gazebo assets** saved to /home/"host"/.gz/fuel/fuel.gazebosim.org/openrobotics/models/
  "host" is your nickname

### Docker build
```bash
docker build -t dron_sim:latest . --build-arg USER_UID=$(id -u)
```

To set DOCKER_GPU_PARAM environment variable, run:
```bash
source ./scripts/set_GPU_param.sh
```

### Docker run
Running docker with GPU support requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), but if you don't have NVIDIA GPU, you can remove the `--gpus=all` flag and run this container only with CPU.
```bash
docker run --rm --net host --ipc host --gpus=all --privileged -it \
    -e DISPLAY=${DISPLAY} \
    -e ROS_DOMAIN_ID=0 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v=/tmp/.X11-unix:/tmp/.X11-unix \
    dron_sim:latest ros2 launch sim_bringup sim.launch.py
```

### Run for development
Running docker with GPU support requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), but if you don't have NVIDIA GPU, you can remove the `--gpus=all` flag in `devcontainer.json` and run this container only with CPU.

Click `Ctrl+Shift+P` and select `Dev Containers: Rebuild and Reopen in Container`. 
This will open the repository in the container and you can start developing.

To rebuild workspace use shortcut `Ctrl+Shift+B` in the vscode.

## Simulation Execution Guide
After run docker environment

**1. Launch simulator**
Open first terminal and write:
```bash
colcon build
source install/setup.bash
ros2 launch uav_bringup uav.launch.py
```

### Throubleshoting
If you will encounter a problem with "xcb" (GUI applications won't open) just run `xhost +local:docker` on your host machine.

`sudo apt install libignition-gazebo6-plugins`

## Acknowledgement
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/

Enable async CUDA allocator
export TF_GPU_ALLOCATOR=cuda_malloc_async
