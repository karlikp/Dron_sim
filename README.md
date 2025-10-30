# dron_sim
Simulation of UAV in Gazebo with ROS2, PX4 Autopilot and QGroundControl.

### Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** (recommended) with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

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


### Throubleshoting
If you will encounter a problem with "xcb" (GUI applications won't open) just run `xhost +local:docker` on your host machine.



sudo apt install libignition-gazebo6-plugins

### Useful statement

colcon build --symlink-install
source install/setup.bash