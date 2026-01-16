# UAV SAR Simulation Environment with AI-Based Perception
This project presents a simulation environment for supporting Search and Rescue (SAR) missions using Unmanned Aerial Vehicles (UAVs). The system integrates a realistic UAV simulation in Gazebo with PX4 SITL, ROS2-based system architecture, and AI-based perception modules for human detection and depth estimation.

The simulation environment enables testing and development of autonomous and semi-autonomous UAV functionalities in a safe and repeatable setup. Visual perception is implemented using deep learning models, including YOLOv8 for human detection and HITNet for stereo-based depth estimation. The generated depth maps are further used for basic obstacle avoidance logic.

Mission planning and monitoring are performed using QGroundControl, integrated with custom terrain maps hosted via online map services. Communication between PX4 and ROS2 is handled using Micro XRCE-DDS Agent, enabling direct integration without MAVLink bridges.

The project is fully containerized using Docker, ensuring reproducible builds and simplified deployment across different systems.

## Requirements
- **Docker** (Docker CLI recommended)
  - Add your user to the Docker group to grant the necessary permissions; otherwise, you must prefix commands with sudo.
  
- **Visual Studio Code** with the following extensions:
  - [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
 
- **GPU support requires**
  - [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Preparing the environment
 
### Docker build
```bash
docker build -t dron_sim:latest . --build-arg USER_UID=$(id -u)
```

To set DOCKER_GPU_PARAM environment variable, run:
```bash
source ./scripts/set_GPU_param.sh
```


### Python Virtual Environment Setup (venv)
  1. go to workspace folder:
  ```bash
  cd workspace
  ```

  2. Open VS code in the folder:
  ```bash
  code .
  ```

  3. Build container with .devcontainer:

  Use shortcut:
  ```bash
  Ctrl + Shift + P
  ```
  Then choose option:
  ```bash
  Dev Containers: Rebuild Container
  ```
  
  4. Change directory:
 
  ```bash
  cd ../venv
  ```
 
  5.  Create a virtual environment

  ```bash
  python3 -m venv hitnet_gpu
  ```

  6. Activate the virtual environment:
     
  ```bash
  source hitnet_gpu/bin/activate
  ```
  After activation, your shell prompt should show:
  ```bash
  (hitnet_gpu)
  ```

  7. Installing dependencies:
  ```bash
  pip install -r requirements.txt
  ```

  **(good practice) Using venv inside Docker / Devcontainer**
  
  When working in Docker or a VS Code devcontainer, it is recommended to keep the virtual environment outside the workspace directory to avoid build tools (e.g. colcon) scanning site-packages.

## Simulation Execution Guide
This section describes how to launch the simulation in Gazebo, visualize sensor data and execute the object detection and obstacle avoiding.


**1. Launch Gazebo Simulation**

Open a terminal and execute the following commands:
```bash
source install/setup.bash
ros2 launch uav_bringup uav.launch.py
```
This command starts the TurtleBot4 Lite model in the `office` world with the specified initial position.

<p align="center">
  <img src="images/world.png" alt="Gazebo simulator view">
  <br>
  <em>Figure 1: Gazebo simulator view</em>
</p>

<p align="center">
  <img src="images/QGC_init.png" alt="Gazebo simulator view">
  <br>
  <em>Figure 2: QGroundControl tool view</em>
</p>

**2. Launch vision system**

```bash
cd ~/ws
source ../venv/hitnet_gpu/bin/activate
source install/setup.bash
cd src/uav_vision
ros2 launch uav_vision uav_vision_all.launch.py
```

<p align="center">
  <img src="images/object_detection.png" alt="Screenshot showing people detection">
  <br>
  <em>Figure 3: Screenshot showing people detection</em>
</p>

<p align="center">
  <img src="images/disparity_depth.png" alt="Screenshot showing disparity and depth estimation based on stereo vision cameras">
  <br>
  <em>Figure 4: Screenshot showing disparity and depth estimation based on stereo vision cameras</em>
</p>

<p align="center">
  <img src="images/stop_before_obstacle_QGC.png" alt="Screenshot showing the UAV stopping by entering hold mode based on a depth estimation reading of less than 5 meters">
  <br>
  <em>Figure 5: Screenshot showing the UAV stopping by entering hold mode based on a depth estimation reading of less than 5 meters</em>
</p>

<p align="center">
  <img src="images/stop_before_obstacle.png" alt="Screenshot showing the UAV stopping in the Gazebo simulator">
  <br>
  <em>Figure 6: Screenshot showing the UAV stopping in the Gazebo simulator</em>
</p>

## Acknowledgement
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/
