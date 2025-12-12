#!/usr/bin/env bash

#set -e

# colcon build
# source /home/karol/ws/install/setup.bash # Point(.) is equel (source)

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix uav_sim)/share/uav_sim/models_sdf
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ws/install/uav_sensors_gz/lib
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ws/install/uav_sim/share


# mkdir -p /home/karol/.config/QGroundControl
# chown -R karol:karol /home/karol/.config/QGroundControl
# chmod -R u+rwX /home/karol/.config/QGroundControl
# chmod -R a+rw /home/karol/ws/src/sim_bringup/worlds || true




