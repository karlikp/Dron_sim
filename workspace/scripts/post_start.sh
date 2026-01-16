#!/usr/bin/env bash
set -e

sudo chown -R karol:karol /home/karol/.gz
chmod -R u+rwX /home/karol/.gz
mkdir -p /home/karol/.gz/sim/log

#!/usr/bin/env bash
set -e

# Force host X11 display inside devcontainer
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=/home/karol/.Xauthority
export GZ_SIM_RENDER_ENGINE=ogre2



# export XDG_RUNTIME_DIR=/tmp/xdg-runtime-dir-1000
# mkdir -p "$XDG_RUNTIME_DIR"
# chmod 700 "$XDG_RUNTIME_DIR"

sudo chown -R "$USER:$USER" ~/.gz


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

#source $SCRIPT_DIR/../install/setup.bash # Point(.) is equel (source)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix uav_sim)/share/uav_sim/models_sdf

mkdir -p $HOME/.config/QGroundControl
chown -R karol:karol $HOME/.config/QGroundControl
chmod -R u+rwX $HOME/.config/QGroundControl
#chmod -R a+rw $SCRIPT_DIR/src/uav_sim/worlds || true



colcon build