#!/usr/bin/env bash
set -e

export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=/home/karol/.Xauthority
export GZ_SIM_RENDER_ENGINE=ogre2
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix uav_sim)/share/uav_sim/models_sdf
export GZ_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins/gui:/usr/lib/x86_64-linux-gnu/gz-gui-7/plugins:$GZ_GUI_PLUGIN_PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo chown -R "$USER:$USER" ~/.gz
sudo chown -R karol:karol /home/karol/.gz
chmod -R u+rwX /home/karol/.gz
mkdir -p /home/karol/.gz/sim/log
QGC_CONFIG_DIR="$SCRIPT_DIR/../.qgc-config"
QGC_INI="$QGC_CONFIG_DIR/QGroundControl.ini"
QGC_TEMPLATE="$QGC_CONFIG_DIR/QGroundControl.ini.template"

if [ ! -f "$QGC_INI" ] && [ -f "$QGC_TEMPLATE" ]; then
    sed "s|/home/YOUR_USERNAME|$HOME|g" "$QGC_TEMPLATE" > "$QGC_INI"
    echo "[QGC] Config generated from template: $QGC_INI"
fi

colcon build