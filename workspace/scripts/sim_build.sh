#!/bin/sh
set -e

if [ -n "$1" ]; then
    cd "$1"
fi

SYMLINK_FLAG=""

if [ "$2" = "--symlink-install" ]; then
    SYMLINK_FLAG="--symlink-install"
fi

colcon build ${SYMLINK_FLAG} \
    --packages-skip \
        px4_msgs \
        microxrcedds_agent
