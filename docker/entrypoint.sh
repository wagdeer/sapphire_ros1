#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [[ -f /workspace/devel_ros1/setup.bash ]]; then
    source /workspace/devel_ros1/setup.bash
elif [[ -f /workspace/devel/setup.bash ]]; then
    source /workspace/devel/setup.bash
elif [[ -f /workspace/install_ros1/setup.bash ]]; then
    source /workspace/install_ros1/setup.bash
elif [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
fi

exec "$@"
