#!/bin/bash
set -e
. /opt/ros/humble/setup.sh
# Build 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release