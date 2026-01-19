#!/bin/bash

source /opt/ros/humble/setup.bash
source /robots_ws/install/setup.bash
source $(pwd)/install/setup.bash
# ros2 launch cv_drone launch_sim.py
ros2 launch cv_drone launch_complex.py