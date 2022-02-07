#! /bin/bash

git pull origin main --recurse-submodules
git submodule update --remote
cd robot_ws/
colcon build
source install/setup.bash