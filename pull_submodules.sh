#! /bin/bash

git pull origin main --recurse-submodules
git submodule update --remote
rm -r robot_ws/src/image_transport/
cp -r image_common/image_transport/ robot_ws/src/image_transport/
cd robot_ws/
colcon build
source install/setup.bash