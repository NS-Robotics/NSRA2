#! /bin/bash

git pull origin main
cd robot_ws/
colcon build
source install/setup.bash
