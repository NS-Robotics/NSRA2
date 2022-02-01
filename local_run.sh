#! /bin/bash

git pull origin main
cd robot_ws/
colcon build --packages-select robot_emu robot_descriptions robot_bringup robot_hardware