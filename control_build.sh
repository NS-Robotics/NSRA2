#! /bin/bash

cd robot_ws/
colcon build --packages-select robot_emu robot_descriptions robot_bringup robot_hardware rviz_visual_tools moveit_visual_tools robot_interface --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1