#! /bin/bash

cd robot_ws/
colcon build --packages-select nssc_interface robot_emu robot_descriptions robot_interface robot_bringup robot_hardware rviz_visual_tools moveit_visual_tools robot_interface robot_viewer