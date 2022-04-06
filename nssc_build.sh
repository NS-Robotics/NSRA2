#! /bin/bash

git pull
cd robot_ws/
colcon build --packages-select camera_ingest nssc_web_interface