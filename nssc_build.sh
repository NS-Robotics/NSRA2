#! /bin/bash

git pull
cd robot_ws/
colcon build --packages-select nssc_interface camera_ingest nssc_web_interface