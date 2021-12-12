#! /bin/bash

git pull origin main
cd robot_ws/
colcon build --packages-select camera_ingest