#!/bin/bash
cd /workspaces/isaac_ros-dev/ros_ws
git add .
git commit -m "Automatic backup on $(date)"
git push origin main
