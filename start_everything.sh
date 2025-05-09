#!/bin/bash
# Start everything for Mars Rover Simulation (Clean Reset)

echo "Killing previous Gazebo and ROS 2 processes..."
pkill -9 gzserver
pkill -9 gzclient
pkill -f launch
# pkill -f spawn_entities
pkill -f motion_command
pkill -f obstacle_detector
sleep 1

echo "Activating Conda Environment..."
eval "$(conda shell.bash hook)"
conda activate ros_env || {
    echo "Failed to activate ros_env. Make sure it exists."
    exit 1
}

echo "Rebuilding ROS-GZ plugin workspace..."
cd ~/gz_bridge_ws
colcon build --packages-select ros_gz_sim --allow-overriding ros_gz_sim
source install/setup.bash

# Ensure Gazebo can find the system plugin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/gz_bridge_ws/install/ros_gz_sim/lib

echo "Rebuilding Mars Rover workspace..."
cd ~/ros2_mars_ws
colcon build --packages-select mars_rover_control
source install/setup.bash

echo "Setting up environment variables..."
export DISPLAY=172.28.32.1:0
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export IGN_GAZEBO_RESOURCE_PATH=$HOME/ros2_mars_ws/src/mars_rover_control/models

echo "Environment setup complete."

echo "Launching Gazebo world (reset)..."
ign gazebo $(ros2 pkg prefix mars_rover_control)/share/mars_rover_control/models/mars_world.sdf -r &

echo "Waiting for Gazebo to initialize..."
sleep 5

echo "Launching ROS 2 nodes (motion_command, obstacle_detector, bridge)..."
ros2 launch mars_rover_control rover_launch.py
