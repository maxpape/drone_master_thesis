#!/bin/bash
source /opt/ros/noetic/setup.bash
source ./PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$(pwd)/acados/lib"
export ACADOS_SOURCE_DIR="$(pwd)/acados"
