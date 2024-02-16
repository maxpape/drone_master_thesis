#!/bin/bash


git submodule update --init --recursive
#source /opt/ros/noetic/setup.bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

#source ./PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

cd PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo-classic

cd ./..



pip install -e ./acados/interfaces/acados_template
pip install PyYAML
pip install rospkg

cd acados
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
make install -j4

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$(pwd)/acados/lib"
#export ACADOS_SOURCE_DIR="$(pwd)/acados"
