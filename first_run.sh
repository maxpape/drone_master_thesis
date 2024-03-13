#!/bin/bash


git submodule update --init --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx


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

make install -j4

cd /drone_master_thesis/ros
catkin build open3d_slam -DCMAKE_BUILD_TYPE=Release
catkin build open3d_slam_ros -DCMAKE_BUILD_TYPE=Release
