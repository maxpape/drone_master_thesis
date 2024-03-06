FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

ADD . .
# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update -y && sudo apt upgrade -y

RUN sudo apt install wget
# Install Git
RUN sudo apt install -y git

RUN sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs libgstreamer-plugins-base1.0-dev

RUN sudo apt install -y python3-tk nano python3-catkin-tools
RUN sudo apt install -y libpcl1 ros-noetic-octomap-*

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

RUN sudo chmod +x ./install_geographiclib_datasets.sh  

RUN sudo ./install_geographiclib_datasets.sh

RUN echo "source /drone_master_thesis/ros/devel/setup.bash" >> ~/.bashrc
RUN echo "source /drone_master_thesis/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /drone_master_thesis/PX4-Autopilot /drone_master_thesis/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/drone_master_thesis/PX4-Autopilot" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/drone_master_thesis/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/drone_master_thesis/acados/lib " >> ~/.bashrc
RUN echo "export ACADOS_SOURCE_DIR=/drone_master_thesis/acados " >> ~/.bashrc
