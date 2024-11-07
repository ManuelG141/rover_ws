#!/bin/bash

#Following the documentation on https://github.com/ros2/ros1_bridge/blob/master/src/simple_bridge.cpp
#Setting up ro1_bridge on bash script

#Saving the arguments given while executing the script
ROS_HOSTNAME=$1 #The numbers indicate the position of the given argument
ROS_MASTER_URI=$2
ROS1_INSTALL_PATH=$3
ROS2_INSTALL_PATH=$4

echo "ROS1_INSTALL_PATH=$ROS1_INSTALL_PATH"
echo "ROS2_INSTALL_PATH=$ROS2_INSTALL_PATH"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"


cd $HOME/ros2_humble

#Build ROS2 enviroment skipping ros1_bridge package
colcon build --symlink-install --packages-skip ros1_bridge

#Sourcing ROS1 and ROS2
source $ROS1_INSTALL_PATH/setup.bash
source $ROS2_INSTALL_PATH/setup.bash

#Build ros1_bridge package
colcon build --symlink-install --packages-select ros1_bridge

#Sourcing againg ROS1 and ROS2 setup.bash scripts
source $ROS1_INSTALL_PATH/setup.bash
source $ROS2_INSTALL_PATH/setup.bash

#Export ROS_HOSTNAME and ROS_MASTER_URI so it can works from others machines
export ROS_HOSTNAME=$ROS_HOSTNAME
export ROS_MASTER_URI=$ROS_MASTER_URI

#Executing ROS1 roscore
gnome-terminal -- bash -c "echo 'Running roscore.bash'; $HOME/rover_ws/bash_scripts/roscore.bash "$ROS_HOSTNAME" "$ROS_MASTER_URI"; exec bash"

#Run ros1_bridge
ros2 run ros1_bridge dynamic_bridge __log_disable_rosout:=true