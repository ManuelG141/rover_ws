#!/bin/bash

# Prompt the user for input
read -p "Enter the name of the Wi-Fi interface (e.g., wlan0): " INTERFACE_NAME

# Retrieve the IP address of the specified Wi-Fi interface
WIFI_IP=$(ip addr show "$INTERFACE_NAME" | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)

#saving ROS1 and ROS2 paths 
ROS1_INSTALL_PATH="/opt/ros/noetic"
ROS2_INSTALL_PATH="$HOME/ros2_humble/install"

# Check if an IP address was found
if [ -z "$WIFI_IP" ]; then
    #Indicate if there's no ip from the given interface
    echo "No IP address found for the interface $INTERFACE_NAME. Please check the interface name and try again."
else

    echo "The IP address of the Wi-Fi interface $INTERFACE_NAME is: $WIFI_IP"

    #Creating the ROS_HOSTNAME and ROS_MASTER_URI varibles to share with other scripts
    ROS_HOSTNAME=$WIFI_IP
    ROS_MASTER_URI=http://$WIFI_IP:11311

    #Printing the variables on console
    echo "ROS_HOSTNAME=$ROS_HOSTNAME"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "ROS1_INSTALL_PATH=$ROS1_INSTALL_PATH"
    echo "ROS2_INSTALL_PATH=$ROS2_INSTALL_PATH"

    #Executing roscore
    gnome-terminal -- bash -c "echo 'Running bridge.bash'; $HOME/rover_ws/bash_scripts/bridge.bash "$ROS_HOSTNAME" "$ROS_MASTER_URI" "$ROS1_INSTALL_PATH" "$ROS2_INSTALL_PATH"; exec bash"
fi