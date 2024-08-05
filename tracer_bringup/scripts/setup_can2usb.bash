#!/bin/bash

# enable kernel module: gs_usb
sudo modprobe gs_usb

# install can utils
sudo apt install -y can-utils

# udev rule for bring up can interface
sudo cp `rospack find tracer_bringup`/scripts/tracer.rules /etc/udev/rules.d

# Remap velodyne scan
sudo cp `rospack find tracer_bringup`/scripts/laserscan_nodelet.launch `rospack find velodyne_pointcloud`/launch/laserscan_nodelet.launch 

sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger