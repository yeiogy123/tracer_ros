#!/bin/bash

# enable kernel module: gs_usb
sudo modprobe gs_usb

# install can utils
sudo apt install -y can-utils

# udev rule for bring up can interface
sudo cp tracer.rules
sudo cp `rospack find tracer_bringup`/scripts/tracer.rules /etc/udev/rules.d

sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger