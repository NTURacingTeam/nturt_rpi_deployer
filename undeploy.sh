#!/bin/bash

# remove "nturt_ros" from service
sudo service nturt_ros stop
sudo rm /etc/init.d/nturt_ros
sudo systemctl daemon-reload

# stop "nturt_ros" to run at start up
sudo update-rc.d nturt_ros remove

# remove named pipe
rm nturt_ros_pipe
