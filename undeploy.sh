#!/bin/bash

# declare text styles
COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_RED='\e[1;31m'
COLOR_YELLOW='\e[1;93m'

print_usage() {
    echo "Undeploy NTURT ROS2 control system"
    echo "Note: This script needs to run with root permission"
    echo ""
    echo "Usage: sudo ./undeploy.sh [OPTIONS]"
    echo "Options:"
    echo "    -h --help       Print this help message and exit"
}

# check for root permission
if ! [ $(id -u) = 0 ]; then
    echo -e "${COLOR_RED}Error: ${HIGHLIGHT}The script need to be run with root permission.${COLOR_REST}" >&2
    exit 1
fi

# remove "nturt_ros" from service
service nturt_ros stop
rm /etc/init.d/nturt_ros
systemctl daemon-reload

# stop "nturt_ros" to run at start up
update-rc.d nturt_ros remove

# remove named pipe
rm nturt_ros_pipe
