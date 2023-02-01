#!/bin/bash

set -e

# declare text styles
COLOR_REST='\033[0m'
HIGHLIGHT='\033[0;1m'
COLOR_RED='\033[1;31m'

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
