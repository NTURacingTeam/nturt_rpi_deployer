#!/bin/bash

set -e

# declare text styles
COLOR_REST='\033[0m'
HIGHLIGHT='\033[0;1m'
COLOR_RED='\033[1;31m'

configure_startup() {
    # copy launch script for custom service "nturt_ros"
    cp -f scripts/nturt_ros /etc/init.d/
    # update systemctl daemon to make change into effect
    systemctl daemon-reload
    # register "nturt_ros" service to load at startup
    update-rc.d nturt_ros defaults
}

# check for root permission
if ! [ $(id -u) = 0 ]; then
    echo -e "${COLOR_RED}Error: ${HIGHLIGHT}The script need to be run with root permission.${COLOR_REST}" >&2
    exit 1
fi

# cehck for depnedencies
# check wiringpi
gpio readall &>/dev/null
if [[ ! $? == 0 ]]; then
    echo -en "${COLOR_RED}Error: ${HIGHLIGHT}Wiringpi is not installed, " >&2
    echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
    exit 1
# check bcm2835
elif [[ ! -a /usr/local/include/bcm2835.h ]]; then
    echo -en "${COLOR_RED}Error: ${HIGHLIGHT}C library for bcm2835 is not installed, " >&2
    echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
    exit 1
# check real-time priority
elif [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
    echo -en "${COLOR_RED}Error: ${HIGHLIGHT}Permission for real-time priority not set, " >&2
    echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
# check memlock priority
elif [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
    echo -en "${COLOR_RED}Error: ${HIGHLIGHT}Permission for memlock not set, " >&2
    echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
# check ros2 build tools
elif [[ -z "$(which colcon)" ]]; then
    echo -en "${COLOR_RED}Error: ${HIGHLIGHT}ROS2 build tools are not installed, " >&2
    echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
    exit 1
fi
