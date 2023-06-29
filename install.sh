#!/bin/bash

# declare text styles
COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_RED='\e[1;31m'
COLOR_YELLOW='\e[1;93m'

print_usage() {
    echo "Install dependencies and configure settings for NTURT ROS2 control system"
    echo "Note: This script needs to run with root permission"
    echo ""
    echo "Usage: sudo ./install.sh [OPTIONS]"
    echo "Options:"
    echo "    -h --help       Print this help message and exit"
    echo "    -r --real-time  Configure for real-time"
}

# default argument/option
CONFIGURE_REALTIME=false
NEED_TO_REBOOT=false

PARAM=$(getopt -o hr -l help,real-time -n "$0" -- "$@")

eval set -- "${PARAM}"

while true; do
    case "$1" in
        -h|--help)
            print_usage
            exit
            shift
            ;;
        -r|--realtime)
            CONFIGURE_REALTIME=true
            shift
            ;;
        --)
            shift
            break
            ;;
    esac
done

# check for root permission
if [[ ! $(id -u) == 0 ]]; then
    echo -e "${COLOR_RED}Error: ${HIGHLIGHT}The script need to be run with root permission.${COLOR_REST}" >&2
    exit 1
fi

# get the name of the user who invoke sudo
if [ $SUDO_USER ]; then
    REAL_USER=$SUDO_USER
else
    REAL_USER=$(whoami)
fi

# get the home directory of the user who invoke sudo
REAL_USER_HOME=$(awk -F: -v v="${REAL_USER}" '{if ($1==v) print $6}' /etc/passwd)

# if wiringpi is not installed
gpio readall &>/dev/null
if [[ ! $? == 0 ]]; then
    echo "Wiringpi is not installed, installing..."
    git clone https://github.com/WiringPi/WiringPi.git
    cd WiringPi
    ./build
    cd .. && rm -rf WiringPi
fi

# if bcm2835 is not installed
if [[ ! -a /usr/local/include/bcm2835.h ]]; then
    echo "C library for bcm2835 is not installed, installing..."
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
    tar zxvf bcm2835-1.71.tar.gz
    cd bcm2835-1.71/
    ./configure && make && make check && make install
    cd .. && rm -rf bcm2835-1.71/ && rm bcm2835-1.71.tar.gz
fi

# if /boot/config.txt is not yet configured
if [[ -z $(cat /boot/config.txt | grep -w "can0") ]]; then
    echo "dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000" >>/boot/config.txt
    NEED_TO_REBOOT=true
fi

# if real-time permission is not configured
if [ "${CONFIGURE_REALTIME}" = true ]; then
    # real-time priority
    if [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
        echo "No real-time priority permission set, setting to highest 98..."
        sed -i "/# End of file/i\\${REAL_USER} -       rtprio          98" /etc/security/limits.conf
    fi
    # memlock priority
    if [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
        echo "No memlock permission set, setting to highest 1GB..."
        sed -i "/# End of file/i\\${REAL_USER} -       memlock         1048576" /etc/security/limits.conf
    fi
fi

# if docker is not installed
if [[ -z "$(which docker)" ]]; then
    echo "Docker is not installed, installing..."
    curl -fsSL https://get.docker.com -o get_docker.sh
    chmod +x get_docker.sh
    ./get_docker.sh
    rm get_docker.sh

    # add the user to docker group
    usermod -aG docker ${USER}
    
    NEED_TO_REBOOT=true
fi

echo "Installation of dependencies finished"
if [ "${NEED_TO_REBOOT}" = true ]; then
    echo "Some installations/configurations need to reboot the system to take into effect, please rboot"
fi
