#!/bin/bash

# declare text styles
COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_RED='\e[1;31m'
COLOR_YELLOW='\e[1;93m'

print_usage() {
    echo "Check if installation/configuration for NTURT ROS2 control system is conplete"
    echo "If so, deploy the control system"
    echo "Note: This script needs to run with root permission"
    echo ""
    echo "Usage: sudo ./deploy.sh [OPTIONS]"
    echo "Options:"
    echo "    -h --help       Print this help message and exit"
    echo "    -r --real-time  Check for real-time configuration"    
    echo "    -s --skip-check Skip checking for installation/configuration"
}

configure_startup() {
    # copy launch script for custom service "nturt_ros"
    cp -f scripts/nturt_ros /etc/init.d/
    # update systemctl daemon to make change into effect
    systemctl daemon-reload
    # register "nturt_ros" service to load at startup
    update-rc.d nturt_ros defaults
}

# default argument/option
CHECK_REALTIME=false
SKIP_CHECK=false

PARAM=$(getopt -o hrs -l help,real-time,skip-check -n "$0" -- "$@")

eval set -- "${PARAM}"

while true; do
    case "$1" in
        -h|--help)
            print_usage
            exit
            shift
            ;;
        
        -r|--realtime)
            CHECK_REALTIME=true
            shift
            ;;
        -s|--skip-check)
            SKIP_CHECK=true
            shift
            ;;
        --)
            shift
            break
            ;;
    esac
done

if [ "${SKIP_CHECK}" = false ]; then
    echo "Checking for system installations/configurations..."
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
    fi

    # check bcm2835
    if [[ ! -a /usr/local/include/bcm2835.h ]]; then
        echo -en "${COLOR_RED}Error: ${HIGHLIGHT}C library for bcm2835 is not installed, " >&2
        echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
        exit 1
    fi

    # check real-time configuration
    if [ "${CONFIGURE_REALTIME}" = true ]; then
        # real-time priority
        elif [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
            echo -en "${COLOR_RED}Error: ${HIGHLIGHT}Permission for real-time priority not set, " >&2
            echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
        fi
        # memlock priority
        elif [[ -z "$(cat /etc/security/limits.conf | grep -e ${REAL_USER}.*rtprio)" ]]; then
            echo -en "${COLOR_RED}Error: ${HIGHLIGHT}Permission for memlock not set, " >&2
            echo -e "please install it by executing \"install.sh\" script first.${COLOR_REST}" >&2
        fi
    fi

    # check docker
    if [[ -z "$(which docker)" ]]; then
        echo "Docker is not installed, please install it by executing 'install.sh' script first"
        exit 1
    fi

    echo "Checking for installations/configurations finished"
fi

# check if named pipe exist
if [[ ! -p "nturt_ros_pipe" ]]; then
    echo "Named pipe for executing command on host in docker cintainer doesnot exit, adding..."
    mkfifo nturt_ros_pipe
fi

# check if the directory of the start up script is changed
if [ "$(cat scripts/nturt_ros | grep NTURT_ROS_DIRECTORY= | cut -d= -f2)" != "\"$(pwd)\"" ]; then
    echo "Directory of this package has been changed, updating..."
    sed -i "/NTURT_ROS_DIRECTORY=/c\NTURT_ROS_DIRECTORY=\"$(pwd)\"" scripts/nturt_ros
    configure_startup
fi

# check if start up script is modified, does not exist or directory is changed
if [[ ! -a /etc/init.d/nturt_ros ]]; then
    echo "Start up file does not exist, copying and configuring..."
    configure_startup
elif [[ -n $(cmp /etc/init.d/nturt_ros scripts/nturt_ros) ]]; then
    echo "Start up file modified, copying and configuring..."
    configure_startup
fi
