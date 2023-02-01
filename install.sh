#!/bin/bash

set -e

# declare text styles
COLOR_REST='\033[0m'
HIGHLIGHT='\033[0;1m'
COLOR_RED='\033[1;31m'

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

# if /boot/firmware/config.txt is not yet configured
if [[ -z $(cat /boot/frimware/config.txt | grep -w "can0") ]]; then
    echo "Boot config is not yet configured for can hat, please reboot afterwards to make it into effect."
    echo "dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000" >>/boot/config.txt
fi

# real-time permission
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

# if ros2 build tools are not installed
colcon -h &>/dev/null
if [[ ! $? == 0 ]]; then
    echo "ROS2 build tools are not installed, installing, this may take a while..."
    apt install ros-dev-tools
    echo 'source /opt/ros/humble/setup.bash' >>${REAL_USER_HOME}/.bashrc
    echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >>${REAL_USER_HOME}/.bashrc
    echo 'export _colcon_cd_root=/opt/ros/humble/' >>${REAL_USER_HOME}/.bashrc
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >>${REAL_USER_HOME}/.bashrc
fi

echo "Installation of depencies finished"
