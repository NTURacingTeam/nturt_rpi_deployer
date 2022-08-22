#!/bin/bash

configure_startup() {
    # copy launch script for custom service "nturt_ros"
    sudo cp -f scripts/nturt_ros /etc/init.d/
    # update systemctl daemon to make change into effect
    sudo systemctl daemon-reload
    # register "nturt_ros" service to load at startup
    sudo update-rc.d nturt_ros defaults
}

get_pipe_line() {
    echo $(cat $1 | grep PIPE_DIRECTORY= | cut -d= -f2)
}

# cehck for depnedencies
# check wiringpi
dpkg -s wiringpi &>/dev/null
if [[ ! $? == 0 ]]; then
    echo "Wiringpi is not installed, please install it by executing 'install.sh' script first"
    exit 1
# check bcm2835
elif [[ ! -a /usr/local/include/bcm2835.h ]]; then
    echo "C library for bcm2835 is not installed, please install it by executing 'install.sh' script first"
    exit 1
# check docker
elif [[ -z "$(which docker)" ]]; then
    echo "Docker is not installed, please install it by executing 'install.sh' script first"
    exit 1
fi

# check if named pipe exist
if [[ ! -p "nturt_ros_pipe" ]]; then
    echo "Named pipe for executing command on host in docker cintainer doesnot exit, adding..."
    mkfifo nturt_ros_pipe
fi

# check if start up script is modified, does not exist or directory is changed
if [[ ! -a /etc/init.d/nturt_ros ]]; then
    echo "Start up file does not exist, copying and configuring..."
    configure_startup
elif [[ -n $(cmp /etc/init.d/nturt_ros scripts/nturt_ros) ]]; then
    echo "Start up file modified, copying and configuring..."
    configure_startup
elif [ "$(cat scripts/nturt_ros | grep NTURT_ROS_DIRECTORY= | cut -d= -f2)" != "$(pwd)" ]; then
    echo "Directory of this package has been changed, updating..."
    sed -i "/NTURT_ROS_DIRECTORY=/c\NTURT_ROS_DIRECTORY=\"$(pwd)\"" scripts/nturt_ros
    configure_startup
fi

# check if docker container exists
if [[ -z "$(docker ps -a | grep -w ros)" ]]; then
    PWD=$(pwd)
    echo "Docker container for ros does not exist, creating"
    # cloning docker environment for using ros on rpi
    cd
    echo "Building a custom docker image, this may take a while..."
    git clone https://github.com/NTURacingTeam/docker.git
    cd docker && ./build_image.sh ros_rpi
    echo "Creating a container named 'ros'"
    ./start_container.sh create ros ros_rpi

    # copy this package to docker package directory
    cd .. && sudo cp ${PWD} ~/docker/packages/ros/
fi
