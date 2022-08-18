#!/bin/bash

# copy launch script for custom service "nturt_ros"
sudo cp -f scripts/nturt_ros /etc/init.d/
# register "nturt_ros" service to load at startup
sudo update-rc.d nturt_ros defaults
# update systemctl daemon to make it into effect
sudo systemctl daemon-reload

# if bcm2835 is not installed
if [[ ! -a /usr/local/include/bcm2835.h ]]; then
    echo "C library for bcm2835 is not installed, installing..."
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
    tar zxvf bcm2835-1.71.tar.gz 
    cd bcm2835-1.71/
    sudo ./configure && sudo make && sudo make check && sudo make install
    cd .. && rm -r bcm2835-1.71/
fi

# if wiringpi is not installed
dkpg -s wiringpi &>/dev/null
if [[ ! $? == 0 ]]; then
    echo "Wiringpi is not installed, installing..."
    sudo apt-get install wiringpi
fi

# if /boot/config.txt is not yet configured
if [[ -z $(cat /boot/config.txt | grep -m "can0") ]]; then
    echo "Boot config is not yet configured for can hat, please reboot to make it into effect"
    sudo echo "dtparam=spi=on" >> /boot/config.txt
    sudo echo "dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000" >> /boot/config.txt
fi

# if docker is not installed
if [[ -z "$(which docker)" ]]; then
    echo "Docker is not installed, installing..."
    # if get_docker.sh script does not exist
    if [[ ! -a "scripts/get_docker.sh" ]]; then
        echo "Script for getting docker does not exist, downloading..."
        curl -fsSL https://get.docker.com -o get_docker.sh
        chmod +x get_docker.sh
        mv get_docker.sh scripts/
    fi
    # executing the script to install docker
    sudo ./scripts/get_docker.sh

    # add the user to docker group
    sudo usermod -aG docker ${USER}

    # create a docker directory to put this package in
    mkdir -p ~/docker/packages/ros
    cd .. && cp -r nturt_deploy_to_rpi/ ~/docker/packages/ros
    
    # re-login and change directory to ${HOME}
    sudo su - ${USER}

    # cloning docker environment for using ros on rpi
    echo "Building a custom docker image, this may take a while..."
    git clone https://github.com/NTURacingTeam/docker.git
    cd docker && ./build_image.sh ros_rpi
    echo "Creating a container named 'ros'"
    ./start_container.sh create ros ros_rpi
fi
