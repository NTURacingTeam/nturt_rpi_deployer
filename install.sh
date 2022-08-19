#!/bin/bash

# if wiringpi is not installed
dpkg -s wiringpi &>/dev/null
if [[ ! $? == 0 ]]; then
    echo "Wiringpi is not installed, installing..."
    wget https://project-downloads.drogon.net/wiringpi-latest.deb
    sudo dpkg -i wiringpi-latest.deb
fi

# if bcm2835 is not installed
if [[ ! -a /usr/local/include/bcm2835.h ]]; then
    echo "C library for bcm2835 is not installed, installing..."
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
    tar zxvf bcm2835-1.71.tar.gz
    cd bcm2835-1.71/
    sudo ./configure && sudo make && sudo make check && sudo make install
    cd .. && rm -rf bcm2835-1.71/ && rm bcm2835-1.71.tar.gz
fi

# if /boot/config.txt is not yet configured
if [[ -z $(cat /boot/config.txt | grep -w "can0") ]]; then
    echo "Boot config is not yet configured for can hat, please reboot to make it into effect"
    echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
    echo "dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000" | sudo tee -a /boot/config.txt
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
    
    # re-login and change directory to ${HOME}
    sudo su - ${USER}
fi

echo "Installation of depency finished"
