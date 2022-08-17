#!/bin/bash

TO_RELOGIN=false
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
    # set to re-login
    TO_RELOGIN=true
fi

# copy launch script for custom service "nturt_ros"
sudo cp -f scripts/nturt_ros /etc/init.d/
# register "nturt_ros" service to load at startup
sudo update-rc.d nturt_ros defaults
# update systemctl daemon to make it into effect
sudo systemctl daemon-reload

# if have to re-login
if ${TO_RELOGIN} ; then
    sudo su - ${USER}
fi
