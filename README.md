# NTURT Rpi Deployer

## Introduction

It is alright to test ROS on desktop computers, but in order to set it up on the race car, or specifically raspberry pi, we need to setup a lot of things. This ROS package `nturt_rpi_deployer` provides some useful scripts that automate the setup process and make rpi launch ROS at start up.

Since ROS aren't natively support on Raspberry Pi OS, we utillize a docker virtual environment to contain ROS in a docker container.

## Usage

### Deploying ros to rpi

#### Install

On raspberry pi, run

```shell=
sudo ./install.sh [OPTIONS]
```

first to install needed dependencies.

#### Deploy

After that, run

```shell=
sudo ./deploy.sh [OPTIONS]
```

to deploy ROS on raspberry pi and run it at startup.

#### Undeploy

If you want to undeploy, run

```shell=
sudo ./undeploy.sh [OPTIONS]
```

to revert the changes done when deploying.

> Note: For more inofrmation about the usage of the scripts, use `-h` option to see the help message.

### system_stats_monitort_node

This package also provides utilities to monitor the usage of cpu, memory, swap and disk, as well as the temperature of cpu, the ssid of the connected wifi and its strength using ros message nturt_ros_interface/SystemStats every second.

Usage

```shell=
ros2 run nturt_rpi_deployer system_stats_monitort_node
```

> Note: Since only the wifi interface of rpi, `wlan0`, is known when writting the code, this node may not run at other platforms if the wifi interface is not `wlan0`.

## What will be done when deploying

### Install

First, when installing, the follow will be install:

1. WiringPi
2. bcm2835
3. docker

And the following changes will be made to the system:

1. /boot/configure.txt will be modified for can hat
2. Swap size will be increased to 1GB
3. Real-time permission will be configured if `-r` option is specified

### Deploy

Second, when deploying, installation will be checked first.
After that, the following changes will be made to deploy ros on raspberry pi:

1. Make a named pipe `nturt_ros_pipe`
2. Check if the service `nturt_ros` exist in `/etc/init.d/`, if not, copy the startup script in `scripts/nturt_ros` to `/etc/init.d/` and configure it to run at startup
3. Check if the startup script in script/nturt_ros is modified, if so, copy it to `/etc/init.d/nturt_ros` and reload `systemctl daemon` to make the changes into effect

### Undeploy

When undeploying, the followign changes will be made:

1. Remove the service `nturt_ros` so it will not run at start up
2. remove named pipe `nturt_ros_pipe`

## `nturt_ros` as a service that runs at start up

`nturt_ros` is copied to `/etc/init.d/` such that it is deemed as a service hence it can be controlled as

```shell=
sudo service nturt_ros \<COMMAND\>
```

whrere COMMAND can be `start`, `stop` or `restart`.

### start

When starting the service, it

1. runs docker container `ros2` if it is not running
2. launches ros using command
    ```shell=
    ros2 launch nturt_rpi_deployer nturt_ros.launch.py
    ```
3. runs `scripts/execute_loop.sh` that listenes to names pipe `nturt_ros_pipe` and executes text passed to it as shell commands.

### stop

When stoping the service, it

1. stop `ros2` docker container
2. stop `execute_loop.sh` by passing `stop`.

### restart

When restarting the service, it first stops then starts.
