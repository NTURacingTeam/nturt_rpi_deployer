# NTURT Rpi Deployer

## Introduction

It is alright to test ros on desktop computers, but in order to set it up on the race car, or specifically raspberry pi, we need to setup a lot of things. This ros package `nturt_rpi_deployer` provides some useful scripts that automate the setup process and make rpi launch ros at start up.

Since ros aren't natively support on Raspian, so we utillize a docker virtual environment to contain ros in a container.

## Usage

### Deploying ros to rpi

#### Install

On raspberry pi, run

```shell=
./install.sh
```

first to install needed libraries.

#### Deploy

After that, run

```shell=
./deploy.sh
```

to deploy it.

#### Undeploy

If you want to undeploy, run

```shell=
./undeploy.sh
```

to revert the changes done when deploying.

### ROS setup

ROS will be launched at start up using launch file in `launch/nturt_ros.launch`, so custom it to fit your need.

This ros package also provides testing python scripts `ros_test.py` in `scripts/ros_test.py` that blinks a led on gpio 11 which can be useful to find out weather the ros is launched. You can run it by

```shell=
roslaunch nturt_ros_deploy_to_rpi ros_test.py
```

It also contains some testing C++ executables in `test/` that will not be further documented here.

## What will be done when deploying

### Install

First, when installing, the follow will be install:

1. wiringpi
2. bcm2835
3. /boot/configure.txt will be modified for can hat
4. docker

### Deploy

Second, when deploying, the following changes will be made to deploy ros on rpi:

1. Make a named pipe `nturt_ros_pipe`
2. Check if the service `nturt_ros` exist in `/etc/init.d/`, if not, copy the startup script in `scripts/nturt_ros` to `/etc/init.d/` and configure it to run at startup
3. Check if the startup script in script/nturt_ros is modified, if so, copy it to `/etc/init.d/nturt_ros` and reload `systemctl daemon` to make the changes into effect
4. Build docker image `ros_rpi` from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) and create a container named `ros` based on that image if no container names `ros` exist
5. move the content of this package into `ros` container's source directory

### Undeploy

When undeploying, the followign changes will be made:

1. Remove the service `nturt_ros` so it will not run at start up
2. remove named pipe `nturt_ros_pipe`

## `nturt_ros` as a service that runs at start up

`nturt_ros` is copied to `/etc/init.d/` such that it is deemed as a service hence it can be controlled as

```shell=
sudo service nturt_ros COMMAND
```

whrere COMMAND can be `start`, `stop` or `restart`.

### start

When starting the service, it

1. runs docker container `ros` if it is not running
2. configures and activates can hat.
3. launches ros using launch file in `launch/`.
4. runs `execute_loop.sh` in `scripts/` that listenes to names pipe `nturt_ros_pipe` and executes text passed to it as shell commands.

### start

When stoping the service, it

1. kill ros
2. stop `execute_loop.sh` by passing `stop`.

### restart

When restarting the service, it first stops then starts.
