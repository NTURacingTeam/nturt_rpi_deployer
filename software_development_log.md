# National Taiwan University Racing Team Software Development Log
###### tags: `development_log` `NTURT`
##### Group: electrical system group
##### Person in charge: 羅紀翔
##### Authors: 羅紀翔
##### Subsystem: RPI
##### Subsystem number: RP2
##### Software name: rpi_deployer
##### Repository: [github](https://github.com/NTURacingTeam/nturt_rpi_deployer)
##### Started designing date: 8/17
##### Current version: 1.0
##### Last modified date: 8/28

---

## Engineering goal:

To avoid painful and repetitive processes of deploying ros to raspberry pi, this package automate it and even provide extra functionallities such as testing scripts to determine weather ros is running.

## Program structure:

### Deploying

In order to automate the process of deploying ros on raspberry pi, this program uses bash scripts to install, deploy and undeploy to raspberry pi. Please checkout `Usage` session in `README.md` for more information.

### Launch ros at start up

Please checkout `nturt_ros as a service that runs at start up` session in `README.md`.

### Named pipe

In order to execute host command in a docker container, a named pipe `nturt_ros_pipe` is created such that when passing shell command to it, a `execute_loop.sh` in `scripts/` will execute the command on the host meaching, making the container able to shutdown the host, etc.

`execute_loop.sh` can be stopped by passing `stop` to stop it.

## Included libraries:

- 

## Testing environment:

- bash 5.0.17(1)-release (x86_64-pc-linux-gnu)
- ros noetic
- docker virtual environment from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) with image `ros_matlab`, `ros_rpi` based on ubuntu20.04


##### Testing hardware:

- asus tuf gaming a15 FA506II-0031A 4800H
- raspberry pi 3B+

##### Operating system:

- ubuntu 20.04
- raspbian 32-bit

##### Compiler(intepreter) version:

- gcc 9.4.0 (Ubuntu 9.4.0-1ubuntu1~20.04.1)
- python 3.8.10

---

## Testing result of 1.0:

### Deploying to rpi

run 

```shell=
./install.sh
./deploy.sh
```

on a blank rpi, deployed successfully.

### Launch ros ar start up

ros launched successfully

## Todos in 1.0:
