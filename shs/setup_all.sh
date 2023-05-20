#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'

# get the shell type
str="${SHELL}" ; shtype=${str##*/};

# export Isaac Sim path
if [ -d "${HOME}"/.local/share/ov/pkg/isaac_sim-2022.2.0 ];then
	ISAACSIM_PATH=${HOME}/.local/share/ov/pkg/isaac_sim-2022.2.0
elif [ -d /isaac-sim ];then
	ISAACSIM_PATH=/isaac-sim
else echo -e "${FAILED}Please check your installation of Isaac Sim!${DEF}" && exit 0
fi
export ISAACSIM_PATH
export ISAACSIM_PYTHON_EXE=${ISAACSIM_PATH}/python.sh

# export Python package path
if [ "$(grep docker < /proc/1/cgroup)" == "" ]; then
	export PYTHONPATH=${PYTHONPATH}:${PWD}/ros/src/arx5_pickplace/arx5_vision/scripts:${PWD}/ros/src/arx5_pickplace/arx5_moveit/scripts
fi

# export ros ws path
source ./ros/devel/setup."${shtype}" || { echo -e "${FAILED}Please use 'source' cmd to run this setup.sh file!${DEF}" && exit 0; }

# OK
echo -e "${OK}OK!${DEF}"