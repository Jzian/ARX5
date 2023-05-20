#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'

# export Isaac Sim path
if [ -d "${HOME}"/.local/share/ov/pkg/isaac_sim-2022.2.0 ];then
	ISAACSIM_PATH=${HOME}/.local/share/ov/pkg/isaac_sim-2022.2.0
elif [ -d /isaac-sim ];then
	ISAACSIM_PATH=/isaac-sim
else echo -e "${FAILED}Please check your installation of Isaac Sim!${DEF}" && exit 0
fi
export ISAACSIM_PATH
export ISAACSIM_PYTHON_EXE=${ISAACSIM_PATH}/python.sh

# OK
echo -e "${OK}Set up sim OK!${DEF}"