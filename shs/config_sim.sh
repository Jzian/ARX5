#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# get the shell type
str="${SHELL}";shtype=${str##*/};name=${shtype}"rc";
in_docker="$(grep docker < /proc/1/cgroup)"

if [ "${in_docker}" == "" ];then
	ISAACSIM_PATH=~/.local/share/ov/pkg/isaac_sim-2022.2.0
	if [ ! -d "${ISAACSIM_PATH}" ];then
		ISAACSIM_PATH=~/.local/share/ov/pkg/isaac_sim-2022.2.1
		if [ ! -d "${ISAACSIM_PATH}" ];then
			echo -e "${FAILED}Please install the IsaacSim 2022.2.1 first.${DEF}" && exit 0 ;fi
		fi
else
	ISAACSIM_PATH=/isaac-sim
	if [ ! -d ${ISAACSIM_PATH} ];then echo -e "${FAILED}Please install the IsaacSim 2022.2.0 in the docker first.${DEF}" && exit 0 ;fi
fi

# auto modify the .bashrs or .zshrc file
{ echo "export ISAACSIM_PATH=${ISAACSIM_PATH}";echo "export ISAACSIM_PYTHON_EXE=${ISAACSIM_PATH}/python.sh";} >> ~/."${name}"

ln -s "$( pwd )"/sim  "$ISAACSIM_PATH"/standalone_examples/api/ARX5-SIM

# some notes
echo -e "${OK}Sim config OK!${DEF}"