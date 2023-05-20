#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'

# get the shell type
str="${SHELL}" ; shtype=${str##*/};

# export ros ws path
source ./ros/devel/setup."${shtype}" || { echo -e "${FAILED}Please use 'source' cmd to run this setup.sh file!${DEF}" && exit 0; }

# OK
echo -e "${OK}OK!${DEF}"