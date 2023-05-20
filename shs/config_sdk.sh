#!/bin/bash

OK='\e[1;32m'
DEF='\e[0m'

sudo apt-get install liborocos-kdl1.4
# # build
# cd sdk || exit 0
# mkdir build
# cd build || exit 0
# cmake ..
# make

echo -e "${OK}OK!${DEF}"