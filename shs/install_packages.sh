#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# if there is a conda environment then deacticate it, which is only effecive in this .sh script
if [ -d ~/anaconda3/etc/profile.d/ ] && [ "${CONDA_DEFAULT_ENV}" != "" ];then
	#shellcheck source=~/anaconda3/etc/profile.d/conda.sh
	source ~/anaconda3/etc/profile.d/conda.sh
	conda deactivate
fi

# install the ARX5 packages
nowpath=$( pwd )
cd ./ros/src/arx5/arx5_vision || exit 0
python3 setup.py build
sudo python3 setup.py install
cd "$nowpath"  || exit 0
cd ./ros/src/arx5/arx5_moveit  || exit 0
python3 setup.py build
sudo python3 setup.py install
cd "$nowpath" || exit 0

# install the required ROS packages through rosdep
rosdep install --from-path ros/src --ignore-src -r -y || { echo -e "${WARN}rosdep install failed(not complete), please check your rosdep.${DEF}";rosdepwarn=1; }

# ok
if [ $rosdepwarn == 1 ];then echo -e "${OK}OK, all requied packages have been installed except all the packages rosdep installed.${DEF}"
else echo -e "${OK}OK, all requied packages have been installed.${DEF}"
fi