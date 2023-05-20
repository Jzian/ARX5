#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# get the shell type
str="${SHELL}";shtype=${str##*/};name=${shtype}"rc";
in_docker="$(grep docker < /proc/1/cgroup)"

# check whether ROS has been installed, and whether the ROS has been correctly sourced
if [ "${ROS_ROOT}" == "" ];then
	if [ -d /opt/ros/noetic/share/ros ];then
		echo -e "${FAILED}Please check whether you have correctly sourced the /opt/ros/noetic/setup.*sh directory in this terminal or in your .*rc file.${name} ${DEF}" && exit 0
	else echo -e "${FAILED}Please install ROS-Noetic first.${DEF}" && exit 0
	fi
fi

# init the description file
rm -f "$(pwd)"/ros/src/arx5/arx5_description
ln -s "$(pwd)"/ros/descriptions/arx5_description "$(pwd)"/ros/src/arx5/arx5_description

# if there is a conda environment then deacticate it, which is only effecive in this .sh script
if [ -d ~/anaconda3/etc/profile.d/ ] && [ "${CONDA_DEFAULT_ENV}" != "" ];then
	#shellcheck source=~/anaconda3/etc/profile.d/conda.sh
	source ~/anaconda3/etc/profile.d/conda.sh
	conda deactivate
fi

# install the required ROS packages through rosdep
pip3 install rosdepc || { echo -e "${FAILED}Failed to install rosdepc, please try to use this cmd to install again: wget http://fishros.com/install -O fishros && bash fishros, tips: enter 3 to choose to install rosdepc..${DEF}" && exit 0; }
output=$(rosdepc init 2>&1)
if echo "$output" | grep -q "找不到命令" || echo "$output" | grep -q "command not found"; then
    echo -e "${FAILED}Failed to install rosdepc completely, please try to use this cmd to install again: wget http://fishros.com/install -O fishros && bash fishros, tips: enter 3 to choose to install rosdepc.${DEF}" && exit 0
fi
{ { sudo rosdepc init > /dev/null 2>&1 || rosdepc init > /dev/null; } && rosdepc update; } > /dev/null || { echo -e "${FAILED}rosdepc failed to be ready.${DEF}" && exit 0; }
rosdep install --from-path ros/src --ignore-src -r -y || { echo -e "${WARN}rosdep install failed(not complete), please check your rosdep.${DEF}";rosdepwarn=1; }

# install the required python packages
pip3 install scipy || { echo -e "${FAILED}scipy install failed.${DEF}" && exit 0; }

# install catkin tools
sudo apt-get install python3-catkin-tools 2> /dev/null || pip3 install catkin-tools catkin-tools-python 2> /dev/null

# build the ros workspace
cd ros || exit 0
if [ "${in_docker}" == "" ];then catkin clean -y && catkin build || { echo -e "${FAILED}ROS workspace build failed${DEF}" && exit 0; }
else catkin build || { echo -e "${FAILED}ROS workspace build failed.${DEF}" && exit 0; }
fi
cd ..

# auto modify the .bashrs or .zshrc file
echo "source $(pwd)/ros/devel/setup.${shtype}" >> ~/."${name}"

# OK
if [ "$rosdepwarn" == 1 ];then echo -e "${OK}ROS config OK with warning.${DEF}"
else echo -e "${OK}ROS config OK.${DEF}"
fi