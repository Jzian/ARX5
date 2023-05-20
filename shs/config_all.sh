#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# get the shell type
str="${SHELL}";shtype=${str##*/};name=${shtype}"rc";
in_docker="$(grep docker < /proc/1/cgroup)"
# check the ubuntu version
if [ "$( grep "20.04.1-Ubuntu" < /proc/version )" == "" ];then echo -e "${FAILED}Please make sure your OS is Ubuntu 20.04.1 LTS." && exit 0 ;fi

# check whether ROS and IsaacSim have been installed, and whether the ROS has been correctly sourced
if [ "${ROS_ROOT}" == "" ];then
	if [ -d /opt/ros/noetic/share/ros ];then
		echo -e "${FAILED}Please check whether you have correctly sourced the /opt/ros/noetic/setup.zsh directory in this terminal or in your .*rc file.${name} ${DEF}" && exit 0
	else echo -e "${FAILED}Please install ROS-Noetic first.${DEF}" && exit 0
	fi
fi
if [ "${in_docker}" == "" ];then
	ISAACSIM_PATH=~/.local/share/ov/pkg/isaac_sim-2022.2.0
	if [ ! -d "${ISAACSIM_PATH}" ];then echo -e "${FAILED}Please install the IsaacSim 2022.2.0 first.${DEF}" && exit 0 ;fi
else
	ISAACSIM_PATH=/isaac-sim
	if [ ! -d ${ISAACSIM_PATH} ];then echo -e "${FAILED}Please install the IsaacSim 2022.2.0 in the docker first.${DEF}" && exit 0 ;fi
fi

# check whether the apt source has changed,if not,then ask and change it
if [ "$(grep -e "aliyun" -e "tsinghua" -e "huawei" -e "xjtu" -e "ustc.edu" -e "sdu.edu" /etc/apt/sources.list )" == "" ];then
	read -r -p "WARNING: Your apt source may have not be changed to any of aliyun, tsinghua, huawei, xjtu, ustc and sdu source, 
	if you are sure you have changed the source to other sources, you can ignore this warning and enter 'N/n', 
	but if you haven't changed the source, you can enter 'Y/y' and aliyun will be used as the new source.
	If you want to use another one, just enter 'Ctrl+C' to stop this process.(Y/N)" yn
	if [ "${yn}" == "Y" ] || [ "${yn}"  == "y" ] || [ "${yn}"  == "" ]; then
		sudo sed -i "s@http://.*archive.ubuntu.com@http://mirrors.aliyun.com@g" /etc/apt/sources.list
		sudo sed -i "s@http://.*security.ubuntu.com@http://mirrors.aliyun.com@g" /etc/apt/sources.list
		echo -e "${OK}Changed the apt source to aliyun.${DEF}"
	fi
fi

# if there is a conda environment then deacticate it, which is only effecive in this .sh script
if [ -d ~/anaconda3/etc/profile.d/ ] && [ "${CONDA_DEFAULT_ENV}" != "" ];then
	#shellcheck source=~/anaconda3/etc/profile.d/conda.sh
	source ~/anaconda3/etc/profile.d/conda.sh
	conda deactivate
fi

# check whether pip has installed and has changed the source,if not,then change it to tsinghua source
sudo apt-get update
if [ "$(grep -i "index-url" ~/.config/pip/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" ~/.pip/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" ~/anaconda3/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" /etc/pip.conf 2>/dev/null )" == "" ];then
	sudo apt-get install python3-pip || { echo -e "${FAILED}Install pip3 failed.${DEF}" && exit 0; }
	pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && echo -e "${OK}changed the pip source to tsinghua.${DEF}"
fi

# install the required python packages
pip3 install scipy || { echo -e "${FAILED}Python packages install failed${DEF}" && exit 0; }

# install the V4L2 API for Linux or the FPS of the USB camera will be extremely low when using OpenCV to capture the video
if [ "${in_docker}" == "" ]; then sudo apt-get install v4l-utils || echo -e "${WARN}Install v4l-utils failed which may affect your real robot vision task on Linux OS.${DEF}";fi

# install the required ROS packages through rosdep
rosdep install --from-path ros/src --ignore-src -r -y > /dev/null 2>&1 || { pip3 install rosdepc && sudo rosdepc init > /dev/null 2>&1 && rosdepc update > /dev/null 2>&1; }
rosdep install --from-path ros/src --ignore-src -r -y || { echo -e "${FAILED}rosdep install failed, please check your rosdep.${DEF}" && exit 0; }

# install the other required none-ros packages through apt
if [ "${in_docker}" == "" ]; then sudo apt-get install liborocos-kdl1.4; fi

# build the ros workspace
sudo apt-get install python3-catkin-tools  # https://zhuanlan.zhihu.com/p/399753815
cd ros || exit 0
if [ "${in_docker}" == "" ];then catkin clean -y && catkin build || { echo -e "${FAILED}ROS workspace build failed${DEF}" && exit 0; }
else catkin build || { echo -e "${FAILED}ROS workspace build failed${DEF}" && exit 0; }
fi
cd ..

# auto modify the .bashrs or .zshrc file
{ echo "export ISAACSIM_PATH=${ISAACSIM_PATH}";echo "export ISAACSIM_PYTHON_EXE=${ISAACSIM_PATH}/python.sh";echo "source $(pwd)/ros/devel/setup.${shtype}";} >> ~/."${name}"
if [ "${in_docker}" == "" ];then
	{ echo "export PYTHONPATH=${PYTHONPATH}:${PWD}/ros/src/arx5_pickplace/arx5_vision/scripts/:${PWD}/ros/src/arx5_pickplace/arx5_moveit/scripts/"; } >> ~/."${name}"
fi

# some notes
echo -e "${WARN}NOTE: Please don't often change the directory at will and it's recomended to put the folder in ${HOME} though you can actually put it anywhere,if so,
 you should run config.sh again (which won't remove the obvious config so that redundant codes will accumulate in your .*rc file) 
 or you can just modify the setup.sh path of this ROS workspace in your .*rc file if you know how to do it!${DEF}"
echo "${OK}Config All OK!${DEF}"