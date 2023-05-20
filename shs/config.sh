#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# check the ubuntu version
if [ "$( grep "20.04.1-Ubuntu" < /proc/version )" == "" ];then
	if [ "$( grep "20.04" < /proc/version )" == "" ];then
		echo -e "${WARN}Please make sure your OS is Ubuntu 20.04 LTS.${DEF}"
	fi
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
	# # check the python environment(conda环境下可能必须加入这段)
	# if [ "$(grep usr/bin/pyt | echo $PYTHONPATH)" ];then echo "export $PYTHONPATH=$PYTHONPATH:usr/bin/pyt";fi
fi

# check whether pip has installed and has changed the source,if not,then change it to tsinghua source
sudo apt-get update
if [ "$(grep -i "index-url" ~/.config/pip/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" ~/.pip/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" ~/anaconda3/pip.conf 2>/dev/null )" == "" ] && [ "$(grep -i "index-url" /etc/pip.conf 2>/dev/null )" == "" ];then
	sudo apt-get install python3-pip || { echo -e "${FAILED}Install pip3 failed.${DEF}" && exit 0; }
	pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && echo -e "${OK}changed the pip source to tsinghua.${DEF}"
fi
sudo apt-get install gedit

# some notes
echo -e "${WARN}NOTE: Please don't often change the directory of this folder at will.${DEF}"
echo -e "${OK}Basic config OK!${DEF}"