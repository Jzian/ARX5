#!/bin/bash

# if there is a conda environment then deacticate it, which is only effecive in this .sh script
if [ -d ~/anaconda3/etc/profile.d/ ] && [ "${CONDA_DEFAULT_ENV}" != "" ];then
	#shellcheck source=~/anaconda3/etc/profile.d/conda.sh
	source ~/anaconda3/etc/profile.d/conda.sh
	conda deactivate
fi

# init the description file
rm -f "$(pwd)"/ros/src/arx5/arx5_description
ln -s "$(pwd)"/ros/descriptions/arx5_description "$(pwd)"/ros/src/arx5/arx5_description

# update python packages
nowpath=$( pwd )
cd ./ros/src/arx5/arx5_vision || exit
python3 setup.py build
sudo python3 setup.py install
cd "$nowpath"  || exit
cd ./ros/src/arx5/arx5_moveit  || exit
python3 setup.py build
sudo python3 setup.py install
cd "$nowpath" || exit

# update ros workspace
in_docker="$(grep docker < /proc/1/cgroup)"
cd ros || exit 0
if [ "${in_docker}" == "" ];then catkin clean -y && catkin build || { echo -e "${FAILED}ROS workspace build failed${DEF}" && exit 0; }
else catkin build || { echo -e "${FAILED}ROS workspace build failed${DEF}" && exit 0; }
fi
cd ..

echo "update ok"