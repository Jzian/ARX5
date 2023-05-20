#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# check whether sim is running
pid=$(pgrep arx5s.py | grep -v grep | awk '{print $2}')
if [[ -n $pid ]]; then
	read -r -p "Detect that the sim is running, you need to kill it if it is running the same task to avoid the conflict between sim and real, do you wan't to kill it?(Y/N)" yn
	if [ "${yn}" == "Y" ] || [ "${yn}"  == "y" ] || [ "${yn}"  == "" ]; then kill "$pid"
	else echo -e "${WARN}You choose don't to kill the sim, the process will still be running.${DEF}" && sleep 1
	fi
fi

# detect the rosmaster
if rostopic list > /dev/null 2>&1; then
 	read -r -p "Detect that rosmaster is running,you should kill it first, do you wan't to kill it now and continue the progress?(Y/N)" yn
	if [ "${yn}" != "Y" ] && [ "${yn}"  != "y" ] && [ "${yn}"  != "" ]; then
		echo -e "${FAILED}Please kill rosmaster and then try again.${DEF}" && exit 0
	else
		rosnode kill -a  > /dev/null 2>&1
		killall rosmaster  > /dev/null 2>&1
	fi
fi
sleep 1

# change the description file
if [ "$1" != '-v4' ] && [ "$2" != '-v4' ] && [ "$3" != '-v4' ] ;then
	rm -f "$(pwd)"/ros/src/arx5/arx5_description
	ln -s "$(pwd)"/ros/descriptions/arx5_description "$(pwd)"/ros/src/arx5/arx5_description
else
	rm -f "$(pwd)"/ros/src/arx5/arx5_description
	ln -s "$(pwd)"/ros/descriptions/arx5_description_pick_place "$(pwd)"/ros/src/arx5/arx5_description
fi

# enable the usb and can port
if [ "$1" != '-ng' ];then
	output=$(lsusb | grep 'ID 1a86:7523')
	if [ -z "$output" ]; then echo -e "${FAILED}Please connect the gripper to your PC.${DEF}" && exit 0
	else
		sudo chmod a+rw /dev/ttyUSB*  # enable all connected USB devices
		echo -e "${OK}Use gripper.${DEF}"
	fi
fi

output=$(sudo ip link set can0 up type can bitrate 1000000 2>&1)
if echo "$output" | grep -q "Cannot find device"; then
	echo -e "${FAILED}Cannot find device can0, please check whether you connect yout PC with the arm.${DEF}" && exit 0
elif echo "$output" | grep -q "busy"; then
    echo -e "${OK}CAN already connected.${DEF}"
else echo -e "${OK}Successfully connected to CAN.${DEF}"
fi

# start roscore and moveit
if [ "$1" == '--drag' ] || [ "$2" == '--drag' ] || [ "$3" == '--drag' ] || [ "$4" == '--drag' ];then
	drag_="true"
else drag_="false"
fi

if [ "$1" == '--rviz' ] || [ "$2" == '--rviz' ];then
	roslaunch arx5_moveit arx5_moveit.launch use_sim:=false use_rviz:=true drag:="$drag_" 2> >( grep -v "The complete state" > /dev/null )
else
	roslaunch arx5_moveit arx5_moveit.launch use_sim:=false use_rviz:=false drag:="$drag_" 2> >( grep -v "The complete state" > /dev/null )
fi

# check and kill the rosmaster(all the shutdow that is not triggerd by the shell won't auto kill the async progress such as the rosmaser there so we need to make sure to kill it)
if rostopic list > /dev/null 2>&1;then
	rosnode kill -a  > /dev/null 2>&1
	killall rosmaster  > /dev/null 2>&1
	echo "rosmater down"
fi