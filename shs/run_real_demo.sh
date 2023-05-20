#!/bin/bash

# note:
	# the name of the file will be changed into vision adjust alike 
	# both real and sim can use this script in the future but now only real due to the B_C script only support real

# detect whether the rosmaster is running
if rostopic list > /dev/null 2>&1; then echo "rosmaster ok"
else
	if [ "$1" == '-s' ];then echo "Please run run_sim.sh first." && exit 0
	else echo "Please run run_real.sh first." && exit 0
	fi
fi

# start vision
if [ "$1" != '-s' ];then
	rosrun arx5_vision Best_Contour.py -r -t "$1" &
else
	rosrun arx5_vision Best_Contour.py -s -t "$2" &  # 1是-s(表示使用仿真），2是pi0 pi1 pl0 pl1(或无-s顺延)，或任何一个不是no的字符，如t
fi
sleep 3

# start keyboard
if [ "$1" != '-s' ];then
	rosrun arx5_moveit arx5_keyboard.py  # 启动实机键控程序，机械臂自动移动到pick姿态
else
	rosrun arx5_moveit arx5_keyboard.py -s  # 启动仿真键控程序
fi

# always manually kill the async node no matter it has benn killed or not
rosnode kill "/arx5_cube_detect"  > /dev/null 2>&1  # The node name is defined in the Python script: arx5_moveit_follow.py.