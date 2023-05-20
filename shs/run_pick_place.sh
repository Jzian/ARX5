#!/bin/bash

# detect the rosmaster whether is running
if rostopic list > /dev/null 2>&1; then
	echo "rosmaster ok"
else echo "Please excute run_sim.sh first" && exit 0
fi
# detect whether the sim is ready
if [ "$1" != 'real' ] && [ "$2" != 'real' ] && [ "$3" != 'real' ] && [ "$4" != 'real' ];then
	while [ "$(rosparam list | (grep /cube_delta_z))" == "" ]
	do
		echo "Wait for the sim to be really ready."
		sleep 5
	done
fi
sleep 1

# start follow script
if [ "$1" == '' ] || [ "$1" == '-ns' ];then
	rosrun arx5_moveit arx5_moveit_follow.py &
elif [ "$1" == 'real' ];then
	rosrun arx5_moveit arx5_moveit_follow.py -r -nt &
elif [ "$1" == 'adjust' ];then  # ./shs/run_pick_place.sh adjust 0/1 real；adjust -- cp
	if [ "$3" == 'real' ];then
		rosrun arx5_moveit arx5_moveit_follow.py -r -nt -cp "$2" &
	else
		rosrun arx5_moveit arx5_moveit_follow.py -cp "$2" &
	fi
elif [ "$1" == 'go' ];then
	rosparam set control_param1 "$2"
	rosparam set control_param2 "$3"
	if [ "$4" == 'real' ];then
		rosrun arx5_moveit arx5_moveit_follow.py -r -nt -cp 2 &
	else
		rosrun arx5_moveit arx5_moveit_follow.py -cp 2 &
	fi
fi
sleep 3

# start vision script
if [ "$1" == '' ];then
	rosrun arx5_vision Best_Contour.py
elif [ "$1" == '-ns' ];then
	rosrun arx5_vision Best_Contour.py -ns
elif [ "$1" == 'real' ] || [ "$3" == 'real' ] || [ "$4" == 'real' ];then
	rosrun arx5_vision Best_Contour.py -r -ng
else
	rosrun arx5_vision Best_Contour.py -hsv 'use'
fi

# always manually kill the async node no matter it has benn killed or not
rosnode kill "/arx5_pick_place"  > /dev/null 2>&1