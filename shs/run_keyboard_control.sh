#!/bin/bash

# detect the rosmaster whether is running
if rostopic list > /dev/null 2>&1; then
	echo "rosmaster ok"
else echo "Please run run_sim.sh or run_real.sh first" && exit 0
fi

rosrun arx5_moveit arx5_keyboard.py