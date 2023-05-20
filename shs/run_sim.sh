#!/bin/bash

#note:
	# the 1 param is whether to use rviz, which can be '--rviz' or ''(empty)
	# the 2 param is the kind of the sim scene, which can be '-s' or ''
	# the 3 param is the headless mode, which can be '--headless' or ''

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# check whether another sim is running
pid=$(pgrep arx5s.py | grep -v grep | awk '{print $2}')
if [[ -n $pid ]]; then
	read -r -p "Detect that the sim is running,you'd better kill it to reduce the burden of your computer,do you wan't to kill it?(Y/N)" yn
	if [ "${yn}" == "Y" ] || [ "${yn}"  == "y" ] || [ "${yn}"  == "" ]; then kill "$pid"
	else echo "You choose don't to kill the sim.The process will still continue." && sleep 1
	fi
fi

# detect the rosmaster
if rostopic list > /dev/null 2>&1; then
 	read -r -p "Detect that rosmaster is running,you should kill it first,do you wan't to kill it now and continue the progress?(Y/N)" yn
	if [ "${yn}" != "Y" ] && [ "${yn}"  != "y" ] && [ "${yn}"  != "" ]; then echo "Please kill rosmaster and then try again." && exit 0
	else
		rosnode kill -a  > /dev/null 2>&1
		killall rosmaster  > /dev/null 2>&1
	fi
fi
sleep 1

# change the description file
rm -f "$(pwd)"/ros/src/arx5/arx5_description
ln -s "$(pwd)"/ros/descriptions/arx5_description_pick_place "$(pwd)"/ros/src/arx5/arx5_description

# start roscore and moveit
echo -e "${OK}Start roscore and MoveIt!.${DEF}"
if [ "$1" == '--rviz' ];then
	# start roscore and moveit
	roslaunch arx5_moveit arx5_moveit.launch use_sim:=true use_rviz:=true  2> >( grep -v "WARN" > /dev/null ) &
else
	roslaunch arx5_moveit arx5_moveit.launch use_sim:=true use_rviz:=false 2> >( grep -v "WARN" > /dev/null ) &
fi
sleep 5  # need to sleep for a while or the sim may run incorrectly

# start the sim script(if in docker, use headless mode )
echo -e "${OK}Start Isaac Sim.${DEF}"
if [ "$1" == '-s' ] || [ "$2" == '-s' ];then sim='-nb -gm 1'; fi
if [ "$1" == '--headless' ] || [ "$2" == '--headless' ] || [ "$3" == '--headless' ];then hd='--headless'; fi
filter_cmd="| grep -v 'Warning'"
python_cmd="${ISAACSIM_PYTHON_EXE} ./sim/arx5s.py \"$sim\" \"$hd\""
eval "${python_cmd} ${filter_cmd}"

# check and kill the rosmaster(all the shutdow that is not triggerd by the shell won't auto kill the async progress such as the rosmaster there until the terminal is completely closed so we need to make sure to kill it because we don't like to often close the shell window)
sleep 2
if rostopic list > /dev/null 2>&1;then
	rosnode kill -a  > /dev/null 2>&1
	killall rosmaster  > /dev/null 2>&1
	echo "rosmater down"
	echo "All Closed." && exit 0
fi