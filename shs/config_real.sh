#!/bin/bash

OK='\e[1;32m'
DEF='\e[0m'

# install the V4L2 API for Linux or the FPS of the USB camera will be extremely low when using OpenCV to capture the video
sudo apt-get install v4l-utils || echo -e "${WARN}Install v4l-utils failed which may affect your real robot vision task on Linux OS.${DEF}"
# real control
pip3 install pyserial  || { echo -e "${FAILED}pyserial install failed.${DEF}" && exit 0; }

echo -e "${OK}OK!${DEF}"