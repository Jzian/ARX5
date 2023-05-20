#!/usr/bin/env python3
# -- coding: utf-8 --

import can
import os
import time

class CAN_BUS:
    def __init__(self, bustype='socketcan', channel='can0', bitrate=1000000):
        os.system("sudo ip link set can0 up type can bitrate 1000000")
        self._can_bus = can.Bus(bustype=bustype, channel=channel, bitrate=bitrate)

    def terminate(self):
        os.system("sudo ip link set can0 down")
 
can_bus = CAN_BUS()
