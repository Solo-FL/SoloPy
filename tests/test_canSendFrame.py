# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

import time
import can

bustype = 'socketcan'
channel = 'can0'
bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=500000)

msg = can.Message(arbitration_id=0x601,
                  data=[0x22, 0x10, 0x30, 0x00, 0x98, 0x13, 0x00, 0x00],
                  is_extended_id=False)
while True:
    bus.send(msg)
    time.sleep(1)
