# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.1.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

import SoloPy as solo
import time

# sudo ip link set can0 up type can bitrate 1000000
# mySolo = solo.SOLOMotorControllersCanopen(0,solo.CAN_BUS_BAUD_RATE.RATE_1000 )

mySolo = solo.SoloMotorControllerUart("/dev/ttyS0",0, UART_BAUD_RATE.RATE_937500,1)

while True:
    a, b = mySolo.get_bus_voltage()
    print(a, b)
    time.sleep(1)
