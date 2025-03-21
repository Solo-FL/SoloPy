# Copyright: (c) 2021-present, SOLO motor controllers project
# MIT License (see LICENSE file for more details)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2025
# Code version: 4.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

# EXAMPLE of how read the SOLO Battery or Supply Input Voltage, 
# every second we print the value of it.

import SoloPy as solo

import time

# instanciate a SOLO object:
mySolo = solo.SoloMotorControllerUart("/dev/ttyUSB0", 0, solo.UartBaudRate.RATE_937500)

# loop actions
while True:
    # reading
    busVoltage, error = mySolo.get_bus_voltage()

    # print
    print("Read from SOLO: " + str(busVoltage))
    print("Error: " + str(error))

    time.sleep(1)
